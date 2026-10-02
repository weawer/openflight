#!/usr/bin/env python3
"""Score the firmware's club and ball tracking on recorded captures against OPS.

    uv run python scripts/analysis/evaluate_iwr_tracking.py iwr-test-sessions \
        [--json out.json] [--compare baseline.json] [--allow-more-none N]

Every ``.l3dump`` whose session log (``dump_viewer.session_context``) gives an
OPS ball speed and a ``triggerCfg`` tee bin is replayed through the compiled
firmware as the board ran it: the session's trigger configuration, and ball
tracking from the recorded freeze frame when the dump carries a retention
report. Over those captures it counts:

* club at impact: the club track's last points before the split approach the
  ball at >= 0.5 bins/frame ("club"), do not ("stuck"), or are too few;
* ball vs OPS: the launch speed within 15 % of the OPS ball speed ("ok"),
  outside it ("wrong"), or no launch ("none");
* ball present: the saved post-impact targets hold a chain of >= 3 points in
  consecutive frames moving at 0.7-1.1 x the OPS speed -- whether the data
  contains the ball at all.

The Pi detector's ball rules can be switched on for a run (all off by default,
as the firmware ships): ``--fast-ball`` (m/s, or ``club`` for the session's
club-class floor, ``shot.CLUB_MIN_BALL_MS``) with ``--fast-support``,
``--min-departure-mps`` and ``--far-window-m``. The launch's horizontal
angle is recorded per capture so runs can be compared.

``--impact`` adds the impact-time evaluation (``impact_eval``): the firmware's
three-track fit (method A) against the joint-fit comparator (method C), per
capture and summarised under ``"impact"``. ``--band-bins`` sets the tee band's
total width in bins for every capture, placed by the firmware on the noisiest
idle bins near the tee (unset: the firmware default, off).
"""

from __future__ import annotations

import argparse
import json
import sys
from collections.abc import Iterable, Iterator, Sequence
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

from openflight.iwr6843 import firmware_replay as fr, impact_eval
from openflight.iwr6843.dump import is_range_snapshot, parse_header
from openflight.iwr6843.dump_viewer import (
    ViewerOptions,
    bin_width_m,
    session_context,
    tee_bin_for,
)
from openflight.iwr6843.labels import LabelError, load_labels
from openflight.iwr6843.shot import CLUB_MIN_BALL_MS, club_class

MPH_TO_MPS = 0.44704
BALL_TOLERANCE = 0.15
CLUB_MIN_STEP_BINS = 0.5
CLUB_POINTS = 5
CHAIN_RATE_BOUNDS = (0.7, 1.1)
CHAIN_MIN_POINTS = 3
# The label's own gap and back-projection allowances (spec E1): the tracker's
# impact-region coast and the gate anchor's tolerance.
GAP_MAX_US = 18_000
ANCHOR_TOL_US = 15_000


@dataclass(frozen=True)
class Case:
    """One capture to replay: where it is, how the board ran it, and the truth."""

    path: Path
    config: fr.ReplayConfig
    ops_mps: float
    club: str | None = None  # the session's club for the shot, when logged


@dataclass(frozen=True)
class Outcome:
    """How the firmware did on one capture."""

    name: str
    club: str  # "club", "stuck" or "few"
    ball: str  # "ok", "wrong" or "none"
    ball_present: bool
    ball_present_strict: bool
    launch_mps: float | None
    ops_mps: float
    launch_hla_deg: float | None = None
    impact: impact_eval.ImpactOutcome | None = None  # with --impact
    net_reached: bool | None = None  # with --net-range-m


NET_FRAMES = 2  # the net diagnostic's allowance either side, in frame periods


def net_reached(ball_points, launch_mps, impact_us, tee_m, net_range_m, frame_us) -> bool | None:
    """Spec E3: whether the confirmed ball reaches the net range within
    NET_FRAMES frame periods of when its launch speed says it should. None
    without a launch. A diagnostic only: no firmware code knows the net."""
    if launch_mps is None or launch_mps <= 0.0:
        return None
    due_us = impact_us + (net_range_m - tee_m) / launch_mps * 1e6
    for p in ball_points:
        if p.range_m >= net_range_m - 0.5 * bin_width_m():
            return abs(p.timestamp_us - due_us) <= NET_FRAMES * frame_us
    return False


def club_verdict(points: Sequence, split_frame: int | None) -> str:
    """ "club" when the club track's last points before the split close on the ball."""
    before = [p for p in points if split_frame is None or p.frame < split_frame][-CLUB_POINTS:]
    if len(before) < 3:
        return "few"
    slope = float(np.polyfit([p.frame for p in before], [p.range_bin for p in before], 1)[0])
    return "club" if slope >= CLUB_MIN_STEP_BINS else "stuck"


def ball_verdict(
    launch_mps: float | None, ops_mps: float, tolerance: float = BALL_TOLERANCE
) -> str:
    """The launch speed against OPS: "ok", "wrong", or "none" without a launch."""
    if launch_mps is None:
        return "none"
    return "ok" if abs(launch_mps - ops_mps) <= tolerance * ops_mps else "wrong"


def ball_present(
    frames: Sequence,
    ops_mps: float,
    bin_m: float,
    *,
    max_gap_us: int | None = None,
    anchor: tuple[float, int, int] | None = None,
) -> bool:
    """A chain of at least CHAIN_MIN_POINTS targets moving at the OPS speed.

    Each point links to one earlier point at a rate within CHAIN_RATE_BOUNDS of
    OPS: with max_gap_us None only a point in the immediately preceding frame
    (the strict label), else any point up to max_gap_us earlier. With anchor
    (tee bin, impact us, tolerance us) the line through the chain's first and
    newest points must also pass the tee within the tolerance of the impact."""
    lo, hi = (bound * ops_mps for bound in CHAIN_RATE_BOUNDS)
    # (frame, us, bin, chain length, first us, first bin) per target seen so far
    nodes: list[tuple[int, int, float, int, int, float]] = []
    for frame in frames:
        current = []
        for target in frame.targets:
            best = (1, frame.timestamp_us, target.range_bin)
            for n_frame, n_us, n_bin, n_len, n_first_us, n_first_bin in nodes:
                dt_us = frame.timestamp_us - n_us
                if max_gap_us is None:
                    if n_frame != frame.frame - 1 or dt_us <= 0:
                        continue
                elif not 0 < dt_us <= max_gap_us:
                    continue
                rate = (target.range_bin - n_bin) * bin_m / (dt_us * 1e-6)
                if lo <= rate <= hi and n_len + 1 > best[0]:
                    best = (n_len + 1, n_first_us, n_first_bin)
            length, first_us, first_bin = best
            if length >= CHAIN_MIN_POINTS and _back_projects(
                anchor, first_us, first_bin, frame.timestamp_us, target.range_bin, bin_m
            ):
                return True
            current.append(
                (frame.frame, frame.timestamp_us, target.range_bin, length, first_us, first_bin)
            )
        nodes.extend(current)
        if max_gap_us is not None:
            nodes = [n for n in nodes if frame.timestamp_us - n[1] <= max_gap_us]
    return False


def _back_projects(anchor, first_us, first_bin, last_us, last_bin, bin_m) -> bool:
    """True without an anchor; else whether the line through the two points
    passes the tee bin within the anchor's tolerance of its impact time."""
    if anchor is None:
        return True
    tee_bin, impact_us, tol_us = anchor
    rate_bins_per_us = (last_bin - first_bin) / (last_us - first_us)
    if rate_bins_per_us <= 0.0:
        return False
    cross_us = first_us - (first_bin - tee_bin) / rate_bins_per_us
    return abs(cross_us - impact_us) <= tol_us


def labelled_presence(path: Path) -> bool | None:
    """A reviewed label's answer (a ball with at least CHAIN_MIN_POINTS points),
    None without a reviewed label or when the label file cannot be read."""
    try:
        labels = load_labels(path)
    except LabelError as error:
        print(f"{path.name}: label ignored: {error}", file=sys.stderr)
        return None
    if labels is None or not labels.reviewed:
        return None
    return len(labels.ball) >= CHAIN_MIN_POINTS


def split_frame(result) -> int | None:
    """The first post-impact frame: the recorded freeze, else the frame after the gate."""
    if result.config.post_from_frame is not None:
        return result.config.post_from_frame
    return None if result.fired_frame is None else result.fired_frame + 1


def iter_cases(roots: Iterable[Path]) -> Iterator[Case]:
    """Captures under the roots with an OPS ball speed and the gate the board ran."""
    for root in roots:
        for path in sorted(Path(root).rglob("*.l3dump")):
            context = session_context(path)
            if not context or not context.get("ball_speed_mph"):
                continue
            defaults = context["defaults"]
            if "tee_bin" not in defaults and "tee_range_m" not in defaults:
                continue  # neither triggerCfg nor a slant range: the gate is unknown
            try:
                meta = parse_header(path.read_bytes())
            except ValueError:
                continue
            if not is_range_snapshot(meta):
                continue  # raw ADC samples: nothing for the firmware replay to read
            options = ViewerOptions.for_recording(defaults)
            config = fr.ReplayConfig(
                tee_bin=tee_bin_for(options),
                snr=options.snr,
                pitch_deg=options.pitch_deg,
                post_from_frame=(meta.get("retention") or {}).get("pre_frames"),
            )
            shot = context.get("shot") or {}
            yield Case(
                path,
                config,
                float(context["ball_speed_mph"]) * MPH_TO_MPS,
                club=shot.get("club"),
            )


def club_fast_ball_mps(club: str | None) -> float:
    """The Pi detector's fastest-credible floor for the club's class."""
    return CLUB_MIN_BALL_MS[club_class(club)]


def tuning_for(
    case: Case, tuning: fr.BallTuning | None, *, fast_ball_from_club: bool = False
) -> fr.BallTuning | None:
    """The run's tuning for one capture: the club-class floor filled in when asked."""
    if not fast_ball_from_club:
        return tuning
    return replace(tuning or fr.BallTuning(), fast_ball_mps=club_fast_ball_mps(case.club))


def evaluate(
    case: Case,
    *,
    lib=None,
    ball_hypotheses: bool | None = None,
    tuning: fr.BallTuning | None = None,
    fast_ball_from_club: bool = False,
    band_bins: float | None = None,
    impact: bool = False,
    net_range_m: float | None = None,
) -> Outcome:
    """Replay one capture and judge its club and ball tracks (and its impact
    time, when asked). ``band_bins`` None keeps the case's own tee band."""
    config = case.config
    if ball_hypotheses is not None:
        config = replace(config, ball_hypotheses=ball_hypotheses)
    tuning = tuning_for(case, tuning, fast_ball_from_club=fast_ball_from_club)
    if tuning is not None:
        config = replace(config, ball_tuning=tuning)
    if band_bins is not None:
        config = replace(config, band_bins=band_bins)
    result = fr.replay_dump(case.path.read_bytes(), config, lib=lib)
    split = split_frame(result)
    post = [f for f in result.frames if split is not None and f.frame >= split]
    launch = None if result.launch is None else float(result.launch.speed_mps)
    tee_bin = float(config.dest_bin if config.dest_bin is not None else config.tee_bin)
    impact_us = result.frozen_impact_timestamp_us
    if impact_us is None and post:
        impact_us = post[0].timestamp_us
    anchor = None if impact_us is None else (tee_bin, int(impact_us), ANCHOR_TOL_US)
    labelled = labelled_presence(case.path)
    present = (
        labelled
        if labelled is not None
        else ball_present(post, case.ops_mps, bin_width_m(), max_gap_us=GAP_MAX_US, anchor=anchor)
    )
    net = None
    if net_range_m is not None and len(post) >= 2:
        frame_us = int(np.median(np.diff([f.timestamp_us for f in post])))
        net = net_reached(
            result.ball_points,
            launch,
            int(result.shot.impactTimestampUs),
            tee_bin * bin_width_m(),
            net_range_m,
            frame_us,
        )
    return Outcome(
        name=case.path.name,
        club=club_verdict(result.points, split),
        ball=ball_verdict(launch, case.ops_mps),
        ball_present=present,
        ball_present_strict=ball_present(post, case.ops_mps, bin_width_m()),
        launch_mps=launch,
        ops_mps=case.ops_mps,
        launch_hla_deg=None if result.launch is None else result.launch.hla_deg,
        impact=impact_eval.impact_outcome(case.path.name, result) if impact else None,
        net_reached=net,
    )


def summarize(outcomes: Iterable[Outcome]) -> dict:
    """Counts per category, the numbers the spec's acceptance is written in."""
    outcomes = list(outcomes)

    def count(field: str, values: tuple[str, ...]) -> dict[str, int]:
        return {v: sum(1 for o in outcomes if getattr(o, field) == v) for v in values}

    verdicts = ("ok", "wrong", "none")

    def by(present: bool) -> dict[str, int]:
        return {
            v: sum(1 for o in outcomes if o.ball_present == present and o.ball == v)
            for v in verdicts
        }

    nets = [o.net_reached for o in outcomes if o.net_reached is not None]
    extra = {"net_reached": sum(1 for o in outcomes if o.net_reached)} if nets else {}
    return {
        **extra,
        "captures": len(outcomes),
        "club": count("club", ("club", "stuck", "few")),
        "ball": count("ball", verdicts),
        "ball_present": sum(1 for o in outcomes if o.ball_present),
        "ball_present_strict": sum(1 for o in outcomes if o.ball_present_strict),
        "ball_by_presence": {"present": by(True), "absent": by(False)},
    }


def compare(summary: dict, baseline: dict, *, allow_more_none: int = 0) -> list[str]:
    """Regressions against a baseline summary, one readable line each."""
    if summary["captures"] != baseline["captures"]:
        return [f"captures: {summary['captures']} vs baseline {baseline['captures']}"]
    problems = []
    if summary["club"]["club"] < baseline["club"]["club"]:
        problems.append(
            f"club at impact: {summary['club']['club']} < baseline {baseline['club']['club']}"
        )
    if summary["ball"]["ok"] < baseline["ball"]["ok"]:
        problems.append(f"ball ok: {summary['ball']['ok']} < baseline {baseline['ball']['ok']}")
    if summary["ball"]["none"] > baseline["ball"]["none"] + allow_more_none:
        problems.append(
            f"no launch: {summary['ball']['none']} > baseline {baseline['ball']['none']}"
            f" + {allow_more_none}"
        )
    return problems


LABEL_DIFFERS = "label differs: "


def is_informational(line: str) -> bool:
    """Whether an accept_split line is information (a capture whose run label
    differs from the baseline's) rather than a reason not to accept."""
    return line.startswith(LABEL_DIFFERS)


def _baseline_problem(baseline: dict) -> str | None:
    """Why a baseline JSON cannot be judged against, None when it can."""
    if "summary" not in baseline:
        return "baseline has no summary: write it with --json"
    if "ball_by_presence" not in baseline["summary"] or "club" not in baseline["summary"]:
        return "baseline summary has no ball_by_presence: re-run it under the gap-tolerant label"
    if "outcomes" not in baseline:
        return "baseline has no per-capture outcomes: re-run it with --json"
    for row in baseline["outcomes"]:
        missing = [key for key in ("name", "ball", "ball_present") if key not in row]
        if missing:
            return f"baseline outcome has no {', '.join(missing)}: re-run it with --json"
    return None


def _split_counts(rows: Iterable[tuple[str, bool]]) -> dict[str, dict[str, int]]:
    """(verdict, present) pairs counted per presence and verdict."""
    counts = {side: {v: 0 for v in ("ok", "wrong", "none")} for side in ("present", "absent")}
    for verdict, present in rows:
        counts["present" if present else "absent"][verdict] += 1
    return counts


def _duplicates(names: Sequence[str]) -> list[str]:
    return sorted({n for n in names if names.count(n) > 1})


def accept_split(outcomes: Iterable[Outcome], baseline: dict) -> list[str]:
    """Spec E2 against legacy: the run's verdicts and the baseline's, both split
    by the BASELINE's per-capture ball_present, so a label the run's replay
    moved cannot move a capture between the populations being judged.

    Returns one line per reason not to accept, then one informational line per
    capture whose run label differs from the baseline's (``is_informational``;
    never a failure). Empty: accepted. The capture sets must match."""
    outcomes = list(outcomes)
    problem = _baseline_problem(baseline)
    if problem is not None:
        return [problem]
    rows = baseline["outcomes"]
    run_names = [o.name for o in outcomes]
    base_names = [row["name"] for row in rows]
    for who, names in (("run", run_names), ("baseline", base_names)):
        duplicated = _duplicates(names)
        if duplicated:
            return [f"duplicate capture names in the {who}: {', '.join(duplicated)}"]
    if set(run_names) != set(base_names):
        only_run = sorted(set(run_names) - set(base_names))
        only_base = sorted(set(base_names) - set(run_names))
        return [
            f"captures differ: {len(run_names)} in the run, {len(base_names)} in the baseline;"
            f" only in the run: {', '.join(only_run) or '-'};"
            f" only in the baseline: {', '.join(only_base) or '-'}"
        ]
    label = {row["name"]: bool(row["ball_present"]) for row in rows}
    now = _split_counts((o.ball, label[o.name]) for o in outcomes)
    base = _split_counts((row["ball"], label[row["name"]]) for row in rows)
    problems = []
    if now["present"]["ok"] <= base["present"]["ok"]:
        problems.append(f"ball-present ok {now['present']['ok']} not above {base['present']['ok']}")
    if now["absent"]["none"] <= base["absent"]["none"]:
        problems.append(
            f"ball-absent none {now['absent']['none']} not above {base['absent']['none']}"
        )
    if now["absent"]["wrong"] >= base["absent"]["wrong"]:
        problems.append(
            f"ball-absent wrong {now['absent']['wrong']} not below {base['absent']['wrong']}"
        )
    run_club = sum(1 for o in outcomes if o.club == "club")
    base_club = baseline["summary"]["club"]["club"]
    if run_club < base_club:
        problems.append(f"club at impact {run_club} < baseline {base_club}")
    side = {True: "present", False: "absent"}
    for o in sorted(outcomes, key=lambda o: o.name):
        if bool(o.ball_present) != label[o.name]:
            problems.append(
                f"{LABEL_DIFFERS}{o.name} baseline={side[label[o.name]]}"
                f" run={side[bool(o.ball_present)]}"
            )
    return problems


def on_off(value: str | None) -> bool | None:
    """A CLI on/off switch as a bool; None when not given."""
    return None if value is None else value == "on"


def parse_tuning(args: argparse.Namespace) -> fr.BallTuning | None:
    """The command line's ball-rule overrides; None when none is given."""
    fast = args.fast_ball
    if fast is not None and fast != "club":
        try:
            fast_mps = float(fast)
        except ValueError as error:
            raise SystemExit(f"--fast-ball needs m/s or 'club', got {fast!r}") from error
    else:
        fast_mps = None
    tuning = fr.BallTuning(
        fast_ball_mps=fast_mps,
        fast_support_fraction=args.fast_support,
        min_departure_mps=args.min_departure_mps,
        far_window_m=args.far_window_m,
        corridor_gate=on_off(args.corridor_gate),
        impact_coast_ms=args.impact_coast_ms,
        max_decel_mps2=args.max_decel,
        classify_points=args.classify_points,
        recover=on_off(args.recover),
        recover_gate_m=args.recover_gate_m,
        history_snr=args.history_snr,
    )
    return None if tuning == fr.BallTuning() else tuning


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    parser.add_argument("roots", nargs="+", type=Path, help="Folders searched for .l3dump files")
    parser.add_argument("--json", type=Path, help="Write the summary and every outcome here")
    parser.add_argument("--compare", type=Path, help="A baseline written by --json")
    parser.add_argument("--accept", type=Path, help="Judge against a legacy baseline by spec E2")
    parser.add_argument("--allow-more-none", type=int, default=0)
    parser.add_argument(
        "--ball-hypotheses",
        choices=("firmware", "on", "off"),
        default="firmware",
        help="Ball search: the firmware default, the hypotheses, or the legacy acquisition",
    )
    parser.add_argument(
        "--fast-ball",
        help="Fastest-credible floor for the hypotheses: m/s, or 'club' for the club class",
    )
    parser.add_argument(
        "--fast-support", type=float, help="Share of the most points a fast ball needs"
    )
    parser.add_argument(
        "--min-departure-mps", type=float, help="Hard speed floor for the ball (both searches)"
    )
    parser.add_argument(
        "--far-window-m",
        type=float,
        help="Hypothesis points only this many metres beyond the accept bin",
    )
    parser.add_argument(
        "--corridor-gate", choices=("on", "off"), help="Hypothesis corridor gate on or off"
    )
    parser.add_argument(
        "--impact-coast-ms", type=float, help="How long the impact region may coast, in ms"
    )
    parser.add_argument(
        "--max-decel", type=float, help="Deceleration ceiling for hypothesis tracks, m/s^2"
    )
    parser.add_argument(
        "--classify-points", type=int, help="Points a hypothesis needs before it is classified"
    )
    parser.add_argument("--recover", choices=("on", "off"), help="Ball track recovery on or off")
    parser.add_argument("--recover-gate-m", type=float, help="Recovery gate half-width in metres")
    parser.add_argument(
        "--history-snr", type=float, help="SNR for the history targets (0: same as snr)"
    )
    parser.add_argument(
        "--net-range-m",
        type=float,
        help="Report whether the ball track reaches this range on time (diagnostic)",
    )
    parser.add_argument(
        "--band-bins",
        type=float,
        help="Tee band total width in bins for every capture, placed on the noisiest "
        "idle bins near the tee (unset: the firmware default, off)",
    )
    parser.add_argument(
        "--impact",
        action="store_true",
        help="Also report impact from the tracks either side of the tee band (A vs C)",
    )
    args = parser.parse_args(argv)
    search = {"firmware": None, "on": True, "off": False}[args.ball_hypotheses]
    from_club = args.fast_ball == "club"
    tuning = parse_tuning(args)
    outcomes = [
        evaluate(
            case,
            ball_hypotheses=search,
            tuning=tuning,
            fast_ball_from_club=from_club,
            band_bins=args.band_bins,
            impact=args.impact,
            net_range_m=args.net_range_m,
        )
        for case in iter_cases(args.roots)
    ]
    summary = summarize(outcomes)
    print(json.dumps(summary, indent=2))
    written = {"summary": summary, "outcomes": [asdict(o) for o in outcomes]}
    if args.impact:
        impact = impact_eval.summarize_impact([o.impact for o in outcomes])
        print("impact:")
        print(json.dumps(impact, indent=2))
        written["impact"] = impact
    if args.json is not None:
        args.json.write_text(json.dumps(written, indent=2) + "\n", encoding="utf-8")
    failed = False
    if args.accept is not None:
        baseline = json.loads(args.accept.read_text(encoding="utf-8"))
        for line in accept_split(outcomes, baseline):
            if is_informational(line):
                print(f"NOTE: {line}")
            else:
                print(f"NOT ACCEPTED: {line}")
                failed = True
    if args.compare is not None:
        baseline = json.loads(args.compare.read_text(encoding="utf-8"))["summary"]
        problems = compare(summary, baseline, allow_more_none=args.allow_more_none)
        for problem in problems:
            print(f"REGRESSION {problem}")
        failed = failed or bool(problems)
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
