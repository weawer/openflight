"""The self-trigger's decision on recorded captures, for judging firmware builds.

Each capture is replayed at the kiosk's trigger settings through the compiled
firmware modules (``firmware_replay.replay_dump``). The outcome says whether
the trigger fired, on which frame and by which rule (the range impact's
``crossing`` or ``end``, or the ball-leave fallback), and, for a range-impact
fire, what the club-in estimate and the approach behind it looked like when
the rule armed: the evidence that tells a downswing from a club at address.

The capture sets are the labelled swings (``tests/radar/recordings``, and the
golfer-in-the-approach folder beside them) and the bench sessions in
``BENCH_SETS``, whose dumps were frozen by the board's own fires, true and
false. A dump holds only the frames the ring kept before the freeze (9 at the
2 ms profile, 18 ms), so a fire that needed a longer track history does not
reproduce: a replay that fires is evidence, one that does not is not.

``scripts/analysis/compare_trigger_builds.py`` runs this file inside a
worktree of each git ref it is given, so it may only use replay APIs that
older builds have too (``replay_dump``, ``ReplayConfig``,
``reviewed_recordings``, ``l3_impact_update_range``'s arguments).

    uv run python -m openflight.iwr6843.trigger_eval [--group NAME] [--json]
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

from openflight.iwr6843 import firmware_host as fw, firmware_replay as fr, label_scoring as ls
from openflight.iwr6843.monitor import SELF_TRIGGER_TEE_LEAD_BINS
from openflight.iwr6843.self_trigger import FIRMWARE_TRIGGER_DEFAULT_SNR, TEE_BAND_DEFAULT_BINS

REPO_ROOT = Path(__file__).resolve().parents[3]

# An approach run walks back from the arming point while each older point is
# farther from the ball; a coast of up to this many frames does not end it.
APPROACH_MAX_COAST_FRAMES = 2


@dataclass(frozen=True)
class BenchSet:
    """Dumps from one bench session, replayed at the trigger bin it ran with."""

    name: str
    directory: str  # relative to the repo root
    trigger_bin: int  # the session's ``triggerCfg`` bin
    note: str


BENCH_SETS: tuple[BenchSet, ...] = (
    BenchSet(
        "bench_2026-10-04",
        "tests/radar/trackman_20261004_095102/iwr6843",
        36,
        "Robinante TrackMan session (firmware 1.0.2, triggerCfg 36 1.0 1): 11 on-time "
        "captures and 15 that fired 0.5-0.8 s early at takeaway; match_status from "
        "trackman_openflight_aligned.csv",
    ),
    BenchSet(
        "bench_2026-10-03",
        "openflight_sessions/backswing_2ms_20261003",
        42,
        "2 ms backswing session (triggerCfg 42 1.0 1): kiosk_2ms_capture.log has the OPS "
        "verdict per trigger; 13 of the 18 found no ball",
    ),
)


@dataclass(frozen=True)
class TriggerOutcome:  # pylint: disable=too-many-instance-attributes
    """What the self-trigger did on one capture."""

    name: str
    fired_frame: int | None
    rule: str | None  # "crossing", "end" or "leave"; None when nothing fired
    # For a range-impact fire, at the point that armed it (the newest point
    # before an end fire, the fire frame's point for a crossing):
    club_in_points: int | None = None
    club_in_speed_mps: float | None = None
    gap_m: float | None = None  # the ball's range less the arming point's
    approach_points: int | None = None
    approach_travel_m: float | None = None  # toward the ball over the run
    approach_span_ms: float | None = None
    launch_offset: int | None = None  # fired frame - labelled launch frame
    status: str = ""  # the capture's truth where one is known


@dataclass
class _ImpactCall:
    """One ``l3_impact_update_range`` call, as the replay made it."""

    time_us: int
    fit_ok: bool
    fit_points: int
    fit_speed_mps: float
    appended: bool
    gap_m: float | None
    point_us: int | None
    fired: bool
    cause: int


@dataclass
class _ImpactSpy:
    """A firmware library whose range-impact calls are recorded on the way through."""

    lib: object
    calls: list[_ImpactCall] = field(default_factory=list)

    def __getattr__(self, name: str):
        attr = getattr(self.lib, name)
        if name != "l3_impact_update_range":
            return attr

        def recorded(impact_ref, club_in_ref, club_ref, now_us):
            fired = attr(impact_ref, club_in_ref, club_ref, now_us)
            # byref() keeps the structure it points at; read it back after the call.
            estimate, club, impact = (
                ref._obj  # pylint: disable=protected-access
                for ref in (club_in_ref, club_ref, impact_ref)
            )
            appended = bool(club.appended)
            self.calls.append(
                _ImpactCall(
                    time_us=int(now_us),
                    fit_ok=fw.FIT_WHY_NAMES[estimate.why] == "ok",
                    fit_points=int(estimate.points),
                    fit_speed_mps=float(estimate.speedMps),
                    appended=appended,
                    gap_m=float(club.ballRangeM - club.rangeM) if appended else None,
                    point_us=int(club.timeUs) if appended else None,
                    fired=bool(fired),
                    cause=int(impact.cause),
                )
            )
            return fired

        return recorded


def kiosk_config(trigger_bin: int, base: fr.ReplayConfig | None = None) -> fr.ReplayConfig:
    """What the kiosk sends for a trigger at ``trigger_bin``: the default snr
    and tee band, no locked ball, nothing forcing impact and no manifest
    overrides, so the self-trigger alone decides when the capture freezes.
    ``base`` keeps a recording's own capture settings (loop period, cal)."""
    return replace(
        base or fr.ReplayConfig(tee_bin=trigger_bin),
        tee_bin=trigger_bin,
        dest_bin=None,
        snr=FIRMWARE_TRIGGER_DEFAULT_SNR,
        band_bins=TEE_BAND_DEFAULT_BINS,
        post_from_frame=None,
        overrides={},
    )


def approach_run(calls: Sequence[_ImpactCall], arm_index: int) -> list[_ImpactCall]:
    """The appended points behind ``calls[arm_index]``, newest first, while
    each older one is farther from the ball; ``APPROACH_MAX_COAST_FRAMES``
    frames without a point do not end the run, a point no farther does."""
    run = [calls[arm_index]]
    coasted = 0
    for call in reversed(calls[:arm_index]):
        if not call.appended:
            coasted += 1
            if coasted > APPROACH_MAX_COAST_FRAMES:
                break
            continue
        if call.gap_m is None or run[-1].gap_m is None or call.gap_m <= run[-1].gap_m:
            break
        run.append(call)
        coasted = 0
    return run


def _arming_index(calls: Sequence[_ImpactCall], fire_index: int, cause: str) -> int | None:
    """The call whose point armed the fire: the fire frame's own point for a
    crossing, else the newest appended point before it."""
    if cause == "crossing" and calls[fire_index].appended:
        return fire_index
    earlier = [i for i in range(fire_index) if calls[i].appended]
    return earlier[-1] if earlier else None


def replay_trigger(raw: bytes, config: fr.ReplayConfig, *, name: str = "", lib=None):
    """Replay one capture and describe the self-trigger's decision."""
    spy = _ImpactSpy(lib or fr._default_library())  # pylint: disable=protected-access
    result = fr.replay_dump(raw, config, lib=spy)
    fired = result.fired_frame
    if fired is None:
        return TriggerOutcome(name=name, fired_frame=None, rule=None)
    if result.range_frame != fired:
        return TriggerOutcome(name=name, fired_frame=fired, rule="leave")
    fire_index = next(i for i, call in enumerate(spy.calls) if call.fired)
    cause = fw.IMPACT_CAUSE_NAMES[spy.calls[fire_index].cause]
    arm_index = _arming_index(spy.calls, fire_index, cause)
    if arm_index is None:
        return TriggerOutcome(name=name, fired_frame=fired, rule=cause)
    arm = spy.calls[arm_index]
    run = approach_run(spy.calls, arm_index)
    return TriggerOutcome(
        name=name,
        fired_frame=fired,
        rule=cause,
        club_in_points=arm.fit_points,
        club_in_speed_mps=round(arm.fit_speed_mps, 2),
        gap_m=round(arm.gap_m, 3),
        approach_points=len(run),
        approach_travel_m=round(run[-1].gap_m - arm.gap_m, 3),
        approach_span_ms=round((arm.point_us - run[-1].point_us) / 1000.0, 1),
    )


def labelled_outcomes(directory: Path) -> list[TriggerOutcome]:
    """Every reviewed swing with a labelled ball, at the kiosk's settings
    aimed SELF_TRIGGER_TEE_LEAD_BINS short of its first labelled point."""
    outcomes = []
    for path, config, labels in ls.reviewed_recordings(directory):
        if not labels.ball:
            continue
        launch = labels.ball[0]
        trigger_bin = int(round(launch.range_bin)) - SELF_TRIGGER_TEE_LEAD_BINS
        outcome = replay_trigger(
            path.read_bytes(), kiosk_config(trigger_bin, config), name=path.name
        )
        offset = None if outcome.fired_frame is None else outcome.fired_frame - launch.frame
        outcomes.append(replace(outcome, launch_offset=offset, status="labelled"))
    return outcomes


def bench_statuses(directory: Path) -> dict[str, str]:
    """Dump name -> ``match_status`` from an aligned TrackMan CSV beside the
    dump folder, when there is one."""
    statuses: dict[str, str] = {}
    for csv_path in sorted(directory.parent.glob("*aligned*.csv")):
        with csv_path.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                if row.get("iwr_dump_path"):
                    statuses[Path(row["iwr_dump_path"]).name] = row.get("match_status", "")
    return statuses


def bench_outcomes(bench: BenchSet, root: Path = REPO_ROOT) -> list[TriggerOutcome]:
    """Every dump of a bench session, at the kiosk's settings for its trigger bin."""
    directory = root / bench.directory
    statuses = bench_statuses(directory)
    config = kiosk_config(bench.trigger_bin)
    return [
        replace(
            replay_trigger(path.read_bytes(), config, name=path.name),
            status=statuses.get(path.name, ""),
        )
        for path in sorted(directory.glob("*.l3dump"))
    ]


def capture_groups(root: Path = REPO_ROOT) -> dict[str, Callable[[], list[TriggerOutcome]]]:
    """Every capture set by name; each replays when called."""
    recordings = root / "tests" / "radar" / "recordings"
    groups: dict[str, Callable[[], list[TriggerOutcome]]] = {
        "labelled": lambda: labelled_outcomes(recordings),
        "golfer": lambda: labelled_outcomes(recordings / "golfer_2026-09"),
    }
    for bench in BENCH_SETS:
        groups[bench.name] = lambda bench=bench: bench_outcomes(bench, root)
    return groups


def evaluate(
    root: Path = REPO_ROOT, groups: Iterable[str] | None = None
) -> dict[str, list[TriggerOutcome]]:
    """Replay the named capture groups (all when None)."""
    available = capture_groups(root)
    names = list(available) if groups is None else list(groups)
    unknown = [name for name in names if name not in available]
    if unknown:
        raise ValueError(f"unknown capture group(s) {unknown}; known: {sorted(available)}")
    return {name: available[name]() for name in names}


def fire_counts(outcomes: Iterable[TriggerOutcome]) -> Counter:
    """How many captures fired by each rule ("none" for no fire)."""
    return Counter(outcome.rule or "none" for outcome in outcomes)


def differences(
    results: Mapping[str, Mapping[str, Sequence[Mapping]]],
) -> list[tuple[str, str, dict[str, tuple]]]:
    """Captures whose (fired frame, rule) is not the same under every ref.

    ``results`` maps a ref to its ``evaluate`` output as JSON (group -> list
    of outcome dicts). Returns (group, capture, {ref: (fired_frame, rule)}),
    in group and capture order; a capture missing under a ref reads
    ("missing", None)."""
    refs = list(results)
    rows = []
    groups = dict.fromkeys(group for ref in refs for group in results[ref])
    for group in groups:
        by_ref = {
            ref: {o["name"]: (o["fired_frame"], o["rule"]) for o in results[ref].get(group, ())}
            for ref in refs
        }
        names = dict.fromkeys(name for ref in refs for name in by_ref[ref])
        for name in names:
            seen = {ref: by_ref[ref].get(name, ("missing", None)) for ref in refs}
            if len(set(seen.values())) > 1:
                rows.append((group, name, seen))
    return rows


def _format_outcome(outcome: TriggerOutcome) -> str:
    text = f"{outcome.name:46} {outcome.status[:24]:24} "
    if outcome.rule is None:
        return text + "no fire"
    text += f"fired {outcome.fired_frame:>2} {outcome.rule:8}"
    if outcome.launch_offset is not None:
        text += f" launch{outcome.launch_offset:+d}"
    if outcome.club_in_points is not None:
        text += (
            f" fit {outcome.club_in_points}pt {outcome.club_in_speed_mps:5.1f} m/s"
            f" gap {outcome.gap_m:.2f} m approach {outcome.approach_points}pt"
            f" {outcome.approach_travel_m:.2f} m over {outcome.approach_span_ms:.0f} ms"
        )
    return text


def main(argv: list[str] | None = None) -> int:
    """Print every capture's outcome, or the whole result as JSON."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    parser.add_argument("--root", type=Path, default=REPO_ROOT, help="repo root holding the data")
    parser.add_argument("--group", action="append", help="capture group(s); default all")
    parser.add_argument("--json", action="store_true", help="machine-readable output")
    args = parser.parse_args(argv)
    try:
        results = evaluate(args.root, args.group)
    except ValueError as error:
        parser.error(str(error))
    if args.json:
        json.dump({g: [asdict(o) for o in outs] for g, outs in results.items()}, sys.stdout)
        print()
        return 0
    for group, outcomes in results.items():
        counts = ", ".join(f"{rule} {n}" for rule, n in sorted(fire_counts(outcomes).items()))
        print(f"== {group} ({len(outcomes)} captures: {counts})")
        for outcome in outcomes:
            print("  " + _format_outcome(outcome))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
