# IWR6843 Joint Club/Ball Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After impact, identify the ball as its own emerging trajectory — a bounded set of hypotheses near the origin, classified only after several frames, assigned jointly with the continuing club track — instead of acquiring the most confident departing target.

**Architecture:** A new pure-C module `l3_ball_hyp.c` holds up to four candidate ball trajectories, assigns each post-impact frame's targets to them (never the club track's claimed target), and classifies them by origin crossing, fitted range rate, fit residual, Doppler agreement and strength relative to the club. `l3_ball_track.c` gains a switchable path (`useHypotheses`) that searches with the hypotheses and hands the chosen one to its existing track core; the board (`l3_dump.c`) and the host replay (`firmware_replay.py`) make the same calls. An evaluation script scores every recorded capture against OPS so the switch's default is set by data.

**Tech Stack:** C99 firmware modules (host-built with cc/gcc/clang or `zig cc` via the `ziglang` wheel, driven through ctypes), Python 3.11 with `uv`, pytest, numpy, the Flask/Plotly dump viewer.

**Spec:** `docs/superpowers/specs/2026-09-28-iwr-joint-club-ball-tracking.md`

## Global Constraints

- Firmware modules are pure C99 and compile clean under `-std=c99 -Wall -Wextra -Werror` (the flags in `firmware_host.build_firmware_library`).
- No dynamic memory in firmware: every array is fixed-size (`L3_BALL_HYP_MAX = 4`, `L3_BALL_HYP_POINTS = 8`).
- No `%f` in firmware text: floats print through `l3_text_fixed` (the R4F printf subset lacks it).
- DATA_RAM keeps at least 16384 B free (`tests/test_iwr6843_memory_layout.py`, `MIN_DATA_RAM_FREE_BYTES`); the hypotheses add about 1.6 KB to `gBallTrack`.
- Every ctypes mirror in `src/openflight/iwr6843/firmware_host.py` matches its C struct field for field; each new struct gets a `*_struct_bytes()` size check.
- Positive aliased Doppler means receding (verified 2026-09-28 on `synth_shot_dump`: a 19.5 m/s radial ball reads +1.7 to +1.9 m/s on a 17.93 m/s span).
- All Python through `uv run`; lint `uv run ruff check`, `uv run ruff format`, `uv run pylint ... --fail-under=9`.
- Bug reports get a failing test first (CLAUDE.md).
- The board and the replay make the same calls in the same order (spec R7).

## Review Focus

1. **The gate fires before or after the real impact** (tee misconfigured, 1–3 frames off): the ball must still be classified — Task 4 `test_the_gate_need_not_be_the_exact_impact` (±6 ms) and `test_a_ball_leaving_far_from_the_gate_time_is_not_the_ball` (+30 ms).
2. **No ball in the saved data** (70/93 captures): nothing may be classified and there must be no launch, rather than a club launch — Task 4 `test_a_stationary_return_near_the_origin_is_never_the_ball`, Task 5 `test_without_a_ball_there_is_no_launch`.
3. **More targets than hypothesis slots** (eight targets, four slots): the set stays bounded and deterministic and never evicts an established hypothesis — Task 3 `test_a_full_set_evicts_only_a_single_point_hypothesis`.
4. **Uneven frame spacing** (timed dumps): gates and fits follow timestamps — Task 2 `test_follow_caps_the_club_by_elapsed_time_not_frames`, Task 3 `test_uneven_frame_spacing_is_followed_by_time`.
5. **No club track after impact** (club stopped dead or lost): a lone ball is still found — Task 5 `test_a_lone_ball_is_found_without_a_club_track`.

---

## File Structure

| File | Responsibility |
|---|---|
| `scripts/analysis/evaluate_iwr_tracking.py` (create) | Replay every recorded capture with an OPS speed; count club-at-impact, ball-vs-OPS, ball-present; compare with a baseline. |
| `tests/test_evaluate_iwr_tracking.py` (create) | Tests for the evaluation script. |
| `tests/iwr6843_twotrack.py` (create) | Per-frame target lists for "club + ball after impact" scenes, for the C unit tests. |
| `firmware/iwr6843/l3_ball_hyp.h`, `l3_ball_hyp.c` (create) | Bounded ball hypotheses: spawn, joint assignment, coasting, fitting, classification, angles. |
| `tests/test_iwr6843_firmware_ball_hyp.py` (create) | Tests for the hypotheses. |
| `firmware/iwr6843/l3_club_track.h/.c` (modify) | Public `l3_track_wrapped_diff`; `l3_track_follow`'s cap in bins per second. |
| `firmware/iwr6843/l3_ball_track.h/.c` (modify) | `useHypotheses` path, `l3_ball_track_update_joint`, adoption into the core, `searching` state. |
| `firmware/iwr6843/l3_dump.c` (modify) | Fix the duplicate `frame` declaration; call the joint update; angles for hypothesis points. |
| `firmware/iwr6843/makefile` (modify) | Build `l3_ball_hyp.c` on the R4F. |
| `src/openflight/iwr6843/firmware_host.py` (modify) | Host source list, ctypes mirrors, signatures. |
| `src/openflight/iwr6843/firmware_replay.py` (modify) | `ReplayConfig.ball_hypotheses`, joint update, hypothesis angles, per-frame hypothesis summaries. |
| `src/openflight/iwr6843/dump_viewer.py`, `scripts/iwr6843/dump_viewer.html` (modify) | Option and drawing for the hypotheses. |
| `tests/radar/recordings/` (modify) | Add the 2026-08-24 12:04:08 capture and its expectation. |

---

### Task 0: Prerequisites — commit the baseline, fix the board compile error

**Files:**
- Modify: `firmware/iwr6843/l3_dump.c:3544-3640` (`l3_considerBallTrack`)

**Interfaces:**
- Consumes: nothing.
- Produces: a committed starting point; `l3_considerBallTrack` using `frameIndex` for the post-frame number.

- [ ] **Step 1: Ask the user to commit the 2026-09-27/28 work**

The dump viewer, the club-track rules, `l3_track_follow`, the v9 dump parser and the zig compiler fallback are uncommitted on `feat/iwr-calcs`. Ask the user whether to commit them (and on which branch) before starting; do not commit without that go-ahead. Everything below assumes they are committed.

- [ ] **Step 2: Confirm the compile error**

`l3_considerBallTrack` declares `frame` twice in one scope (committed in `1ed49fb2`):

```c
    l3_detect_frame_t frame = l3_detectFrameOf(slot);
    ...
    uint32_t frame;
```

Run: `git show HEAD:firmware/iwr6843/l3_dump.c | grep -n "uint32_t frame;"`
Expected: one line inside `l3_considerBallTrack`. (The host tests never build `l3_dump.c`; the R4F build does.)

- [ ] **Step 3: Rename the post-frame number to `frameIndex`**

In `l3_considerBallTrack` only: replace the declaration `uint32_t frame;` with `uint32_t frameIndex;`, the assignment `frame = gPreFramesCaptured + gPostFramesScored;` with `frameIndex = gPreFramesCaptured + gPostFramesScored;`, and every use of the post-frame NUMBER (not `&frame` / `frame.binStart`, which are the detect frame) with `frameIndex`. After the change the number appears in exactly these calls:

```c
    found = l3_obs_extract(&params, frameIndex, gPostTimestampUs, frame.binStart, obs, count,
                           gBallFloor, targets, L3_OBS_MAX_TARGETS);
    (void)l3_track_follow(&gClubTrack, targets, found, frameIndex, gPostTimestampUs);
    if (l3_ball_track_update(&gBallTrack, targets, found, frameIndex, gPostTimestampUs) &&
    ...
    if (l3_shot_update(&gShot, &in, frameIndex) == L3_SHOT_SOLVE) {
        in.solved = 1U;
        (void)l3_shot_update(&gShot, &in, frameIndex);
    }
```

- [ ] **Step 4: Check nothing else in the function still means the number**

Run: `awk '/^static void l3_considerBallTrack\(uint32_t slot\)$/{p=1} p&&/[^.&]frame[^.I]/{print NR": "$0} /^}$/{if(p){exit}}' firmware/iwr6843/l3_dump.c`
Expected: only the `l3_detect_frame_t frame = ...` declaration and `&frame` uses.

- [ ] **Step 5: Build the firmware with the TI toolchain where it is installed**

Run (on the firmware build machine): `make -C firmware/iwr6843`
Expected: `l3_dump.c` compiles. Where the toolchain is not installed, record in the task report that this step is pending on the build machine.

- [ ] **Step 6: Commit**

```bash
git add firmware/iwr6843/l3_dump.c
git commit -m "iwr: fix the duplicate frame declaration in l3_considerBallTrack" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 1: Tracking evaluation script and the recorded baseline

**Files:**
- Create: `scripts/analysis/evaluate_iwr_tracking.py`
- Create: `tests/test_evaluate_iwr_tracking.py`
- Create: `docs/superpowers/specs/2026-09-28-tracking-baseline.json`

**Interfaces:**
- Consumes: `firmware_replay.replay_dump`, `ReplayConfig`, `ReplayResult`; `dump_viewer.session_context`, `ViewerOptions`, `tee_bin_for`, `bin_width_m`; `dump.parse_header`.
- Produces: `club_verdict(points, split_frame) -> str`, `ball_verdict(launch_mps, ops_mps) -> str`, `ball_present(frames, ops_mps, bin_m) -> bool`, `split_frame(result) -> int | None`, `iter_cases(roots) -> Iterator[Case]`, `evaluate(case, *, lib=None, ball_hypotheses=None) -> Outcome`, `summarize(outcomes) -> dict`, `compare(summary, baseline, *, allow_more_none=0) -> list[str]`, CLI `main(argv) -> int`. Task 8 adds `--ball-hypotheses`.

- [ ] **Step 1: Write the failing tests**

`tests/test_evaluate_iwr_tracking.py`:

```python
"""Tests for scripts/analysis/evaluate_iwr_tracking.py."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from iwr6843_synth import synth_shot_dump

from openflight.iwr6843 import firmware_host as fw

SCRIPT = Path(__file__).parents[1] / "scripts" / "analysis" / "evaluate_iwr_tracking.py"
BIN_M = 6.0 / 128


@pytest.fixture(scope="module")
def ev():
    spec = importlib.util.spec_from_file_location("evaluate_iwr_tracking", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def point(frame, range_bin):
    return SimpleNamespace(frame=frame, range_bin=range_bin)


def frame_of(frame, timestamp_us, bins):
    return SimpleNamespace(
        frame=frame,
        timestamp_us=timestamp_us,
        targets=[SimpleNamespace(range_bin=b) for b in bins],
    )


def test_club_verdict_reads_the_last_points_before_the_split(ev):
    assert ev.club_verdict([point(f, 30.0 + f) for f in range(6)], 6) == "club"
    assert ev.club_verdict([point(f, 43.5 + 0.1 * f) for f in range(6)], 6) == "stuck"
    assert ev.club_verdict([point(0, 30.0), point(1, 31.0)], 6) == "few"
    after = [point(f, 30.0 + f) for f in range(4)] + [point(f, 50.0) for f in range(4, 9)]
    assert ev.club_verdict(after, 4) == "club"  # points at or after the split are ignored
    assert ev.club_verdict([point(f, 30.0 + f) for f in range(6)], None) == "club"


def test_ball_verdict_is_fifteen_percent_either_side(ev):
    assert ev.ball_verdict(None, 40.0) == "none"
    assert ev.ball_verdict(46.0, 40.0) == "ok"
    assert ev.ball_verdict(34.0, 40.0) == "ok"
    assert ev.ball_verdict(46.1, 40.0) == "wrong"
    assert ev.ball_verdict(13.2, 33.9) == "wrong"


def test_ball_present_needs_three_consecutive_points_at_the_ops_speed(ev):
    step = 40.0 * 0.002 / BIN_M
    frames = [frame_of(k, 2000 * k, [44.0, 50.0 + step * k]) for k in range(1, 4)]
    assert ev.ball_present(frames, 40.0, BIN_M) is True
    assert ev.ball_present(frames[:2], 40.0, BIN_M) is False
    assert ev.ball_present(frames, 20.0, BIN_M) is False  # twice the speed OPS saw
    gap = [frames[0], frames[1], frame_of(4, 8000, [50.0 + step * 4])]
    assert ev.ball_present(gap, 40.0, BIN_M) is False  # frame 3 missing breaks the chain
    assert ev.ball_present([], 40.0, BIN_M) is False


def test_split_frame_prefers_the_recorded_freeze(ev):
    with_freeze = SimpleNamespace(config=SimpleNamespace(post_from_frame=14), fired_frame=11)
    gate_only = SimpleNamespace(config=SimpleNamespace(post_from_frame=None), fired_frame=11)
    no_fire = SimpleNamespace(config=SimpleNamespace(post_from_frame=None), fired_frame=None)
    assert ev.split_frame(with_freeze) == 14
    assert ev.split_frame(gate_only) == 12
    assert ev.split_frame(no_fire) is None


def _outcome(ev, club, ball, present):
    return ev.Outcome("x.l3dump", club, ball, present, None, 40.0)


def test_summarize_counts_every_category(ev):
    outcomes = [
        _outcome(ev, "club", "ok", True),
        _outcome(ev, "stuck", "none", False),
        _outcome(ev, "few", "wrong", True),
    ]
    assert ev.summarize(outcomes) == {
        "captures": 3,
        "club": {"club": 1, "stuck": 1, "few": 1},
        "ball": {"ok": 1, "wrong": 1, "none": 1},
        "ball_present": 2,
    }


def test_compare_names_each_regression(ev):
    base = {
        "captures": 93,
        "club": {"club": 55, "stuck": 32, "few": 6},
        "ball": {"ok": 15, "wrong": 70, "none": 8},
        "ball_present": 23,
    }
    same = json.loads(json.dumps(base))
    assert ev.compare(same, base) == []
    worse = json.loads(json.dumps(base))
    worse["club"]["club"] = 54
    worse["ball"]["ok"] = 14
    worse["ball"]["none"] = 10
    problems = ev.compare(worse, base)
    assert len(problems) == 3
    assert ev.compare(worse, base, allow_more_none=2) == problems[:2]
    other = json.loads(json.dumps(base))
    other["captures"] = 92
    assert ev.compare(other, base) == ["captures: 92 vs baseline 93"]


@pytest.mark.skipif(fw.host_compiler() is None, reason="no C compiler for the firmware modules")
def test_a_synthetic_shot_with_its_session_log_is_scored_end_to_end(ev, tmp_path):
    dumps = tmp_path / "iwr6843"
    dumps.mkdir()
    dump = dumps / "iwr6843_20990101_000000_000_001.l3dump"
    dump.write_bytes(
        synth_shot_dump(ball_speed_ms=60.0, vla_deg=12.0, hla_deg=0.0, tee_range_m=1.372)
    )
    rows = [
        {
            "type": "session_start",
            "trigger_type": "sound",
            "config": {"iwr6843": {"self_trigger": "triggerCfg 29 6.0 2"}},
        },
        {
            "type": "iwr6843_capture",
            "shot_number": 1,
            "capture_path": f"/home/pi/{dump.name}",
            "ball_speed_mph": 60.0 / 0.44704,
        },
    ]
    (tmp_path / "session_x.jsonl").write_text(
        "\n".join(json.dumps(r) for r in rows) + "\n", encoding="utf-8"
    )
    cases = list(ev.iter_cases([tmp_path]))
    assert len(cases) == 1 and cases[0].config.tee_bin == 29
    outcome = ev.evaluate(cases[0])
    assert (outcome.club, outcome.ball, outcome.ball_present) == ("club", "ok", True)
    assert ev.main([str(tmp_path), "--json", str(tmp_path / "out.json")]) == 0
    written = json.loads((tmp_path / "out.json").read_text(encoding="utf-8"))
    assert written["summary"]["captures"] == 1
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `uv run pytest tests/test_evaluate_iwr_tracking.py -v`
Expected: FAIL — `FileNotFoundError` for the script.

- [ ] **Step 3: Write the script**

`scripts/analysis/evaluate_iwr_tracking.py`:

```python
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
"""

from __future__ import annotations

import argparse
import json
from collections.abc import Iterable, Iterator, Sequence
from dataclasses import asdict, dataclass, replace
from pathlib import Path

import numpy as np

from openflight.iwr6843 import firmware_replay as fr
from openflight.iwr6843.dump import parse_header
from openflight.iwr6843.dump_viewer import (
    ViewerOptions,
    bin_width_m,
    session_context,
    tee_bin_for,
)

MPH_TO_MPS = 0.44704
BALL_TOLERANCE = 0.15
CLUB_MIN_STEP_BINS = 0.5
CLUB_POINTS = 5
CHAIN_RATE_BOUNDS = (0.7, 1.1)
CHAIN_MIN_POINTS = 3


@dataclass(frozen=True)
class Case:
    """One capture to replay: where it is, how the board ran it, and the truth."""

    path: Path
    config: fr.ReplayConfig
    ops_mps: float


@dataclass(frozen=True)
class Outcome:
    """How the firmware did on one capture."""

    name: str
    club: str  # "club", "stuck" or "few"
    ball: str  # "ok", "wrong" or "none"
    ball_present: bool
    launch_mps: float | None
    ops_mps: float


def club_verdict(points: Sequence, split_frame: int | None) -> str:
    """"club" when the club track's last points before the split close on the ball."""
    before = [p for p in points if split_frame is None or p.frame < split_frame][-CLUB_POINTS:]
    if len(before) < 3:
        return "few"
    slope = float(np.polyfit([p.frame for p in before], [p.range_bin for p in before], 1)[0])
    return "club" if slope >= CLUB_MIN_STEP_BINS else "stuck"


def ball_verdict(launch_mps: float | None, ops_mps: float, tolerance: float = BALL_TOLERANCE) -> str:
    """The launch speed against OPS: "ok", "wrong", or "none" without a launch."""
    if launch_mps is None:
        return "none"
    return "ok" if abs(launch_mps - ops_mps) <= tolerance * ops_mps else "wrong"


def ball_present(frames: Sequence, ops_mps: float, bin_m: float) -> bool:
    """A chain of targets across consecutive frames moving at the OPS speed."""
    lo, hi = (bound * ops_mps for bound in CHAIN_RATE_BOUNDS)
    previous: list[tuple[float, int]] = []  # (range bin, chain length) in the last frame
    last_frame = last_us = None
    for frame in frames:
        current: list[tuple[float, int]] = []
        consecutive = last_frame is not None and frame.frame == last_frame + 1
        dt_s = (frame.timestamp_us - last_us) * 1e-6 if consecutive else 0.0
        for target in frame.targets:
            length = 1
            if consecutive and dt_s > 0.0:
                for range_bin, chain in previous:
                    rate = (target.range_bin - range_bin) * bin_m / dt_s
                    if lo <= rate <= hi:
                        length = max(length, chain + 1)
            if length >= CHAIN_MIN_POINTS:
                return True
            current.append((target.range_bin, length))
        previous, last_frame, last_us = current, frame.frame, frame.timestamp_us
    return False


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
            if "tee_bin" not in defaults:
                continue  # no triggerCfg: the gate the board ran is unknown
            try:
                meta = parse_header(path.read_bytes())
            except ValueError:
                continue
            options = ViewerOptions.from_mapping(defaults)
            config = fr.ReplayConfig(
                tee_bin=tee_bin_for(options),
                snr=options.snr,
                track_frames=options.track_frames,
                pitch_deg=options.pitch_deg,
                post_from_frame=(meta.get("retention") or {}).get("pre_frames"),
            )
            yield Case(path, config, float(context["ball_speed_mph"]) * MPH_TO_MPS)


def evaluate(case: Case, *, lib=None, ball_hypotheses: bool | None = None) -> Outcome:
    """Replay one capture and judge its club and ball tracks."""
    config = case.config
    if ball_hypotheses is not None:
        config = replace(config, ball_hypotheses=ball_hypotheses)
    result = fr.replay_dump(case.path.read_bytes(), config, lib=lib)
    split = split_frame(result)
    post = [f for f in result.frames if split is not None and f.frame >= split]
    launch = None if result.launch is None else float(result.launch.speed_mps)
    return Outcome(
        name=case.path.name,
        club=club_verdict(result.points, split),
        ball=ball_verdict(launch, case.ops_mps),
        ball_present=ball_present(post, case.ops_mps, bin_width_m()),
        launch_mps=launch,
        ops_mps=case.ops_mps,
    )


def summarize(outcomes: Iterable[Outcome]) -> dict:
    """Counts per category, the numbers the spec's acceptance is written in."""
    outcomes = list(outcomes)

    def count(field: str, values: tuple[str, ...]) -> dict[str, int]:
        return {v: sum(1 for o in outcomes if getattr(o, field) == v) for v in values}

    return {
        "captures": len(outcomes),
        "club": count("club", ("club", "stuck", "few")),
        "ball": count("ball", ("ok", "wrong", "none")),
        "ball_present": sum(1 for o in outcomes if o.ball_present),
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


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    parser.add_argument("roots", nargs="+", type=Path, help="Folders searched for .l3dump files")
    parser.add_argument("--json", type=Path, help="Write the summary and every outcome here")
    parser.add_argument("--compare", type=Path, help="A baseline written by --json")
    parser.add_argument("--allow-more-none", type=int, default=0)
    args = parser.parse_args(argv)
    outcomes = [evaluate(case) for case in iter_cases(args.roots)]
    summary = summarize(outcomes)
    print(json.dumps(summary, indent=2))
    if args.json is not None:
        args.json.write_text(
            json.dumps(
                {"summary": summary, "outcomes": [asdict(o) for o in outcomes]}, indent=2
            )
            + "\n",
            encoding="utf-8",
        )
    if args.compare is not None:
        baseline = json.loads(args.compare.read_text(encoding="utf-8"))["summary"]
        problems = compare(summary, baseline, allow_more_none=args.allow_more_none)
        for problem in problems:
            print(f"REGRESSION {problem}")
        return 1 if problems else 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

`ReplayConfig.ball_hypotheses` does not exist until Task 6; `evaluate` only uses it when the argument is not `None`, so this task's tests never touch it.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `uv run pytest tests/test_evaluate_iwr_tracking.py -v`
Expected: PASS (8 tests).

- [ ] **Step 5: Record the baseline**

Run: `uv run python scripts/analysis/evaluate_iwr_tracking.py iwr-test-sessions --json docs/superpowers/specs/2026-09-28-tracking-baseline.json`
Expected summary (measured 2026-09-28 with the same rules):
`captures 93; club {club 55, stuck 32, few 6}; ball {ok 15, wrong 70, none 8}; ball_present 23`.
If any number differs, stop and report the difference before continuing: the spec's acceptance (R8) is written in these numbers.

- [ ] **Step 6: Lint and commit**

```bash
uv run ruff check scripts/analysis/evaluate_iwr_tracking.py tests/test_evaluate_iwr_tracking.py
uv run ruff format scripts/analysis/evaluate_iwr_tracking.py tests/test_evaluate_iwr_tracking.py
git add scripts/analysis/evaluate_iwr_tracking.py tests/test_evaluate_iwr_tracking.py docs/superpowers/specs/2026-09-28-tracking-baseline.json
git commit -m "iwr: tracking evaluation against OPS, with the 2026-09-28 baseline" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: One Doppler-wrap helper, and the follow cap in bins per second

**Files:**
- Modify: `firmware/iwr6843/l3_club_track.h`, `firmware/iwr6843/l3_club_track.c`
- Modify: `src/openflight/iwr6843/firmware_host.py` (`ClubTrack` mirror, `_SIGNATURES`)
- Test: `tests/test_iwr6843_firmware_club_track.py`

**Interfaces:**
- Consumes: `l3_track_fit(const l3_club_track_t *, uint32_t maxPoints, float *slopeBinsPerS, float *residualBins) -> uint32_t` (existing).
- Produces: `float l3_track_wrapped_diff(float a, float b, float span)` (public; Task 4 uses it); `l3_club_track_t.followBinsPerS` (replaces `followBinsPerFrame`); `#define L3_TRACK_FOLLOW_FIT_POINTS 4U`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_iwr6843_firmware_club_track.py`:

```python
def test_wrapped_diff_goes_the_short_way_round(lib):
    f = lib.l3_track_wrapped_diff
    assert f(1.0, -1.0, 18.0) == pytest.approx(2.0)
    assert f(8.5, -8.5, 18.0) == pytest.approx(1.0)
    assert f(33.6, -2.4, 18.0) == pytest.approx(0.0, abs=1e-4)  # 33.6 m/s reads as -2.4
    assert f(3.0, 1.0, 0.0) == 0.0


def _approach_at(lib, samples):
    """A club track from (timestamp_us, bin) samples, one per frame."""
    tr = Tracker(lib)
    for frame, (ts, bin_) in enumerate(samples, start=1):
        t = target(frame, bin_)
        t.timestampUs = ts
        arr = (Target * 1)(t)
        lib.l3_track_update(ctypes.byref(tr.track), arr, 1, frame, ts)
    return tr


def _follow_at(tr, frame, timestamp_us, bins):
    targets = []
    for b in bins:
        t = target(frame, b)
        t.timestampUs = timestamp_us
        targets.append(t)
    arr = (Target * max(1, len(targets)))(*targets)
    return bool(
        tr.lib.l3_track_follow(ctypes.byref(tr.track), arr, len(targets), frame, timestamp_us)
    )


def test_follow_caps_the_club_by_elapsed_time_not_frames(lib):
    """750 bins/s at impact: a frame retained 4 ms after the last lets the club be
    3 bins on (+0.5 of jitter), whatever the frame count says."""
    history = [(0, 30.0), (2000, 31.5), (4000, 33.0), (6000, 34.5)]
    tr = _approach_at(lib, history)
    assert _follow_at(tr, 5, 10000, [37.3]) is True
    tr = _approach_at(lib, history)
    assert _follow_at(tr, 5, 10000, [38.2]) is False
    assert tr.track.followBinsPerS == pytest.approx(750.0, rel=1e-3)
```

- [ ] **Step 2: Run them to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_club_track.py -k "wrapped_diff or elapsed_time" -v`
Expected: FAIL — `function 'l3_track_wrapped_diff' not found`, and 37.3 refused (the per-frame cap allows 34.5 + 1.5 + 0.5).

- [ ] **Step 3: Implement**

In `l3_club_track.c`, make the wrap helper public (rename every call site from `l3_track_wrappedDiff` to `l3_track_wrapped_diff`):

```c
float l3_track_wrapped_diff(float a, float b, float span)
{
    float diff = a - b;

    if (span <= 0.0F) {
        return 0.0F;
    }
    while (diff > 0.5F * span) {
        diff -= span;
    }
    while (diff < -0.5F * span) {
        diff += span;
    }
    return (diff < 0.0F) ? -diff : diff;
}
```

In `l3_club_track.h`, before `l3_track_why_name`:

```c
/* |a - b| the smaller way round a circle of the given span: the difference of
 * two aliased Doppler readings, or of a speed and an aliased reading (they
 * agree when the speed wraps onto the reading). 0 for a span <= 0. */
float l3_track_wrapped_diff(float a, float b, float span);
```

and next to `L3_TRACK_FOLLOW_LEAD_BINS`:

```c
/* The club's speed at impact, for l3_track_follow's cap, is fitted over this
 * many of its newest points. */
#define L3_TRACK_FOLLOW_FIT_POINTS 4U
```

Replace the state field `float followBinsPerFrame;` with:

```c
    float    followBinsPerS;      /* the club's fitted speed at impact: the follow's cap */
```

In `l3_track_reset`, `track->followBinsPerFrame = 0.0F;` becomes `track->followBinsPerS = 0.0F;`. In `l3_track_follow`:

```c
    if (!track->following) {
        /* The first frame after impact: the club can only slow from here. */
        float slope = 0.0F;
        float residual = 0.0F;

        track->following = 1U;
        track->followBinsPerS =
            (l3_track_fit(track, L3_TRACK_FOLLOW_FIT_POINTS, &slope, &residual) > 0U &&
             slope > 0.0F)
                ? slope
                : 0.0F;
    }
```

In `l3_track_associate`'s `following` branch, the reach becomes time-based:

```c
        if (following) {
            float dtS = (float)(int32_t)(timestampUs - last->timestampUs) * 1.0e-6F;
            float reach = track->lastBin + track->followBinsPerS * ((dtS > 0.0F) ? dtS : 0.0F) +
                          L3_TRACK_FOLLOW_LEAD_BINS;
```

(the rest of the branch is unchanged). Update the comment above `l3_track_associate` and the `l3_track_follow` comment in the header: "where the club would be at its impact speed (fitted in bins per second on the first call)".

In `firmware_host.py`: in `ClubTrack._fields_` rename `("followBinsPerFrame", ctypes.c_float)` to `("followBinsPerS", ctypes.c_float)`; add to `_SIGNATURES`:

```python
    "l3_track_wrapped_diff": ([ctypes.c_float, ctypes.c_float, ctypes.c_float], ctypes.c_float),
```

- [ ] **Step 4: Run the club-track tests**

Run: `uv run pytest tests/test_iwr6843_firmware_club_track.py -v`
Expected: PASS (all, including the existing `test_follow_caps_speed_at_the_impact_speed_not_the_decaying_estimate`, whose frames are evenly 3 ms apart).

- [ ] **Step 5: Run the replay suite (the follow runs in every post-impact frame)**

Run: `uv run pytest tests/test_iwr6843_firmware_replay.py tests/test_iwr6843_dump_viewer.py -v`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add firmware/iwr6843/l3_club_track.h firmware/iwr6843/l3_club_track.c src/openflight/iwr6843/firmware_host.py tests/test_iwr6843_firmware_club_track.py
git commit -m "iwr: time-based follow cap and one public Doppler-wrap helper" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: Ball hypotheses — spawn, joint assignment, coasting

**Files:**
- Create: `tests/iwr6843_twotrack.py`
- Create: `firmware/iwr6843/l3_ball_hyp.h`, `firmware/iwr6843/l3_ball_hyp.c`
- Modify: `firmware/iwr6843/makefile:73` (`SOURCES`)
- Modify: `src/openflight/iwr6843/firmware_host.py` (`HOST_SOURCES`, mirrors, signatures, constants, `__all__`)
- Test: `tests/test_iwr6843_firmware_ball_hyp.py`

**Interfaces:**
- Consumes: `l3_target_obs_t` (`l3_observation.h`), `L3_OBS_MAX_TARGETS`, `L3_OBS_WAVELENGTH_M`.
- Produces (C): `L3_BALL_HYP_MAX`, `L3_BALL_HYP_POINTS`, `L3_BALL_HYP_NONE`; types `l3_ball_hyp_point_t`, `l3_ball_hyp_t`, `l3_ball_hyps_cfg_t`, `l3_ball_hyp_verdict_t`, `l3_ball_hyps_t`; `l3_ball_hyps_cfg_defaults`, `l3_ball_hyps_init`, `l3_ball_hyps_arm`, `l3_ball_hyps_update`, `l3_ball_hyp_fit`, `l3_ball_hyps_set_angles`, `l3_ball_hyps_struct_bytes`. Task 4 adds `l3_ball_hyps_classify`.
- Produces (Python): `fw.BallHypPoint`, `fw.BallHyp`, `fw.BallHypsCfg`, `fw.BallHypVerdict`, `fw.BallHyps`, `fw.BALL_HYP_MAX`, `fw.BALL_HYP_POINTS`, `fw.BALL_HYP_NONE`; `tests/iwr6843_twotrack.py`: `BIN_M`, `SPAN_MPS`, `NO_CLAIM`, `alias()`, `obs()`, `Frame`, `TwoTracks`.

- [ ] **Step 1: Write the scene helper**

`tests/iwr6843_twotrack.py`:

```python
"""Target lists for the two tracks after impact: the club carrying on and the ball.

The ball-hypothesis and ball-track unit tests feed these straight to the C
(no radar cube): per post-impact frame, the extracted targets as
``l3_obs_extract`` lists them (strongest first) and the index of the one the
club track claimed. Ranges are global bins; Doppler is aliased onto
[-span/2, span/2) as the observation layer reads it (positive = receding).
Before impact the ball is stationary and burst MTI cancels it, so it is not
listed; the club approaches the origin.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from openflight.iwr6843 import firmware_host as fw

BIN_M = 6.0 / 128
SPAN_MPS = 2 * fw.OBS_WAVELENGTH_M / (4 * 135e-6)
NO_CLAIM = fw.TRACK_NO_TARGET


def alias(speed_mps: float, span: float = SPAN_MPS) -> float:
    """A radial speed as the lag-1 Doppler reads it."""
    return ((speed_mps + span / 2) % span) - span / 2


def obs(
    frame: int, timestamp_us: int, range_bin: float, stat: float, speed_mps: float
) -> fw.TargetObs:
    t = fw.TargetObs()
    t.frame = frame
    t.timestampUs = timestamp_us
    t.peakBin = int(round(range_bin))
    t.rangeBin = range_bin
    t.energy = 4.0 * stat
    t.peak = stat
    t.stat = stat
    t.snr = stat / 100.0
    t.coherence = 0.9
    t.dopplerAliasMps = alias(speed_mps)
    t.confidence = 0.9
    return t


@dataclass
class Frame:
    frame: int
    timestamp_us: int
    targets: list
    club_index: int
    ball_bin: float | None  # the ball's own listed return; None when not listed


@dataclass
class TwoTracks:
    """A club and a ball leaving the origin; every field is one knob of the scene."""

    origin_bin: float = 46.0
    gate_us: int = 0  # when the tracker is armed (the gate fired)
    impact_offset_us: int = 0  # the true impact is gate_us + impact_offset_us
    club_mps: float = 28.0
    club_decel_mps2: float = 1500.0
    ball_mps: float = 42.0
    club_stat: float = 9000.0
    ball_stat: float = 1500.0
    frame_us: int = 2000
    frames: int = 8
    timestamps_us: list | None = None  # explicit post-frame times, for uneven spacing
    missing_ball: tuple = ()  # 1-based post frames without a ball return
    merged: tuple = ()  # post frames where club and ball are one, club-claimed return
    club_visible: bool = True
    extras: list = field(default_factory=list)  # (range bin, stat, speed m/s) returns

    def _club(self, s: float) -> tuple[float, float]:
        if s < 0.0:
            return self.origin_bin + self.club_mps * s / BIN_M, self.club_mps
        stop_s = self.club_mps / self.club_decel_mps2 if self.club_decel_mps2 > 0 else 1e9
        t = min(s, stop_s)
        travelled = self.club_mps * t - 0.5 * self.club_decel_mps2 * t * t
        return self.origin_bin + travelled / BIN_M, max(self.club_mps - self.club_decel_mps2 * t, 0.0)

    def build(self) -> list[Frame]:
        out = []
        for k in range(1, self.frames + 1):
            ts = self.timestamps_us[k - 1] if self.timestamps_us else self.gate_us + k * self.frame_us
            s = (ts - self.gate_us - self.impact_offset_us) * 1e-6
            entries = []  # (bin, stat, speed, kind)
            club_bin, club_speed = self._club(s)
            ball_listed = s > 0.0 and k not in self.missing_ball
            ball_bin = self.origin_bin + self.ball_mps * s / BIN_M if s > 0.0 else None
            if self.club_visible:
                if k in self.merged and ball_listed:
                    entries.append((0.5 * (club_bin + ball_bin), self.club_stat, club_speed, "club"))
                    ball_listed = False
                else:
                    entries.append((club_bin, self.club_stat, club_speed, "club"))
            if ball_listed:
                entries.append((ball_bin, self.ball_stat, self.ball_mps, "ball"))
            for bin_, stat, speed in self.extras:
                entries.append((bin_, stat, speed, "extra"))
            entries.sort(key=lambda e: -e[1])  # strongest first, as l3_obs_extract lists them
            targets = [obs(k, ts, b, st, sp) for b, st, sp, _ in entries]
            kinds = [kind for *_, kind in entries]
            club_index = kinds.index("club") if "club" in kinds else NO_CLAIM
            out.append(Frame(k, ts, targets, club_index, ball_bin if ball_listed else None))
        return out
```

- [ ] **Step 2: Write the failing tests**

`tests/test_iwr6843_firmware_ball_hyp.py`:

```python
"""Tests for the IWR6843 ball hypotheses, firmware/iwr6843/l3_ball_hyp.c.

After impact the tracker keeps up to four candidate ball trajectories that
start near the origin, assigns each frame's targets to them jointly with the
club track's claim (never the claimed target), and decides later which one is
the ball. Scenes come from tests/iwr6843_twotrack.py.
"""

from __future__ import annotations

import ctypes

import pytest
from iwr6843_twotrack import BIN_M, NO_CLAIM, TwoTracks, obs

from openflight.iwr6843 import firmware_host as fw


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def make_hyps(lib, **overrides):
    cfg = fw.BallHypsCfg()
    lib.l3_ball_hyps_cfg_defaults(ctypes.byref(cfg))
    for name, value in overrides.items():
        setattr(cfg, name, value)
    hyps = fw.BallHyps()
    lib.l3_ball_hyps_init(ctypes.byref(hyps), ctypes.byref(cfg))
    return hyps


def arm(lib, hyps, origin_bin=46.0, gate_us=0):
    lib.l3_ball_hyps_arm(ctypes.byref(hyps), origin_bin, gate_us)


def feed(lib, hyps, frame, timestamp_us, targets, club_index=NO_CLAIM):
    arr = (fw.TargetObs * max(1, len(targets)))(*targets)
    return lib.l3_ball_hyps_update(
        ctypes.byref(hyps), arr, len(targets), frame, timestamp_us, club_index
    )


def run(lib, scene, **overrides):
    hyps = make_hyps(lib, **overrides)
    arm(lib, hyps, scene.origin_bin, scene.gate_us)
    frames = scene.build()
    for f in frames:
        feed(lib, hyps, f.frame, f.timestamp_us, f.targets, f.club_index)
    return hyps, frames


def active(hyps):
    return [hyps.hyp[i] for i in range(fw.BALL_HYP_MAX) if hyps.hyp[i].active]


def bins(hyp):
    return [round(hyp.points[i].rangeBin, 3) for i in range(hyp.count)]


def truth(frames):
    return [round(f.ball_bin, 3) for f in frames if f.ball_bin is not None]


def test_defaults(lib):
    cfg = fw.BallHypsCfg()
    lib.l3_ball_hyps_cfg_defaults(ctypes.byref(cfg))
    assert (cfg.spawnBehindBins, cfg.spawnBeyondBins, cfg.gateBins, cfg.gateMps) == (
        1.0,
        10.0,
        1.5,
        8.0,
    )
    assert (cfg.maxMisses, cfg.classifyPoints, cfg.impactToleranceUs) == (2, 4, 15000)
    assert (cfg.minDepartureMps, cfg.maxSpeedMps) == (10.0, 100.0)
    assert (cfg.maxResidualBins, cfg.dopplerToleranceMps) == (1.0, 2.5)
    assert cfg.binWidthM == pytest.approx(BIN_M)


def test_the_struct_layout_matches_the_c(lib):
    assert ctypes.sizeof(fw.BallHyps) == lib.l3_ball_hyps_struct_bytes()


def test_unarmed_hypotheses_ignore_targets(lib):
    hyps = make_hyps(lib)
    assert feed(lib, hyps, 1, 2000, [obs(1, 2000, 47.0, 1000.0, 40.0)]) == 0
    assert not active(hyps)


def test_the_ball_is_one_hypothesis_and_the_club_claim_none(lib):
    hyps, frames = run(lib, TwoTracks())
    got = active(hyps)
    assert len(got) == 1
    assert bins(got[0]) == truth(frames)
    assert hyps.spawned == 1


def test_only_targets_near_the_origin_start_a_hypothesis(lib):
    hyps = make_hyps(lib)
    arm(lib, hyps)
    targets = [obs(1, 2000, b, 900.0, 5.0) for b in (44.5, 45.2, 55.8, 56.4)]
    feed(lib, hyps, 1, 2000, targets)
    assert sorted(h.points[0].rangeBin for h in active(hyps)) == pytest.approx([45.2, 55.8])


def test_a_merged_first_return_is_a_missed_frame_not_a_point(lib):
    hyps, frames = run(lib, TwoTracks(merged=(1,)))
    (hyp,) = active(hyps)
    assert hyp.points[0].frame == 2
    assert bins(hyp) == truth(frames)


def test_the_ball_coasts_over_two_missing_frames_and_is_picked_up(lib):
    hyps, frames = run(lib, TwoTracks(missing_ball=(3, 4)))
    (hyp,) = active(hyps)
    assert [hyp.points[i].frame for i in range(hyp.count)] == [1, 2, 5, 6, 7, 8]
    assert bins(hyp) == truth(frames)


def test_three_missing_frames_drop_it(lib):
    hyps, _ = run(lib, TwoTracks(missing_ball=(3, 4, 5)))
    assert hyps.dropped == 1
    assert not active(hyps)  # by frame 6 the ball is past the start band


def test_a_ball_return_the_club_claims_is_never_a_ball_point(lib):
    """Filling a gap with the club's return is the failure this module exists to stop."""
    hyps = make_hyps(lib)
    arm(lib, hyps)
    feed(lib, hyps, 1, 2000, [obs(1, 2000, 47.8, 1500.0, 42.0)])
    feed(lib, hyps, 2, 4000, [obs(2, 4000, 49.6, 1500.0, 42.0)])
    feed(lib, hyps, 3, 6000, [obs(3, 6000, 51.4, 9000.0, 42.0)], club_index=0)
    (hyp,) = active(hyps)
    assert (hyp.count, hyp.misses) == (2, 1)


def test_a_full_set_evicts_only_a_single_point_hypothesis(lib):
    hyps = make_hyps(lib)
    arm(lib, hyps)
    first = [obs(1, 2000, b, 1000.0, 30.0) for b in (46.2, 50.0, 53.0, 55.8, 47.5)]
    feed(lib, hyps, 1, 2000, first)  # four slots: the fifth target starts nothing
    assert sorted(round(h.points[0].rangeBin, 1) for h in active(hyps)) == [46.2, 50.0, 53.0, 55.8]
    # 0.5 ms later: 46.2 and 50.0 continue; a new return at 45.3 evicts the
    # oldest single-point hypothesis that missed (53.0), never a two-point one.
    second = [obs(2, 2500, b, 1000.0, 30.0) for b in (46.9, 50.7, 45.3)]
    feed(lib, hyps, 2, 2500, second)
    got = {round(h.points[0].rangeBin, 1): h.count for h in active(hyps)}
    assert got == {46.2: 2, 50.0: 2, 55.8: 1, 45.3: 1}
    assert hyps.dropped == 1


def test_uneven_frame_spacing_is_followed_by_time(lib):
    hyps, frames = run(lib, TwoTracks(timestamps_us=[1000, 2500, 6500, 8000, 12000], frames=5))
    (hyp,) = active(hyps)
    assert bins(hyp) == truth(frames)


def test_set_angles_marks_the_newest_point_this_frame_only(lib):
    hyps = make_hyps(lib)
    arm(lib, hyps)
    feed(lib, hyps, 1, 2000, [obs(1, 2000, 47.8, 1500.0, 42.0)])
    index = next(i for i in range(fw.BALL_HYP_MAX) if hyps.hyp[i].active)
    both = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION
    assert lib.l3_ball_hyps_set_angles(ctypes.byref(hyps), index, 0.1, 0.2, both) == 1
    point = hyps.hyp[index].points[0]
    assert (point.azimuthRad, point.elevationRad) == pytest.approx((0.1, 0.2))
    assert point.anglesValid == both
    feed(lib, hyps, 2, 4000, [])  # nothing appended this frame
    assert lib.l3_ball_hyps_set_angles(ctypes.byref(hyps), index, 0.3, 0.3, both) == 0
    assert lib.l3_ball_hyps_set_angles(ctypes.byref(hyps), fw.BALL_HYP_MAX, 0.3, 0.3, both) == 0


def test_the_fit_reads_the_rate_and_the_range_at_a_reference_time(lib):
    hyps, _ = run(lib, TwoTracks(frames=4))
    (hyp,) = active(hyps)
    rate, at, residual = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
    assert lib.l3_ball_hyp_fit(
        ctypes.byref(hyp), 0, ctypes.byref(rate), ctypes.byref(at), ctypes.byref(residual)
    )
    assert rate.value * BIN_M == pytest.approx(42.0, rel=1e-3)
    assert at.value == pytest.approx(46.0, abs=0.01)  # the origin at the gate time
    assert residual.value == pytest.approx(0.0, abs=1e-3)
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py -v`
Expected: FAIL — `AttributeError: module ... has no attribute 'BallHypsCfg'`.

- [ ] **Step 4: Write the header**

`firmware/iwr6843/l3_ball_hyp.h`:

```c
/* IWR6843 ball hypotheses: the ball found after impact, not assumed.
 *
 * After the gate fires, the club carries on through impact and is usually
 * the strongest, most confident departing return. Taking the most confident
 * target in the departure band therefore follows the club. Instead, up to
 * L3_BALL_HYP_MAX candidate trajectories are kept that start near the origin;
 * each frame's targets are assigned to them jointly with the club track, whose
 * claimed target never becomes a ball point (a merged return is a missed
 * frame); and only once a hypothesis holds enough points is it judged
 * (l3_ball_hyps_classify): origin crossing near the gate time, a physical
 * range rate, a straight fit, Doppler agreeing with the rate, and a return
 * weaker than the club's. Prediction, gates and fits run on timestamps, not
 * frame counts. Pure C, fixed size, no hardware.
 */
#ifndef L3_BALL_HYP_H
#define L3_BALL_HYP_H

#include <stdint.h>

#include "l3_observation.h"

#define L3_BALL_HYP_MAX    4U
#define L3_BALL_HYP_POINTS 8U
#define L3_BALL_HYP_NONE   0xFFFFFFFFU

typedef struct {
    uint32_t frame;
    uint32_t timestampUs;
    float    rangeBin;            /* global, sub-bin */
    float    dopplerAliasMps;
    float    stat;
    float    clubStat;            /* the club's claimed return that frame, 0 without one */
    float    azimuthRad;
    float    elevationRad;
    uint8_t  anglesValid;         /* L3_OBS_ANGLE_* bits */
} l3_ball_hyp_point_t;

typedef struct {
    uint8_t  active;
    uint8_t  count;
    uint8_t  misses;              /* consecutive frames without a point */
    uint32_t id;                  /* spawn order: the tie-break */
    uint32_t lastTargetIndex;     /* this frame's target, L3_BALL_HYP_NONE when none */
    l3_ball_hyp_point_t points[L3_BALL_HYP_POINTS];  /* oldest first; the oldest slides out */
} l3_ball_hyp_t;

typedef struct {
    float    binWidthM;           /* l3_ball_track_init copies these two from its core */
    float    velocitySpanMps;
    float    spawnBehindBins;     /* a hypothesis starts from origin - this ... */
    float    spawnBeyondBins;     /* ... to origin + this */
    float    gateBins;            /* association half-width at zero elapsed time ... */
    float    gateMps;             /* ... growing by this speed uncertainty over the gap */
    uint32_t maxMisses;           /* coasted frames before a hypothesis is dropped */
    uint32_t classifyPoints;      /* points before a hypothesis may be the ball */
    float    minDepartureMps;
    float    maxSpeedMps;
    uint32_t impactToleranceUs;   /* the fit must reach the origin this close to the gate time */
    float    maxResidualBins;     /* RMS about the fitted line */
    float    dopplerToleranceMps; /* a point agrees when its Doppler is this close to the rate */
} l3_ball_hyps_cfg_t;

typedef struct {
    int32_t  index;               /* hypothesis index, -1 when none qualifies */
    uint32_t points;
    float    rateMps;             /* fitted range rate */
    float    originOffsetUs;      /* origin crossing minus the gate time */
    float    residualBins;
    float    dopplerAgreement;    /* 0..1 */
    float    weakerFraction;      /* 0..1 of the frames with a club return; 0.5 without */
    float    score;
} l3_ball_hyp_verdict_t;

typedef struct {
    l3_ball_hyps_cfg_t cfg;
    uint8_t  armed;
    float    originBin;
    uint32_t impactTimestampUs;   /* the gate time the tracker was armed at */
    uint32_t nextId;
    uint32_t spawned;
    uint32_t dropped;             /* coasted out or evicted */
    l3_ball_hyp_t hyp[L3_BALL_HYP_MAX];
} l3_ball_hyps_t;

void l3_ball_hyps_cfg_defaults(l3_ball_hyps_cfg_t *cfg);
void l3_ball_hyps_init(l3_ball_hyps_t *hyps, const l3_ball_hyps_cfg_t *cfg);
/* Forget every hypothesis and start looking from originBin at the gate time. */
void l3_ball_hyps_arm(l3_ball_hyps_t *hyps, float originBin, uint32_t impactTimestampUs);
/* One post-impact frame's targets (strongest first) and the index of the one
 * the club track claimed (L3_TRACK_NO_TARGET, or anything >= n, for none).
 * Returns the hypotheses active afterwards. */
uint32_t l3_ball_hyps_update(l3_ball_hyps_t *hyps, const l3_target_obs_t *targets, uint32_t n,
                             uint32_t frame, uint32_t timestampUs, uint32_t clubIndex);
/* Least squares of range against time over the hypothesis's points, time
 * measured from referenceUs: rate in bins per second, the fitted range at the
 * reference time and the RMS residual. Returns 0 with fewer than 2 points or
 * no spread in time. */
int32_t l3_ball_hyp_fit(const l3_ball_hyp_t *hyp, uint32_t referenceUs, float *rateBinsPerS,
                        float *binAtReference, float *residualBins);
/* Angles for the point hypothesis `index` appended this frame. Returns 0 when
 * it appended nothing this frame or the index is out of range. */
int32_t l3_ball_hyps_set_angles(l3_ball_hyps_t *hyps, uint32_t index, float azimuthRad,
                                float elevationRad, uint8_t anglesValid);
/* sizeof(l3_ball_hyps_t), for the ctypes mirror's layout check. */
uint32_t l3_ball_hyps_struct_bytes(void);

#endif /* L3_BALL_HYP_H */
```

- [ ] **Step 5: Write the implementation**

`firmware/iwr6843/l3_ball_hyp.c`:

```c
/* IWR6843 ball hypotheses. See l3_ball_hyp.h. */
#include <math.h>
#include <string.h>

#include "l3_ball_hyp.h"

void l3_ball_hyps_cfg_defaults(l3_ball_hyps_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->binWidthM = 6.0F / 128.0F;
    cfg->velocitySpanMps = 2.0F * L3_OBS_WAVELENGTH_M / (4.0F * 135.0e-6F);
    cfg->spawnBehindBins = 1.0F;      /* the ball starts at the origin ... */
    cfg->spawnBeyondBins = 10.0F;     /* ... and is first seen within ~3 frames of it */
    cfg->gateBins = 1.5F;
    cfg->gateMps = 8.0F;              /* drag and fit error, per second of prediction */
    cfg->maxMisses = 2U;
    cfg->classifyPoints = 4U;
    cfg->minDepartureMps = 10.0F;     /* the slowest chip leaves faster than this */
    cfg->maxSpeedMps = 100.0F;
    cfg->impactToleranceUs = 15000U;  /* the gate is not the exact impact */
    cfg->maxResidualBins = 1.0F;
    cfg->dopplerToleranceMps = 2.5F;
}

static void l3_ball_hyps_clear(l3_ball_hyps_t *hyps)
{
    uint32_t i;

    memset(hyps->hyp, 0, sizeof(hyps->hyp));
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        hyps->hyp[i].lastTargetIndex = L3_BALL_HYP_NONE;
    }
    hyps->nextId = 0U;
    hyps->spawned = 0U;
    hyps->dropped = 0U;
}

void l3_ball_hyps_init(l3_ball_hyps_t *hyps, const l3_ball_hyps_cfg_t *cfg)
{
    memset(hyps, 0, sizeof(*hyps));
    hyps->cfg = *cfg;
    l3_ball_hyps_clear(hyps);
}

void l3_ball_hyps_arm(l3_ball_hyps_t *hyps, float originBin, uint32_t impactTimestampUs)
{
    l3_ball_hyps_clear(hyps);
    hyps->armed = 1U;
    hyps->originBin = originBin;
    hyps->impactTimestampUs = impactTimestampUs;
}

/* Signed seconds from earlier to later on the wrapping microsecond clock. */
static float l3_ball_hyps_seconds(uint32_t later, uint32_t earlier)
{
    return (float)(int32_t)(later - earlier) * 1.0e-6F;
}

int32_t l3_ball_hyp_fit(const l3_ball_hyp_t *hyp, uint32_t referenceUs, float *rateBinsPerS,
                        float *binAtReference, float *residualBins)
{
    float meanT = 0.0F;
    float meanR = 0.0F;
    float sumTT = 0.0F;
    float sumTR = 0.0F;
    float sse = 0.0F;
    float slope;
    float intercept;
    uint32_t i;

    if (hyp->count < 2U) {
        return 0;
    }
    for (i = 0U; i < hyp->count; i++) {
        meanT += l3_ball_hyps_seconds(hyp->points[i].timestampUs, referenceUs);
        meanR += hyp->points[i].rangeBin;
    }
    meanT /= (float)hyp->count;
    meanR /= (float)hyp->count;
    for (i = 0U; i < hyp->count; i++) {
        float dt = l3_ball_hyps_seconds(hyp->points[i].timestampUs, referenceUs) - meanT;

        sumTT += dt * dt;
        sumTR += dt * (hyp->points[i].rangeBin - meanR);
    }
    if (!(sumTT > 0.0F)) {
        return 0;
    }
    slope = sumTR / sumTT;
    intercept = meanR - slope * meanT;
    for (i = 0U; i < hyp->count; i++) {
        float t = l3_ball_hyps_seconds(hyp->points[i].timestampUs, referenceUs);
        float error = hyp->points[i].rangeBin - (intercept + slope * t);

        sse += error * error;
    }
    *rateBinsPerS = slope;
    *binAtReference = intercept;
    *residualBins = sqrtf(sse / (float)hyp->count);
    return 1;
}

/* Where a hypothesis can be at timestampUs: [lo, hi] around a centre. One
 * point: leaving at anything from minDepartureMps to maxSpeedMps. More: its
 * fitted line, with a gate that widens with the time predicted over. */
static void l3_ball_hyps_window(const l3_ball_hyps_cfg_t *cfg, const l3_ball_hyp_t *hyp,
                                uint32_t timestampUs, float *lo, float *hi, float *centre)
{
    const l3_ball_hyp_point_t *last = &hyp->points[hyp->count - 1U];
    float dtS = l3_ball_hyps_seconds(timestampUs, last->timestampUs);
    float binsPerM = (cfg->binWidthM > 0.0F) ? 1.0F / cfg->binWidthM : 0.0F;
    float rate;
    float atLast;
    float residual;

    if (dtS < 0.0F) {
        dtS = 0.0F;
    }
    if (l3_ball_hyp_fit(hyp, last->timestampUs, &rate, &atLast, &residual)) {
        float spread = cfg->gateBins + cfg->gateMps * binsPerM * dtS;

        *centre = atLast + rate * dtS;
        *lo = *centre - spread;
        *hi = *centre + spread;
        return;
    }
    *lo = last->rangeBin + cfg->minDepartureMps * binsPerM * dtS - cfg->gateBins;
    *hi = last->rangeBin + cfg->maxSpeedMps * binsPerM * dtS + cfg->gateBins;
    *centre = 0.5F * (*lo + *hi);
}

static void l3_ball_hyps_append(l3_ball_hyp_t *hyp, const l3_target_obs_t *target,
                                uint32_t index, uint32_t frame, uint32_t timestampUs,
                                float clubStat)
{
    l3_ball_hyp_point_t *point;

    if (hyp->count == L3_BALL_HYP_POINTS) {
        memmove(&hyp->points[0], &hyp->points[1],
                sizeof(hyp->points[0]) * (L3_BALL_HYP_POINTS - 1U));
        hyp->count--;
    }
    point = &hyp->points[hyp->count++];
    memset(point, 0, sizeof(*point));
    point->frame = frame;
    point->timestampUs = timestampUs;
    point->rangeBin = target->rangeBin;
    point->dopplerAliasMps = target->dopplerAliasMps;
    point->stat = target->stat;
    point->clubStat = clubStat;
    hyp->misses = 0U;
    hyp->lastTargetIndex = index;
}

/* A slot for a new hypothesis: a free one, else a single-point hypothesis not
 * fed this frame (the one that missed most, then the oldest); -1 when every
 * hypothesis is established. */
static int32_t l3_ball_hyps_slot(const l3_ball_hyps_t *hyps)
{
    int32_t best = -1;
    uint32_t i;

    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        if (!hyps->hyp[i].active) {
            return (int32_t)i;
        }
    }
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        const l3_ball_hyp_t *hyp = &hyps->hyp[i];

        if (hyp->count != 1U || hyp->lastTargetIndex != L3_BALL_HYP_NONE) {
            continue;
        }
        if (best < 0 || hyp->misses > hyps->hyp[best].misses ||
            (hyp->misses == hyps->hyp[best].misses && hyp->id < hyps->hyp[best].id)) {
            best = (int32_t)i;
        }
    }
    return best;
}

uint32_t l3_ball_hyps_update(l3_ball_hyps_t *hyps, const l3_target_obs_t *targets, uint32_t n,
                             uint32_t frame, uint32_t timestampUs, uint32_t clubIndex)
{
    const l3_ball_hyps_cfg_t *cfg = &hyps->cfg;
    uint8_t taken[L3_OBS_MAX_TARGETS];
    uint8_t fed[L3_BALL_HYP_MAX];
    float lo[L3_BALL_HYP_MAX];
    float hi[L3_BALL_HYP_MAX];
    float centre[L3_BALL_HYP_MAX];
    float clubStat;
    uint32_t i;
    uint32_t j;
    uint32_t active = 0U;

    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        hyps->hyp[i].lastTargetIndex = L3_BALL_HYP_NONE;
    }
    if (!hyps->armed) {
        return 0U;
    }
    if (n > L3_OBS_MAX_TARGETS) {
        n = L3_OBS_MAX_TARGETS;
    }
    memset(taken, 0, sizeof(taken));
    memset(fed, 0, sizeof(fed));
    clubStat = (clubIndex < n) ? targets[clubIndex].stat : 0.0F;
    if (clubIndex < n) {
        taken[clubIndex] = 1U;  /* the club's return is never a ball point */
    }
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        if (hyps->hyp[i].active) {
            l3_ball_hyps_window(cfg, &hyps->hyp[i], timestampUs, &lo[i], &hi[i], &centre[i]);
        }
    }
    /* Joint assignment: the cheapest (hypothesis, target) pair first, each
     * hypothesis and each target used once. Ties go to the earlier pair. */
    for (;;) {
        int32_t bestHyp = -1;
        int32_t bestTarget = -1;
        float bestCost = 0.0F;

        for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
            float half;

            if (!hyps->hyp[i].active || fed[i]) {
                continue;
            }
            half = 0.5F * (hi[i] - lo[i]);
            for (j = 0U; j < n; j++) {
                float cost;

                if (taken[j] || targets[j].rangeBin < lo[i] || targets[j].rangeBin > hi[i]) {
                    continue;
                }
                cost = (half > 0.0F) ? fabsf(targets[j].rangeBin - centre[i]) / half : 0.0F;
                if (bestHyp < 0 || cost < bestCost) {
                    bestHyp = (int32_t)i;
                    bestTarget = (int32_t)j;
                    bestCost = cost;
                }
            }
        }
        if (bestHyp < 0) {
            break;
        }
        l3_ball_hyps_append(&hyps->hyp[bestHyp], &targets[bestTarget], (uint32_t)bestTarget,
                            frame, timestampUs, clubStat);
        fed[bestHyp] = 1U;
        taken[bestTarget] = 1U;
    }
    /* Coast the hypotheses that found nothing; drop them after maxMisses. */
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        l3_ball_hyp_t *hyp = &hyps->hyp[i];

        if (!hyp->active || fed[i]) {
            continue;
        }
        hyp->misses++;
        if (hyp->misses > cfg->maxMisses) {
            hyp->active = 0U;
            hyps->dropped++;
        }
    }
    /* Start hypotheses from what is left in the start band. */
    for (j = 0U; j < n; j++) {
        float range = targets[j].rangeBin;
        l3_ball_hyp_t *hyp;
        int32_t slot;

        if (taken[j] || range < hyps->originBin - cfg->spawnBehindBins ||
            range > hyps->originBin + cfg->spawnBeyondBins) {
            continue;
        }
        slot = l3_ball_hyps_slot(hyps);
        if (slot < 0) {
            break;
        }
        hyp = &hyps->hyp[slot];
        if (hyp->active) {
            hyps->dropped++;  /* evicted */
        }
        memset(hyp, 0, sizeof(*hyp));
        hyp->active = 1U;
        hyp->id = hyps->nextId++;
        l3_ball_hyps_append(hyp, &targets[j], j, frame, timestampUs, clubStat);
        hyps->spawned++;
        taken[j] = 1U;
    }
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        active += hyps->hyp[i].active;
    }
    return active;
}

int32_t l3_ball_hyps_set_angles(l3_ball_hyps_t *hyps, uint32_t index, float azimuthRad,
                                float elevationRad, uint8_t anglesValid)
{
    l3_ball_hyp_t *hyp;
    l3_ball_hyp_point_t *point;

    if (index >= L3_BALL_HYP_MAX) {
        return 0;
    }
    hyp = &hyps->hyp[index];
    if (!hyp->active || hyp->count == 0U || hyp->lastTargetIndex == L3_BALL_HYP_NONE) {
        return 0;
    }
    point = &hyp->points[hyp->count - 1U];
    point->azimuthRad = azimuthRad;
    point->elevationRad = elevationRad;
    point->anglesValid = anglesValid;
    return 1;
}

uint32_t l3_ball_hyps_struct_bytes(void)
{
    return (uint32_t)sizeof(l3_ball_hyps_t);
}
```

- [ ] **Step 6: Add the module to both builds**

`firmware/iwr6843/makefile:73` — add `l3_ball_hyp.c` to `SOURCES` after `l3_club_track.c`:

```make
SOURCES    = l3_dump.c track_select.c detect_queue.c capture_plan.c compact_iq16.c live_selector.c l3_text.c l3_frames.c l3_angle.c l3_observation.c l3_trigger.c l3_ball.c l3_club_track.c l3_ball_hyp.c l3_impact.c l3_shot.c l3_ball_track.c l3_result.c l3_profile.c l3_adaptive.c l3_iq8.c l3_retain.c l3_iq16_stats.c
```

`firmware_host.py` — in `HOST_SOURCES` add `"l3_ball_hyp.c",` after `"l3_club_track.c",`. After `TRACK_WHY_NAMES` add:

```python
# l3_ball_hyp.h
BALL_HYP_MAX = 4
BALL_HYP_POINTS = 8
BALL_HYP_NONE = 0xFFFFFFFF
```

Before `class BallTrackCfg`, add the mirrors:

```python
class BallHypPoint(ctypes.Structure):
    """``l3_ball_hyp_point_t``."""

    _fields_ = [
        ("frame", ctypes.c_uint32),
        ("timestampUs", ctypes.c_uint32),
        ("rangeBin", ctypes.c_float),
        ("dopplerAliasMps", ctypes.c_float),
        ("stat", ctypes.c_float),
        ("clubStat", ctypes.c_float),
        ("azimuthRad", ctypes.c_float),
        ("elevationRad", ctypes.c_float),
        ("anglesValid", ctypes.c_uint8),
    ]


class BallHyp(ctypes.Structure):
    """``l3_ball_hyp_t``: one candidate ball trajectory."""

    _fields_ = [
        ("active", ctypes.c_uint8),
        ("count", ctypes.c_uint8),
        ("misses", ctypes.c_uint8),
        ("id", ctypes.c_uint32),
        ("lastTargetIndex", ctypes.c_uint32),
        ("points", BallHypPoint * BALL_HYP_POINTS),
    ]


class BallHypsCfg(ctypes.Structure):
    """``l3_ball_hyps_cfg_t``."""

    _fields_ = [
        ("binWidthM", ctypes.c_float),
        ("velocitySpanMps", ctypes.c_float),
        ("spawnBehindBins", ctypes.c_float),
        ("spawnBeyondBins", ctypes.c_float),
        ("gateBins", ctypes.c_float),
        ("gateMps", ctypes.c_float),
        ("maxMisses", ctypes.c_uint32),
        ("classifyPoints", ctypes.c_uint32),
        ("minDepartureMps", ctypes.c_float),
        ("maxSpeedMps", ctypes.c_float),
        ("impactToleranceUs", ctypes.c_uint32),
        ("maxResidualBins", ctypes.c_float),
        ("dopplerToleranceMps", ctypes.c_float),
    ]


class BallHypVerdict(ctypes.Structure):
    """``l3_ball_hyp_verdict_t``."""

    _fields_ = [
        ("index", ctypes.c_int32),
        ("points", ctypes.c_uint32),
        ("rateMps", ctypes.c_float),
        ("originOffsetUs", ctypes.c_float),
        ("residualBins", ctypes.c_float),
        ("dopplerAgreement", ctypes.c_float),
        ("weakerFraction", ctypes.c_float),
        ("score", ctypes.c_float),
    ]


class BallHyps(ctypes.Structure):
    """``l3_ball_hyps_t``: the bounded set of candidate ball trajectories."""

    _fields_ = [
        ("cfg", BallHypsCfg),
        ("armed", ctypes.c_uint8),
        ("originBin", ctypes.c_float),
        ("impactTimestampUs", ctypes.c_uint32),
        ("nextId", ctypes.c_uint32),
        ("spawned", ctypes.c_uint32),
        ("dropped", ctypes.c_uint32),
        ("hyp", BallHyp * BALL_HYP_MAX),
    ]
```

Add to `_SIGNATURES`:

```python
    "l3_ball_hyps_cfg_defaults": ([_P(BallHypsCfg)], None),
    "l3_ball_hyps_init": ([_P(BallHyps), _P(BallHypsCfg)], None),
    "l3_ball_hyps_arm": ([_P(BallHyps), ctypes.c_float, _U32], None),
    "l3_ball_hyps_update": ([_P(BallHyps), _P(TargetObs), _U32, _U32, _U32, _U32], _U32),
    "l3_ball_hyp_fit": (
        [_P(BallHyp), _U32, _P(ctypes.c_float), _P(ctypes.c_float), _P(ctypes.c_float)],
        ctypes.c_int32,
    ),
    "l3_ball_hyps_set_angles": (
        [_P(BallHyps), _U32, ctypes.c_float, ctypes.c_float, ctypes.c_uint8],
        ctypes.c_int32,
    ),
    "l3_ball_hyps_struct_bytes": ([], _U32),
```

Add `"BALL_HYP_MAX"`, `"BALL_HYP_NONE"`, `"BALL_HYP_POINTS"`, `"BallHyp"`, `"BallHypPoint"`, `"BallHypVerdict"`, `"BallHyps"`, `"BallHypsCfg"` to `__all__`.

- [ ] **Step 7: Run the tests to verify they pass**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py -v`
Expected: PASS (13 tests). If `test_a_full_set_evicts_only_a_single_point_hypothesis` fails on which hypothesis was evicted, check the tie-break in `l3_ball_hyps_slot` (most misses, then lowest id) before changing the test: after frame 2 both 53.0 and 55.8 have one miss and 53.0 has the lower id.

- [ ] **Step 8: Run every firmware suite (a new source in the shared library)**

Run: `uv run pytest tests/test_iwr6843_firmware_*.py -q`
Expected: PASS apart from the failures recorded before this plan (`test_iwr6843_firmware_iq8.py::test_mode_validation` is not among them once zig is installed; compare with `git stash`-free baseline output if unsure).

- [ ] **Step 9: Commit**

```bash
git add firmware/iwr6843/l3_ball_hyp.h firmware/iwr6843/l3_ball_hyp.c firmware/iwr6843/makefile src/openflight/iwr6843/firmware_host.py tests/iwr6843_twotrack.py tests/test_iwr6843_firmware_ball_hyp.py
git commit -m "iwr: bounded ball hypotheses assigned jointly with the club track" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: Ball hypotheses — delayed classification

**Files:**
- Modify: `firmware/iwr6843/l3_ball_hyp.h`, `firmware/iwr6843/l3_ball_hyp.c`
- Modify: `src/openflight/iwr6843/firmware_host.py` (`_SIGNATURES`)
- Test: `tests/test_iwr6843_firmware_ball_hyp.py`

**Interfaces:**
- Consumes: Task 3's types and `l3_ball_hyp_fit`; Task 2's `l3_track_wrapped_diff`.
- Produces: `void l3_ball_hyps_classify(const l3_ball_hyps_t *hyps, l3_ball_hyp_verdict_t *out)`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_iwr6843_firmware_ball_hyp.py`:

```python
def verdict(lib, hyps):
    out = fw.BallHypVerdict()
    lib.l3_ball_hyps_classify(ctypes.byref(hyps), ctypes.byref(out))
    return out


def test_no_verdict_before_four_points(lib):
    hyps, _ = run(lib, TwoTracks(frames=3))
    assert verdict(lib, hyps).index == -1


def test_the_ball_hypothesis_is_classified_with_its_speed(lib):
    hyps, _ = run(lib, TwoTracks(frames=6))
    v = verdict(lib, hyps)
    assert v.index >= 0
    assert bins(hyps.hyp[v.index])[0] == pytest.approx(46.0 + 42.0 * 0.002 / BIN_M, abs=0.01)
    assert v.points == 6
    assert v.rateMps == pytest.approx(42.0, rel=0.02)
    assert (v.weakerFraction, v.dopplerAgreement) == (1.0, 1.0)
    assert abs(v.originOffsetUs) < 200.0


def test_a_stationary_return_near_the_origin_is_never_the_ball(lib):
    """The strong stall beside the ball (2026-08-24: bins 38.2-38.4, SNR up to 970)."""
    scene = TwoTracks(missing_ball=tuple(range(1, 9)), extras=[(48.0, 20000.0, 0.8)])
    hyps, _ = run(lib, scene)
    assert active(hyps)  # it is followed as a hypothesis ...
    assert verdict(lib, hyps).index == -1  # ... but it never leaves: not the ball


@pytest.mark.parametrize("offset_us", [-6000, 6000])
def test_the_gate_need_not_be_the_exact_impact(lib, offset_us):
    hyps, _ = run(lib, TwoTracks(frames=10, impact_offset_us=offset_us))
    v = verdict(lib, hyps)
    assert v.index >= 0
    assert v.originOffsetUs == pytest.approx(offset_us, abs=300.0)


def test_a_ball_leaving_far_from_the_gate_time_is_not_the_ball(lib):
    hyps, _ = run(lib, TwoTracks(frames=24, impact_offset_us=30000))
    assert active(hyps)
    assert verdict(lib, hyps).index == -1


def test_the_tighter_of_two_ball_like_hypotheses_wins(lib):
    hyps = make_hyps(lib)
    arm(lib, hyps)
    step = 42.0 * 0.002 / BIN_M
    jitter = [0.0, 0.6, -0.6, 0.6, -0.6, 0.6]
    for k in range(1, 7):
        ts = 2000 * k
        clean = obs(k, ts, 46.0 + step * k, 1500.0, 42.0)
        noisy = obs(k, ts, 50.0 + step * k + jitter[k - 1], 1500.0, 42.0)
        feed(lib, hyps, k, ts, [clean, noisy])
    v = verdict(lib, hyps)
    assert v.index >= 0
    assert bins(hyps.hyp[v.index])[0] == pytest.approx(46.0 + step, abs=0.01)


def test_unarmed_hypotheses_give_no_verdict(lib):
    assert verdict(lib, make_hyps(lib)).index == -1
```

- [ ] **Step 2: Run them to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py -k "verdict or classified or stationary or gate or tighter or leaving" -v`
Expected: FAIL — `function 'l3_ball_hyps_classify' not found`.

- [ ] **Step 3: Implement**

In `l3_ball_hyp.h`, after `l3_ball_hyps_set_angles`:

```c
/* The ball among the hypotheses holding at least classifyPoints points:
 * fitted over them from the gate time, it must move outward at
 * minDepartureMps..maxSpeedMps, cross the origin within impactToleranceUs of
 * the gate time and fit within maxResidualBins. The best score wins:
 * (1 - residual / maxResidualBins) + the fraction of points whose Doppler
 * agrees with the rate + half the fraction of club frames where it was the
 * weaker return. out->index is -1 when none qualifies. */
void l3_ball_hyps_classify(const l3_ball_hyps_t *hyps, l3_ball_hyp_verdict_t *out);
```

In `l3_ball_hyp.c`, add `#include "l3_club_track.h"` after `#include "l3_ball_hyp.h"`, and:

```c
void l3_ball_hyps_classify(const l3_ball_hyps_t *hyps, l3_ball_hyp_verdict_t *out)
{
    const l3_ball_hyps_cfg_t *cfg = &hyps->cfg;
    uint32_t i;

    memset(out, 0, sizeof(*out));
    out->index = -1;
    if (!hyps->armed || !(cfg->maxResidualBins > 0.0F)) {
        return;
    }
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        const l3_ball_hyp_t *hyp = &hyps->hyp[i];
        float rate;
        float atGate;
        float residual;
        float rateMps;
        float originOffsetS;
        float agree = 0.0F;
        float weaker = 0.0F;
        float withClub = 0.0F;
        float weakerFraction;
        float score;
        uint32_t k;

        if (!hyp->active || hyp->count < cfg->classifyPoints) {
            continue;
        }
        if (!l3_ball_hyp_fit(hyp, hyps->impactTimestampUs, &rate, &atGate, &residual) ||
            !(rate > 0.0F)) {
            continue;
        }
        rateMps = rate * cfg->binWidthM;
        if (rateMps < cfg->minDepartureMps || rateMps > cfg->maxSpeedMps) {
            continue;
        }
        originOffsetS = (hyps->originBin - atGate) / rate;
        if (fabsf(originOffsetS) * 1.0e6F > (float)cfg->impactToleranceUs) {
            continue;
        }
        if (residual > cfg->maxResidualBins) {
            continue;
        }
        for (k = 0U; k < hyp->count; k++) {
            const l3_ball_hyp_point_t *p = &hyp->points[k];

            if (l3_track_wrapped_diff(rateMps, p->dopplerAliasMps, cfg->velocitySpanMps) <=
                cfg->dopplerToleranceMps) {
                agree += 1.0F;
            }
            if (p->clubStat > 0.0F) {
                withClub += 1.0F;
                if (p->stat < p->clubStat) {
                    weaker += 1.0F;
                }
            }
        }
        weakerFraction = (withClub > 0.0F) ? weaker / withClub : 0.5F;
        score = (1.0F - residual / cfg->maxResidualBins) + agree / (float)hyp->count +
                0.5F * weakerFraction;
        if (out->index < 0 || score > out->score) {
            out->index = (int32_t)i;
            out->points = hyp->count;
            out->rateMps = rateMps;
            out->originOffsetUs = originOffsetS * 1.0e6F;
            out->residualBins = residual;
            out->dopplerAgreement = agree / (float)hyp->count;
            out->weakerFraction = weakerFraction;
            out->score = score;
        }
    }
}
```

In `firmware_host.py` `_SIGNATURES`:

```python
    "l3_ball_hyps_classify": ([_P(BallHyps), _P(BallHypVerdict)], None),
```

- [ ] **Step 4: Run the hypothesis tests**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py -v`
Expected: PASS (21 tests).

- [ ] **Step 5: Commit**

```bash
git add firmware/iwr6843/l3_ball_hyp.h firmware/iwr6843/l3_ball_hyp.c src/openflight/iwr6843/firmware_host.py tests/test_iwr6843_firmware_ball_hyp.py
git commit -m "iwr: classify ball hypotheses by origin, rate, fit, Doppler and strength" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: The ball track searches with the hypotheses (switchable)

**Files:**
- Modify: `firmware/iwr6843/l3_ball_track.h`, `firmware/iwr6843/l3_ball_track.c`
- Modify: `src/openflight/iwr6843/firmware_host.py` (`BallTrackCfg`, `BallTrack`, `BALL_TRACK_WHY_NAMES`, `_SIGNATURES`)
- Test: `tests/test_iwr6843_firmware_ball_track.py`

**Interfaces:**
- Consumes: Tasks 3–4 (`l3_ball_hyps_*`), the track core (`l3_track_update`, `l3_track_set_angles`, `l3_track_reset`).
- Produces: `l3_ball_track_cfg_t.useHypotheses` (default 0 until Task 8), `.skipClubClaim` (default 1), `.hyps`; `l3_ball_track_t.hyps`, `.verdict`; `L3_BALL_TRACK_WHY_SEARCHING` ("searching"); `int32_t l3_ball_track_update_joint(l3_ball_track_t *, const l3_target_obs_t *, uint32_t n, uint32_t frame, uint32_t timestampUs, uint32_t clubIndex)`; `uint32_t l3_ball_track_struct_bytes(void)`.

- [ ] **Step 1: Write the failing tests**

In `tests/test_iwr6843_firmware_ball_track.py` (which already imports `ctypes`, `math`, `pytest`, `fw`, and defines `BIN_M` and `lib`), add to the imports at the top:

```python
from iwr6843_twotrack import TwoTracks, obs
```

and append:

```python
# --- the hypothesis search (l3_ball_hyp.c) ------------------------------------


def hyp_track(lib, **overrides):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    cfg.useHypotheses = 1
    for name, value in overrides.items():
        setattr(cfg, name, value)
    track = fw.BallTrack()
    lib.l3_ball_track_init(ctypes.byref(track), ctypes.byref(cfg))
    return track


def run_joint(lib, track, scene, on_frame=None):
    origin = fw.Vec3(scene.origin_bin * BIN_M, 0.0, 0.0)
    lib.l3_ball_track_arm(ctypes.byref(track), scene.origin_bin, ctypes.byref(origin), scene.gate_us)
    whys = []
    for f in scene.build():
        arr = (fw.TargetObs * max(1, len(f.targets)))(*f.targets)
        lib.l3_ball_track_update_joint(
            ctypes.byref(track), arr, len(f.targets), f.frame, f.timestamp_us, f.club_index
        )
        whys.append(fw.BALL_TRACK_WHY_NAMES[track.why])
        if on_frame is not None:
            on_frame(track, f)
    return whys


def core_bins(lib, track):
    out = []
    for i in range(track.core.count):
        p = fw.TrackPoint()
        lib.l3_track_point(ctypes.byref(track.core), i, ctypes.byref(p))
        out.append(round(p.rangeBin, 3))
    return out


def launch_of(lib, track):
    out = fw.Launch()
    used = lib.l3_ball_track_launch(ctypes.byref(track), ctypes.byref(out))
    return used, out


def test_the_hypothesis_search_is_off_by_default_until_evaluated(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    assert (cfg.useHypotheses, cfg.skipClubClaim) == (0, 1)
    assert cfg.hyps.classifyPoints == 4


def test_the_hypotheses_share_the_core_geometry(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    cfg.core.binWidthM = 0.05
    cfg.core.velocitySpanMps = 12.0
    track = fw.BallTrack()
    lib.l3_ball_track_init(ctypes.byref(track), ctypes.byref(cfg))
    assert (track.hyps.cfg.binWidthM, track.hyps.cfg.velocitySpanMps) == pytest.approx((0.05, 12.0))


def test_the_ball_track_layout_matches_the_c(lib):
    assert ctypes.sizeof(fw.BallTrack) == lib.l3_ball_track_struct_bytes()


def test_with_the_search_off_the_joint_update_is_todays_update(lib):
    scene = TwoTracks()
    a = hyp_track(lib, useHypotheses=0)
    run_joint(lib, a, scene)
    b = fw.BallTrack()
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    lib.l3_ball_track_init(ctypes.byref(b), ctypes.byref(cfg))
    origin = fw.Vec3(scene.origin_bin * BIN_M, 0.0, 0.0)
    lib.l3_ball_track_arm(ctypes.byref(b), scene.origin_bin, ctypes.byref(origin), scene.gate_us)
    for f in scene.build():
        arr = (fw.TargetObs * max(1, len(f.targets)))(*f.targets)
        lib.l3_ball_track_update(ctypes.byref(b), arr, len(f.targets), f.frame, f.timestamp_us)
    assert bytes(a.core) == bytes(b.core)
    assert list(a.counters) == list(b.counters)


def test_beside_the_club_the_ball_track_is_the_ball(lib):
    scene = TwoTracks()
    track = hyp_track(lib)
    whys = run_joint(lib, track, scene)
    assert whys[:3] == ["searching"] * 3
    assert whys[3] == "confirmed" and set(whys[4:]) == {"tracked"}
    frames = scene.build()
    assert core_bins(lib, track) == [round(f.ball_bin, 3) for f in frames]
    used, launch = launch_of(lib, track)
    assert used >= 3 and launch.speedMps == pytest.approx(42.0, rel=0.05)


def test_a_lone_ball_is_found_without_a_club_track(lib):
    track = hyp_track(lib)
    run_joint(lib, track, TwoTracks(club_visible=False))
    used, launch = launch_of(lib, track)
    assert track.confirmed and launch.speedMps == pytest.approx(42.0, rel=0.05)


def test_without_a_ball_there_is_no_launch(lib):
    scene = TwoTracks(missing_ball=tuple(range(1, 9)), extras=[(48.0, 20000.0, 0.8)])
    track = hyp_track(lib)
    whys = run_joint(lib, track, scene)
    assert set(whys) == {"searching"}
    assert not track.confirmed
    assert launch_of(lib, track)[0] == 0


def test_a_confirmed_ball_skips_the_club_claim_while_another_candidate_is_in_gate(lib):
    scene = TwoTracks(frames=5)
    track = hyp_track(lib)
    run_joint(lib, track, scene)
    assert track.confirmed
    step = 42.0 * 0.002 / BIN_M
    expected = 46.0 + step * 6
    club = obs(6, 12000, expected - 0.1, 9000.0, 40.0)
    ball = obs(6, 12000, expected + 0.6, 1500.0, 42.0)
    arr = (fw.TargetObs * 2)(club, ball)
    assert lib.l3_ball_track_update_joint(ctypes.byref(track), arr, 2, 6, 12000, 0) == 1
    assert track.lastTargetIndex == 1
    # The club's claim alone in the gate: the two share a bin, and it is taken.
    lone = obs(7, 14000, 46.0 + step * 7, 9000.0, 40.0)
    arr = (fw.TargetObs * 1)(lone)
    assert lib.l3_ball_track_update_joint(ctypes.byref(track), arr, 1, 7, 14000, 0) == 1


def test_angles_on_hypothesis_points_survive_adoption(lib):
    both = fw.ANGLE_AZIMUTH | fw.ANGLE_ELEVATION

    def angle_every_new_point(track, _frame):
        for i in range(fw.BALL_HYP_MAX):
            if track.hyps.hyp[i].lastTargetIndex != fw.BALL_HYP_NONE:
                lib.l3_ball_hyps_set_angles(ctypes.byref(track.hyps), i, 0.02, 0.2, both)

    track = hyp_track(lib)
    run_joint(lib, track, TwoTracks(frames=4), on_frame=angle_every_new_point)
    assert track.confirmed
    for i in range(3):  # the points adopted from earlier frames
        p = fw.TrackPoint()
        lib.l3_track_point(ctypes.byref(track.core), i, ctypes.byref(p))
        assert p.anglesValid == both
```

- [ ] **Step 2: Run them to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_track.py -v`
Expected: FAIL — `AttributeError: useHypotheses` / `l3_ball_track_update_joint not found`.

- [ ] **Step 3: Implement the header changes**

`l3_ball_track.h`: add `#include "l3_ball_hyp.h"` after `#include "l3_frames.h"`. Append to `l3_ball_track_cfg_t` (after `snr`):

```c
    /* Search for the ball with the hypotheses (l3_ball_hyp.h) instead of
     * taking the most confident departing target; set from the recorded
     * captures (docs/superpowers/specs/2026-09-28-iwr-joint-club-ball-tracking.md). */
    uint32_t useHypotheses;
    /* Once confirmed, skip the club's claimed target while another candidate
     * is in the gate. */
    uint32_t skipClubClaim;
    l3_ball_hyps_cfg_t hyps;      /* binWidthM and velocitySpanMps come from core */
```

Add `L3_BALL_TRACK_WHY_SEARCHING,   /* hypotheses kept; none is the ball yet */` immediately before `L3_BALL_TRACK_WHY_COUNT`. Append to `l3_ball_track_t` after `counters`:

```c
    l3_ball_hyps_t hyps;
    l3_ball_hyp_verdict_t verdict; /* the last classification; index -1 before one */
```

After `l3_ball_track_update`:

```c
/* The same, beside the club track: clubIndex is the target l3_track_follow
 * claimed this frame (L3_TRACK_NO_TARGET for none). With useHypotheses the
 * ball is searched for with the hypotheses and, once one is classified, its
 * points seed the track (angles included) and tracking continues; the plain
 * update is this with no claim. */
int32_t l3_ball_track_update_joint(l3_ball_track_t *track, const l3_target_obs_t *targets,
                                   uint32_t n, uint32_t frame, uint32_t timestampUs,
                                   uint32_t clubIndex);
/* sizeof(l3_ball_track_t), for the ctypes mirror's layout check. */
uint32_t l3_ball_track_struct_bytes(void);
```

- [ ] **Step 4: Implement the source changes**

In `l3_ball_track.c`:

`kWhyNames` gains `"searching"` as its last entry (after `"lost"`).

`l3_ball_track_cfg_defaults`, after `cfg->snr = 3.0F;`:

```c
    cfg->useHypotheses = 0U;          /* decided by the recorded captures */
    cfg->skipClubClaim = 1U;
    l3_ball_hyps_cfg_defaults(&cfg->hyps);
```

`l3_ball_track_init` becomes:

```c
void l3_ball_track_init(l3_ball_track_t *track, const l3_ball_track_cfg_t *cfg)
{
    memset(track, 0, sizeof(*track));
    track->cfg = *cfg;
    /* The hypotheses share the core's geometry: one source for both. */
    track->cfg.hyps.binWidthM = cfg->core.binWidthM;
    track->cfg.hyps.velocitySpanMps = cfg->core.velocitySpanMps;
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    track->verdict.index = -1;
    l3_track_init(&track->core, &cfg->core);
    l3_ball_hyps_init(&track->hyps, &track->cfg.hyps);
}
```

`l3_ball_track_reset`, before its closing brace:

```c
    l3_ball_hyps_init(&track->hyps, &track->cfg.hyps);
    memset(&track->verdict, 0, sizeof(track->verdict));
    track->verdict.index = -1;
```

`l3_ball_track_arm`, after `track->impactTimestampUs = impactTimestampUs;`:

```c
    l3_ball_hyps_arm(&track->hyps, originBin, impactTimestampUs);
```

Rename today's `l3_ball_track_update` body to a static function with the club's claim, and change its confirmed branch:

```c
/* The core's gate around its prediction for this frame. */
static int32_t l3_ball_track_inGate(const l3_club_track_t *core, const l3_target_obs_t *target,
                                    uint32_t frame)
{
    float predicted = core->lastBin + core->velocityBinsPerFrame * (float)(frame - core->lastFrame);
    float error = target->rangeBin - predicted;

    return ((error < 0.0F) ? -error : error) <= core->cfg.gateBins;
}

static int32_t l3_ball_track_step(l3_ball_track_t *track, const l3_target_obs_t *targets,
                                  uint32_t n, uint32_t frame, uint32_t timestampUs,
                                  uint32_t skipIndex)
```

Its body is today's `l3_ball_track_update` body, with the confirmed branch's loop replaced by:

```c
    if (track->core.active && track->confirmed) {
        uint8_t otherInGate = 0U;

        for (i = 0U; i < n && kept < L3_OBS_MAX_TARGETS; i++) {
            if (targets[i].rangeBin < track->core.lastBin - 0.5F || i == skipIndex) {
                continue;
            }
            otherInGate |= (uint8_t)l3_ball_track_inGate(&track->core, &targets[i], frame);
            indices[kept] = i;
            candidates[kept++] = targets[i];
        }
        if (skipIndex < n && !otherInGate && kept < L3_OBS_MAX_TARGETS &&
            targets[skipIndex].rangeBin >= track->core.lastBin - 0.5F) {
            /* Only the club's return is in the gate: the two share a bin. */
            indices[kept] = skipIndex;
            candidates[kept++] = targets[skipIndex];
        }
    } else {
```

(the `else` branch — the departure band — is unchanged). Fix the stale comment above it: "Before acquisition every candidate in the departure band is offered and the core takes the most confident; the ball does not reliably outrun the club's follow-through at once."

Then add:

```c
/* The classified hypothesis becomes the track: its points (angles included)
 * seed the core in order, and tracking carries on from them. */
static int32_t l3_ball_track_adopt(l3_ball_track_t *track, uint32_t index)
{
    const l3_ball_hyp_t *hyp = &track->hyps.hyp[index];
    uint32_t k;

    l3_track_reset(&track->core);
    for (k = 0U; k < hyp->count; k++) {
        const l3_ball_hyp_point_t *p = &hyp->points[k];
        l3_target_obs_t seed;

        memset(&seed, 0, sizeof(seed));
        seed.frame = p->frame;
        seed.timestampUs = p->timestampUs;
        seed.peakBin = (uint8_t)(p->rangeBin + 0.5F);
        seed.rangeBin = p->rangeBin;
        seed.stat = p->stat;
        seed.peak = p->stat;
        seed.dopplerAliasMps = p->dopplerAliasMps;
        seed.coherence = 1.0F;
        seed.confidence = 1.0F;
        if (!l3_track_update(&track->core, &seed, 1U, p->frame, p->timestampUs)) {
            l3_track_reset(&track->core);
            return l3_ball_track_note(track, L3_BALL_TRACK_WHY_SEARCHING, 0);
        }
        if (p->anglesValid) {
            (void)l3_track_set_angles(&track->core, p->azimuthRad, p->elevationRad,
                                      p->anglesValid);
        }
    }
    track->confirmed = 1U;
    track->lastTargetIndex = hyp->lastTargetIndex;
    return l3_ball_track_note(track, L3_BALL_TRACK_WHY_CONFIRMED,
                              (hyp->lastTargetIndex != L3_BALL_HYP_NONE) ? 1 : 0);
}

int32_t l3_ball_track_update_joint(l3_ball_track_t *track, const l3_target_obs_t *targets,
                                   uint32_t n, uint32_t frame, uint32_t timestampUs,
                                   uint32_t clubIndex)
{
    uint32_t skip = track->cfg.skipClubClaim ? clubIndex : L3_TRACK_NO_TARGET;

    if (!track->cfg.useHypotheses) {
        return l3_ball_track_step(track, targets, n, frame, timestampUs, L3_TRACK_NO_TARGET);
    }
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    if (!track->armed) {
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_UNARMED, 0);
    }
    if (track->done) {
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_LOST, 0);
    }
    if (track->confirmed) {
        return l3_ball_track_step(track, targets, n, frame, timestampUs, skip);
    }
    (void)l3_ball_hyps_update(&track->hyps, targets, n, frame, timestampUs, clubIndex);
    l3_ball_hyps_classify(&track->hyps, &track->verdict);
    if (track->verdict.index < 0) {
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_SEARCHING, 0);
    }
    return l3_ball_track_adopt(track, (uint32_t)track->verdict.index);
}

int32_t l3_ball_track_update(l3_ball_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                             uint32_t frame, uint32_t timestampUs)
{
    return l3_ball_track_update_joint(track, targets, n, frame, timestampUs, L3_TRACK_NO_TARGET);
}

uint32_t l3_ball_track_struct_bytes(void)
{
    return (uint32_t)sizeof(l3_ball_track_t);
}
```

`l3_ball_track_note` is defined above `l3_ball_track_step` already; keep `l3_ball_track_adopt` below both.

- [ ] **Step 5: Update the ctypes mirrors**

`firmware_host.py`: append `"searching"` to `BALL_TRACK_WHY_NAMES`. Append to `BallTrackCfg._fields_`:

```python
        ("useHypotheses", ctypes.c_uint32),
        ("skipClubClaim", ctypes.c_uint32),
        ("hyps", BallHypsCfg),
```

and to `BallTrack._fields_` (after `counters`):

```python
        ("hyps", BallHyps),
        ("verdict", BallHypVerdict),
```

Add to `_SIGNATURES`:

```python
    "l3_ball_track_update_joint": (
        [_P(BallTrack), _P(TargetObs), _U32, _U32, _U32, _U32],
        ctypes.c_int32,
    ),
    "l3_ball_track_struct_bytes": ([], _U32),
```

- [ ] **Step 6: Run the ball-track, result and hypothesis tests**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_track.py tests/test_iwr6843_firmware_result.py tests/test_iwr6843_firmware_ball_hyp.py -v`
Expected: PASS. `test_why_names_and_formats` covers the new "searching" name against the C table.

- [ ] **Step 7: Run the replay and viewer suites (the search is still off by default)**

Run: `uv run pytest tests/test_iwr6843_firmware_replay.py tests/test_iwr6843_dump_viewer.py -q`
Expected: PASS, unchanged.

- [ ] **Step 8: Commit**

```bash
git add firmware/iwr6843/l3_ball_track.h firmware/iwr6843/l3_ball_track.c src/openflight/iwr6843/firmware_host.py tests/test_iwr6843_firmware_ball_track.py
git commit -m "iwr: switchable hypothesis search in the ball track, joint with the club claim" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: Board, replay and viewer make the same calls

**Files:**
- Modify: `firmware/iwr6843/l3_dump.c` (`l3_considerBallTrack`)
- Modify: `src/openflight/iwr6843/firmware_replay.py`
- Modify: `src/openflight/iwr6843/dump_viewer.py`, `scripts/iwr6843/dump_viewer.html`
- Test: `tests/test_iwr6843_firmware_replay.py`, `tests/test_iwr6843_dump_viewer.py`

**Interfaces:**
- Consumes: Task 5's `l3_ball_track_update_joint`, `track.hyps`; Task 3's `l3_ball_hyp_fit`, `l3_ball_hyps_set_angles`; the club track's `lastTargetIndex` after `l3_track_follow`.
- Produces: `ReplayConfig.ball_hypotheses: bool | None = None`; `HypothesisSummary(id: int, points: tuple[tuple[int, float], ...])`; `ReplayFrame.ball_hypotheses: tuple[HypothesisSummary, ...] = ()`; `ViewerOptions.ball_hypotheses: bool | None = None`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_iwr6843_firmware_replay.py`:

```python
def test_the_hypothesis_search_recovers_the_synthetic_launch(lib, whole_shot):
    result = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN, ball_hypotheses=True), lib=lib)
    assert result.launch is not None
    assert result.launch.speed_mps == pytest.approx(60.0, rel=0.1)
    seen = [frame.ball_hypotheses for frame in result.frames if frame.ball_hypotheses]
    assert seen and all(len(snapshot) <= fw.BALL_HYP_MAX for snapshot in seen)


def test_the_search_switch_leaves_the_default_replay_alone(lib, whole_shot):
    default = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN), lib=lib)
    off = replay_dump(whole_shot, ReplayConfig(tee_bin=TEE_BIN, ball_hypotheses=False), lib=lib)
    assert [p.range_bin for p in default.ball_points] == [p.range_bin for p in off.ball_points]
    assert all(frame.ball_hypotheses == () for frame in off.frames)
```

Append to `tests/test_iwr6843_dump_viewer.py`:

```python
@needs_compiler
def test_the_viewer_can_switch_the_ball_search_and_shows_the_hypotheses():
    raw = synth_shot_dump(
        path_deg=3.0, hla_deg=2.0, vla_deg=12.0, ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M
    )
    options = dv.ViewerOptions.from_mapping({"tee_bin": TEE_BIN, "ball_hypotheses": "true"})
    assert options.ball_hypotheses is True
    data = dv.analyze_dump(raw, options)
    json.dumps(data, allow_nan=False)
    frames = data["firmware"]["frames"]
    assert any(frame["ball_hypotheses"] for frame in frames)
    first = next(frame["ball_hypotheses"] for frame in frames if frame["ball_hypotheses"])
    assert {"id", "points"} <= set(first[0])
```

- [ ] **Step 2: Run them to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_replay.py tests/test_iwr6843_dump_viewer.py -k "hypothes or search" -v`
Expected: FAIL — `unexpected keyword argument 'ball_hypotheses'`.

- [ ] **Step 3: Replay changes**

In `firmware_replay.py`:

Add to `ReplayConfig` (after `retain`):

```python
    # The ball search: True/False sets l3_ball_track_cfg_t.useHypotheses for
    # this replay; None keeps the firmware default.
    ball_hypotheses: bool | None = None
```

Add, next to `PointSummary`:

```python
@dataclass(frozen=True)
class HypothesisSummary:
    """One ball hypothesis after a frame: its id and (frame, global bin) points."""

    id: int
    points: tuple[tuple[int, float], ...]
```

Append to `ReplayFrame` (last field): `ball_hypotheses: tuple[HypothesisSummary, ...] = ()`.

In `replay_dump`, after `lib.l3_ball_track_cfg_defaults(ctypes.byref(ball_cfg))` and the three `ball_cfg.core.*` lines:

```python
    if config.ball_hypotheses is not None:
        ball_cfg.useHypotheses = 1 if config.ball_hypotheses else 0
```

In `_replay_post_frame`, replace the ball update call with the joint one, and after the existing angle block for the core point add the hypothesis angles; return the snapshot in the frame:

```python
    appended = lib.l3_ball_track_update_joint(
        ctypes.byref(ball_track), targets, found, frame, timestamp_us, track.lastTargetIndex
    )
```

```python
    _hypothesis_angles(
        lib, cal, cube, frame, window_start, n_tx, targets, found, ball_track, chirp_period_s
    )
```

and in the returned `ReplayFrame(...)`, after `ball_bin,` pass `ball_hypotheses=_hypothesis_summaries(ball_track)` (convert the positional return to keyword arguments for the fields after `ball_bin` if needed: `retain=None, ball_hypotheses=...`).

Add the helpers:

```python
def _hypothesis_angles(  # pylint: disable=too-many-arguments
    lib, cal, cube, frame, window_start, n_tx, targets, found, ball_track, chirp_period_s
) -> None:
    """Angles for every ball-hypothesis point appended this frame, as the board
    estimates them, so the chosen ball keeps angles on its early points."""
    hyps = ball_track.hyps
    rate, at, residual = ctypes.c_float(), ctypes.c_float(), ctypes.c_float()
    for index in range(fw.BALL_HYP_MAX):
        hyp = hyps.hyp[index]
        if not hyp.active or hyp.lastTargetIndex >= found:
            continue
        newest = hyp.points[hyp.count - 1]
        radial = (
            rate.value * hyps.cfg.binWidthM
            if lib.l3_ball_hyp_fit(
                ctypes.byref(hyp),
                newest.timestampUs,
                ctypes.byref(rate),
                ctypes.byref(at),
                ctypes.byref(residual),
            )
            else 0.0
        )
        obs_angle, flags = _estimate_angles(
            lib,
            cal,
            cube,
            frame,
            window_start,
            n_tx,
            targets[hyp.lastTargetIndex],
            radial,
            chirp_period_s,
        )
        if obs_angle is not None:
            lib.l3_ball_hyps_set_angles(
                ctypes.byref(hyps), index, obs_angle.azimuthRad, obs_angle.elevationRad, flags
            )


def _hypothesis_summaries(ball_track) -> tuple[HypothesisSummary, ...]:
    """The active ball hypotheses after a frame, for the viewer."""
    out = []
    for index in range(fw.BALL_HYP_MAX):
        hyp = ball_track.hyps.hyp[index]
        if hyp.active:
            out.append(
                HypothesisSummary(
                    int(hyp.id),
                    tuple(
                        (int(hyp.points[k].frame), float(hyp.points[k].rangeBin))
                        for k in range(hyp.count)
                    ),
                )
            )
    return tuple(out)
```

Add `"HypothesisSummary"` to `__all__`.

- [ ] **Step 4: Board changes (same calls, same order)**

In `l3_dump.c` `l3_considerBallTrack`, the ball update (Task 0 already renamed the frame number to `frameIndex`) becomes:

```c
    (void)l3_track_follow(&gClubTrack, targets, found, frameIndex, gPostTimestampUs);
    if (l3_ball_track_update_joint(&gBallTrack, targets, found, frameIndex, gPostTimestampUs,
                                   gClubTrack.lastTargetIndex) &&
```

(the core-point angle block that follows is unchanged). After that block and before `(void)l3_ball_track_launch(&gBallTrack, &gLaunch);` add:

```c
    {
        /* Angles for every ball-hypothesis point this frame appended: once one
         * is chosen, its early points must still carry them. */
        uint32_t index;

        for (index = 0U; index < L3_BALL_HYP_MAX; index++) {
            const l3_ball_hyp_t *hyp = &gBallTrack.hyps.hyp[index];
            const l3_target_obs_t *hit;
            l3_angle_obs_t angle;
            float rate;
            float at;
            float residual;
            float radial = 0.0F;

            if (!hyp->active || hyp->lastTargetIndex >= found) {
                continue;
            }
            hit = &targets[hyp->lastTargetIndex];
            if (l3_ball_hyp_fit(hyp, hyp->points[hyp->count - 1U].timestampUs, &rate, &at,
                                &residual)) {
                radial = rate * gBallTrack.hyps.cfg.binWidthM;
            }
            l3_channelSnapshot(&frame, (uint32_t)hit->peakBin - frame.binStart,
                               hit->dopplerPhaseRad, radial, &snapshot);
            if (l3_angle_estimate(&gRadarCal, &snapshot, &angle)) {
                uint8_t flags = 0U;

                if (angle.azimuthValid) {
                    flags |= L3_OBS_ANGLE_AZIMUTH;
                }
                if (angle.elevationValid) {
                    flags |= L3_OBS_ANGLE_ELEVATION;
                }
                (void)l3_ball_hyps_set_angles(&gBallTrack.hyps, index, angle.azimuthRad,
                                              angle.elevationRad, flags);
                gAngleEstimates++;
            }
        }
    }
```

Note for the hardware check (not host-testable): this adds up to `L3_BALL_HYP_MAX` angle estimates per post frame; compare `triggerLog perf` for `L3_PROF_BALL_TRACK` before and after on the board.

- [ ] **Step 5: Viewer changes**

`dump_viewer.py`: add to `ViewerOptions` (after `py_hits`) `ball_hypotheses: bool | None = None`; in `ViewerOptions.from_mapping` the `"bool" in kind` branch already handles `bool | None`; in `firmware_section`'s `fr.ReplayConfig(...)` pass `ball_hypotheses=options.ball_hypotheses`.

`dump_viewer.html`: in the "Firmware replay" form add

```html
      <label>ball search <select id="ball_hypotheses"><option value="">firmware default</option><option value="true">hypotheses</option><option value="false">legacy</option></select></label>
```

add `"ball_hypotheses"` to the `OPTS` array, and in `renderMap`, after the ball-track trace:

```js
    const latest = {};
    F.frames.forEach((fr) => (fr.ball_hypotheses || []).forEach((h) => (latest[h.id] = h.points)));
    const hx = [], hy = [];
    Object.values(latest).forEach((pts) => {
      pts.forEach(([f, b]) => { hx.push(binM(b)); hy.push(T(f)); });
      hx.push(null); hy.push(null);
    });
    if (hx.length) traces.push({ type: "scatter", mode: "lines+markers", name: "ball hypotheses", x: hx, y: hy,
      line: { color: "rgba(255,146,43,.45)", width: 1, dash: "dot" }, marker: { size: 4, color: "rgba(255,146,43,.45)" }, hoverinfo: "skip" });
```

- [ ] **Step 6: Run the suites**

Run: `uv run pytest tests/test_iwr6843_firmware_replay.py tests/test_iwr6843_dump_viewer.py tests/test_iwr6843_firmware_ball_track.py -v`
Expected: PASS.

- [ ] **Step 7: Check the viewer by hand**

Run: `uv run python scripts/iwr6843/dump_viewer.py iwr-test-sessions/iwr6843/iwr6843_20260927_131018_934_002.l3dump`
Set "ball search" to "hypotheses": the range × time map shows dotted orange "ball hypotheses" lines; the state lanes' ball track shows "searching" before "confirmed".

- [ ] **Step 8: Commit**

```bash
git add firmware/iwr6843/l3_dump.c src/openflight/iwr6843/firmware_replay.py src/openflight/iwr6843/dump_viewer.py scripts/iwr6843/dump_viewer.html tests/test_iwr6843_firmware_replay.py tests/test_iwr6843_dump_viewer.py
git commit -m "iwr: board, replay and viewer run the joint ball search" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 7: The 2026-08-24 capture as a recording

**Files:**
- Create: `tests/radar/recordings/iwr6843_20260824_120408_601_001.l3dump` (copied)
- Modify: `tests/radar/recordings/manifest.json`

**Interfaces:**
- Consumes: `Expectation` keys `fires`, `ball_speed_mps`, `club_points_min` (existing).
- Produces: a manifest entry the existing `test_recorded_swings_meet_their_manifest_expectations` runs.

- [ ] **Step 1: Copy the capture**

```bash
cp "E:/Users/corma/Downloads/openflight_trackman_sessions_2026-07-14_to_2026-08-24/openflight_trackman_sessions/sessions/2026-08-24/openflight/iwr6843/iwr6843_20260824_120408_601_001.l3dump" tests/radar/recordings/
```

- [ ] **Step 2: Add the manifest entry**

In `tests/radar/recordings/manifest.json` add (keys override the `default` entry's 2026-09-16 values):

```json
  "iwr6843_20260824_120408_601_001.l3dump": {
    "tee_bin": 39,
    "dest_bin": null,
    "post_from_frame": null,
    "pitch_deg": 10.5,
    "notes": "2026-08-24 trackman session, wide 24f3ms 53-bin IQ16, sound-triggered; OPS ball 100.8 mph (45.1 m/s). The session's tee (1.524 m, bin 33) fires the gate at frame 4; tee_bin 39 is what the replay needs to fire near impact (frame 7). After impact a strong stall sits at 38.2-38.4, the club carries on 40.1 -> 43.1, and the weak ball leaves 44.7 -> 84.8 at ~2.7 bins per 3 ms.",
    "expect": {"fires": true, "ball_speed_mps": [40, 50]}
  }
```

- [ ] **Step 3: Run the recordings test**

Run: `uv run pytest tests/test_iwr6843_firmware_replay.py -k recorded -v`
Expected: PASS for every recording including the new one (the legacy search launches it at 44.2 m/s at tee bin 39).

- [ ] **Step 4: Commit**

```bash
git add tests/radar/recordings/iwr6843_20260824_120408_601_001.l3dump tests/radar/recordings/manifest.json
git commit -m "iwr: record the 2026-08-24 two-track capture with its launch expectation" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 8: Judge the search on the captures and set its default

**Files:**
- Modify: `scripts/analysis/evaluate_iwr_tracking.py` (`--ball-hypotheses`)
- Modify: `tests/test_evaluate_iwr_tracking.py`
- Modify (only if the criteria pass): `firmware/iwr6843/l3_ball_track.c` (`useHypotheses` default), `tests/test_iwr6843_firmware_ball_track.py`
- Modify: `docs/superpowers/specs/2026-09-28-iwr-joint-club-ball-tracking.md` (results)

**Interfaces:**
- Consumes: Task 1's `evaluate(..., ball_hypotheses=)`, Task 6's `ReplayConfig.ball_hypotheses`.
- Produces: the decision, recorded in the spec.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_evaluate_iwr_tracking.py`:

```python
def test_the_cli_passes_the_ball_search_through(ev, monkeypatch, tmp_path):
    seen = []
    monkeypatch.setattr(ev, "iter_cases", lambda roots: iter([object()]))

    def fake_evaluate(case, *, lib=None, ball_hypotheses=None):
        seen.append(ball_hypotheses)
        return ev.Outcome("x", "club", "ok", True, 40.0, 40.0)

    monkeypatch.setattr(ev, "evaluate", fake_evaluate)
    assert ev.main([str(tmp_path), "--ball-hypotheses", "on"]) == 0
    assert ev.main([str(tmp_path), "--ball-hypotheses", "off"]) == 0
    assert ev.main([str(tmp_path)]) == 0
    assert seen == [True, False, None]
```

- [ ] **Step 2: Run it to verify it fails**

Run: `uv run pytest tests/test_evaluate_iwr_tracking.py -k ball_search -v`
Expected: FAIL — `unrecognized arguments: --ball-hypotheses on`.

- [ ] **Step 3: Add the flag**

In `main`:

```python
    parser.add_argument(
        "--ball-hypotheses",
        choices=("firmware", "on", "off"),
        default="firmware",
        help="Ball search: the firmware default, the hypotheses, or the legacy acquisition",
    )
```

and

```python
    search = {"firmware": None, "on": True, "off": False}[args.ball_hypotheses]
    outcomes = [evaluate(case, ball_hypotheses=search) for case in iter_cases(args.roots)]
```

- [ ] **Step 4: Run the script's tests**

Run: `uv run pytest tests/test_evaluate_iwr_tracking.py -v`
Expected: PASS.

- [ ] **Step 5: Evaluate both searches**

```bash
uv run python scripts/analysis/evaluate_iwr_tracking.py iwr-test-sessions --ball-hypotheses off --compare docs/superpowers/specs/2026-09-28-tracking-baseline.json
uv run python scripts/analysis/evaluate_iwr_tracking.py iwr-test-sessions --ball-hypotheses on --json hypotheses.json --compare docs/superpowers/specs/2026-09-28-tracking-baseline.json --allow-more-none 5
```

Expected: the first prints the baseline numbers and exits 0 (the legacy path is unchanged). Decision rule (spec R8) for the second: the hypotheses become the default only if ball `ok` > 15, `none` <= 13, club `club` >= 55 (the `--compare` exit code covers the last two and `ok` >= 15; check `ok` > 15 by eye), and Step 7's recordings pass with the search on.

- [ ] **Step 6: Look at what changed, capture by capture**

```bash
uv run python - <<'EOF'
import json
off = {o["name"]: o for o in json.load(open("docs/superpowers/specs/2026-09-28-tracking-baseline.json"))["outcomes"]}
on = {o["name"]: o for o in json.load(open("hypotheses.json"))["outcomes"]}
for name in sorted(on):
    if off[name]["ball"] != on[name]["ball"]:
        print(name, off[name]["ball"], "->", on[name]["ball"], off[name]["launch_mps"], "->", on[name]["launch_mps"], "OPS", round(on[name]["ops_mps"], 1))
EOF
```

Open two of the captures that got worse in the dump viewer with "ball search: hypotheses" and note why in the task report (which hypothesis won, which test rejected the ball).

- [ ] **Step 7: If the criteria pass, flip the default**

In `l3_ball_track_cfg_defaults`: `cfg->useHypotheses = 1U;` with the comment `/* 2026-09-28: <ok>/93 within 15 % of OPS vs 15 legacy */` (fill in the measured number). In `tests/test_iwr6843_firmware_ball_track.py` rename `test_the_hypothesis_search_is_off_by_default_until_evaluated` to `test_the_hypothesis_search_is_on_by_default` and assert `(cfg.useHypotheses, cfg.skipClubClaim) == (1, 1)`. Then run:

Run: `uv run pytest tests/ -q -p no:cacheprovider`
Expected: no failures beyond the 56 recorded on 2026-09-28 (camera, cloud, geekworm, desktop launcher, serial latency, sim transport, `compact_iq16`/`live_selector` hard-coding `cc`, `self_trigger`, `memory_layout` against a stale local map). If a recording's expectation fails with the search on, do not flip the default: record it in the spec instead.

If the criteria do not pass, leave the default at 0 and continue to Step 8.

- [ ] **Step 8: Record the results in the spec**

Append to `docs/superpowers/specs/2026-09-28-iwr-joint-club-ball-tracking.md`:

```markdown
## Results (<date>)

| search | club at impact | ball within 15 % | no launch | ball present |
|---|---|---|---|---|
| legacy | <club> | <ok> | <none> | <present> |
| hypotheses | <club> | <ok> | <none> | <present> |

Default: <useHypotheses value> because <one sentence>. Captures that got worse: <names and reasons from Step 6>.
```

(fill every `<...>` from Step 5's output before committing — no placeholders in the committed spec).

- [ ] **Step 9: Lint and commit**

```bash
uv run ruff check scripts/analysis/evaluate_iwr_tracking.py tests/test_evaluate_iwr_tracking.py src/openflight/iwr6843/
uv run pylint src/openflight/iwr6843/firmware_replay.py src/openflight/iwr6843/dump_viewer.py src/openflight/iwr6843/firmware_host.py --fail-under=9
git add scripts/analysis/evaluate_iwr_tracking.py tests/test_evaluate_iwr_tracking.py docs/superpowers/specs/2026-09-28-iwr-joint-club-ball-tracking.md firmware/iwr6843/l3_ball_track.c tests/test_iwr6843_firmware_ball_track.py
git commit -m "iwr: judge the ball search on the recorded captures and set its default" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
