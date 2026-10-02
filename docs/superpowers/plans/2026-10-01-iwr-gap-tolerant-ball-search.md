# IWR6843 Gap-Tolerant Ball Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the IWR6843 ball-hypothesis search gap-tolerant and kinematics-driven (impact anchor, distance-vs-time corridor, impact-region coasting, deceleration reject, back-projection-first score, backward recovery) and judge it against legacy on the recorded captures with a corrected label.

**Architecture:** Pure-C firmware modules under `firmware/iwr6843/` built for the host by `firmware_host.py` and driven from Python through ctypes mirrors. A new `l3_ball_anchor.c` holds the impact anchor; `l3_ball_hyp.c` gains the forward-search rules; a new `l3_ball_recover.c` holds the post-impact target history and the backward pass, which `l3_ball_track.c` owns and feeds itself so the board (`l3_dump.c`) and the replay (`firmware_replay.py`) make no new calls. The evaluator (`scripts/analysis/evaluate_iwr_tracking.py`) gets a gap-tolerant label first, then the split acceptance.

**Tech Stack:** C99 (host gcc/clang via `firmware_host.build_firmware_library`; TI armcl for the board), Python 3.11 + ctypes, pytest, `uv`.

**Spec:** `docs/superpowers/specs/2026-10-01-iwr-gap-tolerant-ball-search-design.md`

## Global Constraints

- Always run Python through `uv run` (`uv run pytest …`, `uv run pylint …`, `uv run ruff …`).
- Lint: `uv run pylint src/openflight/ --fail-under=9`; `uv run ruff check src/openflight/`; `uv run ruff format --check src/openflight/`.
- **Legacy acquisition must be unchanged** (`useHypotheses = 0`): it keeps `acceptFromBin` as its origin and `gateUs` as its impact time. Every existing legacy test and recording manifest must pass untouched.
- `useHypotheses` default stays 0. Board build keeps `L3_BALL_HYPOTHESES=0` and `L3_BALL_RECOVER=0`.
- No setting assumes a frame period: distances in metres, speeds in m/s, times in µs. Points stay in bins internally.
- Doppler is never a per-point gate (only classification evidence and the recovery tie-break).
- Every C struct change gets its ctypes mirror updated in `src/openflight/iwr6843/firmware_host.py` with a `*_struct_bytes()` size test.
- R7: board and replay make the same calls in the same order.
- Default values (spec): `spawnBehindM 0.046875`, `spawnBeyondM 0.46875`, `gateM 0.0703125`, `gateMps 8`, `coastUs 6000`, `impactCoastUs 18000`, `impactRegionM 0.5`, `classifyPoints 4`, `minDepartureMps 10`, `maxSpeedMps 100`, `maxResidualBins 1`, `dopplerToleranceMps 2.5`, `farWindowM 0`, `corridorGate 1`, `anchorRangeTolM 0.1`, `maxDecelMps2 200`, `rangeNoiseM 0.012`, weights `3, 2, 1, 1, 0.5, 0.5`; ball track `gateTolUs 15000`, `anchorMaxSigmaUs 3000`, `recover 1`, `historySnr` = 0 (meaning "same as snr"); recover `gateM 0.028125` (0.6 bin), `tieBins 0.1`; `L3_BALL_HISTORY_FRAMES 24`, `L3_BALL_HISTORY_TARGETS 6`; anchor minimum tolerance 2000 µs.

## Review Focus

1. **Timestamp wrap across 2³² µs** in corridor, coast and recovery arithmetic — expect identical decisions to the unwrapped scene. Test added in Task 6 and Task 9.
2. **Anchor from a club track with too few points or a fit that is not finite** — expect a silent fallback to the gate anchor, never `anchorUs = 0`. Test in Task 3.
3. **History full and wrapped before adoption** (a long search) — recovery must only use frames still held and never read stale slots. Test in Task 9.
4. **Merged points exceeding the core's 32-point ring** — recovery caps at the given capacity and keeps the newest hypothesis point. Test in Task 9.
5. **Label file present but malformed** for a capture — the evaluator falls back to the heuristic label and says so, instead of aborting the batch. Test in Task 1.

---

## File Structure

| File | Responsibility |
|---|---|
| `firmware/iwr6843/l3_ball_anchor.{h,c}` (new) | Impact anchor: tee bin, accept bin, gate time, club-fit time and tolerance |
| `firmware/iwr6843/l3_ball_hyp.{h,c}` | Forward search: metric cfg, corridor, stalled drop, time coast, decel reject, score |
| `firmware/iwr6843/l3_ball_recover.{h,c}` (new) | Post-impact target history ring and the backward pass |
| `firmware/iwr6843/l3_ball_track.{h,c}` | Arms with an anchor, owns history, merges recovery at adoption, extract-snr helper |
| `firmware/iwr6843/l3_dump.c` | Arm call builds the anchor; extraction snr helper |
| `firmware/iwr6843/makefile` | New sources; `L3_BALL_RECOVER=0` on the board |
| `src/openflight/iwr6843/firmware_host.py` | Sources list, ctypes mirrors, bindings |
| `src/openflight/iwr6843/firmware_replay.py` | Arm calls, extract snr, `BallTuning`, `recovered_frames` |
| `src/openflight/iwr6843/tunables.py` | Renamed hypothesis tunables |
| `scripts/analysis/evaluate_iwr_tracking.py` | Gap-tolerant label, labels override, split summary/acceptance, net diagnostic, CLI flags |
| `tests/test_evaluate_iwr_tracking.py`, `tests/test_iwr6843_firmware_ball_anchor.py` (new), `tests/test_iwr6843_firmware_ball_hyp.py`, `tests/test_iwr6843_firmware_ball_recover.py` (new), `tests/test_iwr6843_firmware_ball_track.py`, `tests/test_iwr6843_firmware_board_image.py` | Tests |

---

### Task 1: Gap-tolerant `ball_present`, labels override, split summary and acceptance (E1, E2)

**Files:**
- Modify: `scripts/analysis/evaluate_iwr_tracking.py` (constants near line 55, `Outcome`, `ball_present` at ~104, `evaluate` at ~180, `summarize`, `compare`, `main`)
- Test: `tests/test_evaluate_iwr_tracking.py`

**Interfaces:**
- Produces: `ball_present(frames, ops_mps, bin_m, *, max_gap_us: int | None = None, anchor: tuple[float, int, int] | None = None) -> bool`; `Outcome.ball_present_strict: bool`; `summarize(...)["ball_by_presence"] == {"present": {"ok","wrong","none"}, "absent": {...}}` and `["ball_present_strict"]`; `accept_split(summary: dict, baseline: dict) -> list[str]`; constants `GAP_MAX_US = 18_000`, `ANCHOR_TOL_US = 15_000`; CLI `--accept <baseline.json>`.

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_evaluate_iwr_tracking.py` (helpers `frame_of`, `ev`, `BIN_M` already exist there; 40 m/s at 3 ms = 2.56 bins/frame):

```python
STEP = 40.0 * 0.003 / BIN_M  # bins per 3 ms frame at 40 m/s


def chain(frames_ms, start_bin=50.0, speed=40.0):
    """One target per listed frame (3 ms apart) on a 40 m/s line from start_bin at 0 ms."""
    return [
        frame_of(k, k * 3000, [start_bin + speed * (k * 0.003) / BIN_M]) for k in frames_ms
    ]


def test_gap_tolerant_ball_present_bridges_an_impact_gap(ev):
    frames = chain([1, 5, 6])  # frames 2-4 missing: 9 ms gap
    assert ev.ball_present(frames, 40.0, BIN_M) is False  # strict: consecutive only
    assert ev.ball_present(frames, 40.0, BIN_M, max_gap_us=18_000) is True


def test_gap_tolerant_ball_present_refuses_a_gap_longer_than_allowed(ev):
    frames = chain([1, 9, 10])  # 24 ms gap
    assert ev.ball_present(frames, 40.0, BIN_M, max_gap_us=18_000) is False


def test_a_stationary_pair_is_not_a_ball_with_gaps_allowed(ev):
    frames = [frame_of(k, k * 3000, [50.0, 53.0]) for k in range(1, 8)]
    assert ev.ball_present(frames, 40.0, BIN_M, max_gap_us=18_000) is False


def test_the_anchor_requires_the_chain_to_back_project_to_the_tee(ev):
    frames = chain([4, 5, 6], start_bin=50.0)  # passes bin 50 at t = 0
    assert ev.ball_present(frames, 40.0, BIN_M, max_gap_us=18_000, anchor=(50.0, 0, 15_000))
    # The same chain judged against an impact 30 ms earlier does not back-project.
    assert not ev.ball_present(frames, 40.0, BIN_M, max_gap_us=18_000, anchor=(50.0, -30_000, 15_000))


def test_ball_present_with_gaps_and_no_targets(ev):
    assert ev.ball_present([], 40.0, BIN_M, max_gap_us=18_000) is False
    assert ev.ball_present([frame_of(1, 3000, [])], 40.0, BIN_M, max_gap_us=18_000) is False


def outcome(ev, ball, present, strict=None):
    return ev.Outcome(
        name="x", club="club", ball=ball, ball_present=present,
        ball_present_strict=present if strict is None else strict, launch_mps=None, ops_mps=40.0,
    )


def test_summarize_splits_the_ball_verdicts_by_presence(ev):
    outs = [
        outcome(ev, "ok", True), outcome(ev, "none", True),
        outcome(ev, "wrong", False, strict=False), outcome(ev, "none", False),
    ]
    s = ev.summarize(outs)
    assert s["ball_by_presence"] == {
        "present": {"ok": 1, "wrong": 0, "none": 1},
        "absent": {"ok": 0, "wrong": 1, "none": 1},
    }
    assert s["ball_present"] == 2 and s["ball_present_strict"] == 2


def split(present_ok, absent_none, absent_wrong, club=55):
    return {
        "captures": 93,
        "club": {"club": club, "stuck": 0, "few": 0},
        "ball_by_presence": {
            "present": {"ok": present_ok, "wrong": 0, "none": 0},
            "absent": {"ok": 0, "wrong": absent_wrong, "none": absent_none},
        },
    }


def test_accept_split_passes_only_a_strict_improvement(ev):
    base = split(present_ok=10, absent_none=5, absent_wrong=60)
    assert ev.accept_split(split(11, 6, 59), base) == []
    problems = ev.accept_split(split(10, 5, 60, club=54), base)
    assert len(problems) == 4  # ok not higher, none not higher, wrong not lower, club below 55


def test_a_reviewed_label_overrides_the_heuristic(ev, tmp_path, monkeypatch):
    labels = SimpleNamespace(reviewed=True, ball=(1, 2, 3))
    monkeypatch.setattr(ev, "load_labels", lambda _path: labels)
    assert ev.labelled_presence(tmp_path / "x.l3dump") is True
    labels.ball = ()
    assert ev.labelled_presence(tmp_path / "x.l3dump") is False


def test_an_unreviewed_or_broken_label_falls_back_to_the_heuristic(ev, tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(ev, "load_labels", lambda _path: SimpleNamespace(reviewed=False, ball=(1, 2, 3)))
    assert ev.labelled_presence(tmp_path / "x.l3dump") is None

    def broken(_path):
        raise ev.LabelError("bad file")

    monkeypatch.setattr(ev, "load_labels", broken)
    assert ev.labelled_presence(tmp_path / "x.l3dump") is None
    assert "bad file" in capsys.readouterr().err
```

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_evaluate_iwr_tracking.py -v`
Expected: the new tests FAIL (`unexpected keyword argument 'max_gap_us'`, no `accept_split`, no `ball_present_strict`); the old tests pass.

- [ ] **Step 3: Implement**

In `scripts/analysis/evaluate_iwr_tracking.py`:

Imports: add `import sys` and `from openflight.iwr6843.labels import LabelError, load_labels`.

Constants after `CHAIN_MIN_POINTS = 3`:

```python
# The label's own gap and back-projection allowances (spec E1): the tracker's
# impact-region coast and the gate anchor's tolerance.
GAP_MAX_US = 18_000
ANCHOR_TOL_US = 15_000
```

`Outcome`: add `ball_present_strict: bool` directly after `ball_present: bool`.

Replace `ball_present`:

```python
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
```

In `evaluate`, replace the `ball_present=...` argument with the two labels:

```python
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
```

and pass `ball_present=present, ball_present_strict=ball_present(post, case.ops_mps, bin_width_m()),` to `Outcome`.

Replace `summarize`'s return with:

```python
    verdicts = ("ok", "wrong", "none")

    def by(present: bool) -> dict[str, int]:
        return {
            v: sum(1 for o in outcomes if o.ball_present == present and o.ball == v)
            for v in verdicts
        }

    return {
        "captures": len(outcomes),
        "club": count("club", ("club", "stuck", "few")),
        "ball": count("ball", verdicts),
        "ball_present": sum(1 for o in outcomes if o.ball_present),
        "ball_present_strict": sum(1 for o in outcomes if o.ball_present_strict),
        "ball_by_presence": {"present": by(True), "absent": by(False)},
    }
```

Add after `compare`:

```python
CLUB_AT_IMPACT_FLOOR = 55


def accept_split(summary: dict, baseline: dict) -> list[str]:
    """Spec E2, against legacy re-run under the same label: what keeps the
    hypotheses from becoming the default, one line each (empty: accepted)."""
    now, base = summary["ball_by_presence"], baseline["ball_by_presence"]
    problems = []
    if now["present"]["ok"] <= base["present"]["ok"]:
        problems.append(f"ball-present ok {now['present']['ok']} not above {base['present']['ok']}")
    if now["absent"]["none"] <= base["absent"]["none"]:
        problems.append(f"ball-absent none {now['absent']['none']} not above {base['absent']['none']}")
    if now["absent"]["wrong"] >= base["absent"]["wrong"]:
        problems.append(f"ball-absent wrong {now['absent']['wrong']} not below {base['absent']['wrong']}")
    if summary["club"]["club"] < CLUB_AT_IMPACT_FLOOR:
        problems.append(f"club at impact {summary['club']['club']} < {CLUB_AT_IMPACT_FLOOR}")
    return problems
```

In `main`, add `parser.add_argument("--accept", type=Path, help="Judge against a legacy baseline by spec E2")` and after the `--compare` block:

```python
    if args.accept is not None:
        baseline = json.loads(args.accept.read_text(encoding="utf-8"))["summary"]
        problems = accept_split(summary, baseline)
        for line in problems:
            print(f"NOT ACCEPTED: {line}")
        if problems:
            return 1
```

Fix the existing tests that construct `Outcome(...)` without `ball_present_strict` (grep `Outcome(` in the test file) by adding `ball_present_strict=` with the same value as `ball_present`.

- [ ] **Step 4: Run the tests**

Run: `uv run pytest tests/test_evaluate_iwr_tracking.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add scripts/analysis/evaluate_iwr_tracking.py tests/test_evaluate_iwr_tracking.py
git commit -m "feat(eval): gap-tolerant ball_present, labels override, split acceptance

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 2: Re-run the legacy baseline under the new label

**Files:**
- Create: `docs/superpowers/specs/2026-10-01-ball-search-baseline-legacy.json`
- Modify: `docs/superpowers/specs/2026-10-01-iwr-gap-tolerant-ball-search-design.md` (Results)

**Interfaces:**
- Consumes: Task 1's evaluator.
- Produces: the legacy baseline file every later comparison uses.

- [ ] **Step 1: Ask the user for the sessions folder** (the 93 captures; the 2026-09-28 spec calls it `iwr-test-sessions`). Call it `<SESSIONS>` below.

- [ ] **Step 2: Run legacy**

```bash
uv run python scripts/analysis/evaluate_iwr_tracking.py <SESSIONS> --ball-hypotheses off --json docs/superpowers/specs/2026-10-01-ball-search-baseline-legacy.json
```

Expected: 93 captures, `club.club` 55, `ball` 15/70/8 as in the 2026-09-28 baseline (the label does not change verdicts); `ball_present_strict` 23; `ball_present` ≥ 23.

- [ ] **Step 3: Record in the spec's Results section**: the strict vs gap-tolerant `ball_present` counts, the per-presence split, and the names of captures whose label changed (diff the two fields in the JSON outcomes).

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/specs/2026-10-01-ball-search-baseline-legacy.json docs/superpowers/specs/2026-10-01-iwr-gap-tolerant-ball-search-design.md
git commit -m "docs(iwr6843): legacy ball baseline under the gap-tolerant label

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 3: `l3_ball_anchor` — the impact anchor

**Files:**
- Create: `firmware/iwr6843/l3_ball_anchor.h`, `firmware/iwr6843/l3_ball_anchor.c`
- Modify: `firmware/iwr6843/makefile:94` (SOURCES), `src/openflight/iwr6843/firmware_host.py` (`HOST_SOURCES` ~line 30, mirrors, bindings ~line 1794)
- Test: `tests/test_iwr6843_firmware_ball_anchor.py` (new)

**Interfaces:**
- Consumes: `l3_impact_fit_track`, `l3_fit_span_point`, `l3_round_us` (`l3_impact_fit.h`).
- Produces (C):

```c
enum { L3_BALL_ANCHOR_GATE = 0, L3_BALL_ANCHOR_CLUB = 1 };
#define L3_BALL_ANCHOR_MIN_TOL_US 2000U
typedef struct {
    float    anchorBin;      /* the ball's range at impact (the tee), global sub-bin */
    float    acceptFromBin;  /* nothing short of this joins: the band's far edge, else the tee */
    uint32_t gateUs;         /* the gate / range-crossing time: the legacy impact time */
    uint32_t anchorUs;       /* the impact time the search back-projects to */
    uint32_t anchorTolUs;
    float    anchorSigmaUs;  /* the club fit's sigma; 0 from the gate */
    uint8_t  source;         /* L3_BALL_ANCHOR_* */
} l3_ball_anchor_t;
void l3_ball_anchor_make(float anchorBin, float acceptFromBin, uint32_t gateUs,
                         uint32_t gateTolUs, const l3_impact_fit_cfg_t *fitCfg,
                         const l3_club_track_t *club, float maxSigmaUs, l3_ball_anchor_t *out);
const char *l3_ball_anchor_source_name(uint8_t source);
uint32_t l3_ball_anchor_struct_bytes(void);
```
- Produces (Python): `fw.BallAnchor`, `fw.BALL_ANCHOR_SOURCE_NAMES = ("gate", "club")`, bindings for the three functions.

- [ ] **Step 1: Write the failing tests** — `tests/test_iwr6843_firmware_ball_anchor.py`:

```python
"""Tests for the IWR6843 impact anchor, firmware/iwr6843/l3_ball_anchor.c.

The ball search back-projects to where and when the ball was struck: the tee
bin, and the impact time from the club's approach when that fit is tight, else
the gate time with the gate's tolerance.
"""

from __future__ import annotations

import ctypes

import pytest
from iwr6843_twotrack import BIN_M, obs

from openflight.iwr6843 import firmware_host as fw

TEE = 46.0
GATE_US = 30_000


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def fit_cfg(lib):
    cfg = fw.ImpactFitCfg()
    lib.l3_impact_fit_cfg_defaults(ctypes.byref(cfg))
    return cfg


def approach(lib, mps=30.0, frames=8, frame_us=3000, impact_us=GATE_US, jitter=()):
    """A club track closing on the tee at mps, reaching it at impact_us."""
    cfg = fw.TrackCfg()
    lib.l3_track_cfg_defaults(ctypes.byref(cfg))
    track = fw.ClubTrack()
    lib.l3_track_init(ctypes.byref(track), ctypes.byref(cfg))
    for k in range(frames):
        ts = impact_us - (frames - k) * frame_us
        bin_ = TEE - mps * (impact_us - ts) * 1e-6 / BIN_M + (jitter[k] if k < len(jitter) else 0.0)
        target = obs(k + 1, ts, bin_, 9000.0, mps)
        lib.l3_track_update(ctypes.byref(track), ctypes.byref(target), 1, k + 1, ts)
    return track


def make(lib, club=None, max_sigma=3000.0, accept=TEE + 3.0):
    out = fw.BallAnchor()
    lib.l3_ball_anchor_make(
        TEE, accept, GATE_US, 15_000, ctypes.byref(fit_cfg(lib)),
        None if club is None else ctypes.byref(club), max_sigma, ctypes.byref(out),
    )
    return out


def test_the_struct_layout_matches_the_c(lib):
    assert ctypes.sizeof(fw.BallAnchor) == lib.l3_ball_anchor_struct_bytes()


def test_without_a_club_the_gate_anchors_impact(lib):
    a = make(lib)
    assert fw.BALL_ANCHOR_SOURCE_NAMES[a.source] == "gate"
    assert (a.anchorUs, a.gateUs, a.anchorTolUs) == (GATE_US, GATE_US, 15_000)
    assert (a.anchorBin, a.acceptFromBin) == (TEE, TEE + 3.0)


def test_a_clean_club_approach_anchors_impact(lib):
    club = approach(lib, impact_us=GATE_US - 4000)  # the gate fired 4 ms late
    assert club.count >= 4
    a = make(lib, club)
    assert fw.BALL_ANCHOR_SOURCE_NAMES[a.source] == "club"
    assert a.anchorUs == pytest.approx(GATE_US - 4000, abs=300)
    assert a.gateUs == GATE_US
    assert a.anchorTolUs == max(2000, round(3 * a.anchorSigmaUs))


def test_a_loose_club_fit_falls_back_to_the_gate(lib):
    club = approach(lib, frames=4, jitter=(0.0, 2.5, -2.5, 2.5))
    a = make(lib, club, max_sigma=50.0)
    assert fw.BALL_ANCHOR_SOURCE_NAMES[a.source] == "gate"
    assert a.anchorUs == GATE_US


def test_too_few_club_points_fall_back_to_the_gate(lib):
    a = make(lib, approach(lib, frames=1))
    assert fw.BALL_ANCHOR_SOURCE_NAMES[a.source] == "gate"
    assert a.anchorUs == GATE_US and a.anchorUs != 0


def test_a_zero_sigma_limit_never_uses_the_club(lib):
    a = make(lib, approach(lib), max_sigma=0.0)
    assert fw.BALL_ANCHOR_SOURCE_NAMES[a.source] == "gate"
```

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_anchor.py -v`
Expected: FAIL with `AttributeError: ... BallAnchor`.

- [ ] **Step 3: Implement**

`firmware/iwr6843/l3_ball_anchor.h`:

```c
/* IWR6843 impact anchor: where and when the ball was struck, for the ball
 * search to back-project to.
 *
 * Where: the tee bin. Points may only join from acceptFromBin on (the tee
 * band's far edge when one is placed), which is a different number. When:
 * the club's approach fitted to the tee range (l3_impact_fit_track, club in)
 * when that fit is tight, else the gate / range-crossing time with the gate's
 * tolerance. The legacy acquisition keeps gateUs. Pure C, no hardware.
 */
#ifndef L3_BALL_ANCHOR_H
#define L3_BALL_ANCHOR_H

#include <stdint.h>

#include "l3_club_track.h"
#include "l3_impact_fit.h"

enum { L3_BALL_ANCHOR_GATE = 0, L3_BALL_ANCHOR_CLUB = 1 };

/* The club fit's tolerance is 3 sigma, never under this. */
#define L3_BALL_ANCHOR_MIN_TOL_US 2000U

typedef struct {
    float    anchorBin;      /* the ball's range at impact (the tee), global sub-bin */
    float    acceptFromBin;  /* nothing short of this joins: the band's far edge, else the tee */
    uint32_t gateUs;         /* the gate / range-crossing time: the legacy impact time */
    uint32_t anchorUs;       /* the impact time the search back-projects to */
    uint32_t anchorTolUs;
    float    anchorSigmaUs;  /* the club fit's sigma; 0 from the gate */
    uint8_t  source;         /* L3_BALL_ANCHOR_* */
} l3_ball_anchor_t;

/* The gate anchor (anchorUs = gateUs, tolerance gateTolUs), replaced by the
 * club's approach when club holds points, maxSigmaUs > 0 and the club-in fit
 * against anchorBin x fitCfg->binWidthM is OK within maxSigmaUs. */
void l3_ball_anchor_make(float anchorBin, float acceptFromBin, uint32_t gateUs,
                         uint32_t gateTolUs, const l3_impact_fit_cfg_t *fitCfg,
                         const l3_club_track_t *club, float maxSigmaUs, l3_ball_anchor_t *out);
const char *l3_ball_anchor_source_name(uint8_t source);
uint32_t l3_ball_anchor_struct_bytes(void);

#endif /* L3_BALL_ANCHOR_H */
```

`firmware/iwr6843/l3_ball_anchor.c`:

```c
#include "l3_ball_anchor.h"

#include <string.h>

void l3_ball_anchor_make(float anchorBin, float acceptFromBin, uint32_t gateUs,
                         uint32_t gateTolUs, const l3_impact_fit_cfg_t *fitCfg,
                         const l3_club_track_t *club, float maxSigmaUs, l3_ball_anchor_t *out)
{
    l3_fit_span_t span;
    l3_fit_estimate_t est;
    uint32_t anchorUs;
    float tol;

    memset(out, 0, sizeof(*out));
    out->anchorBin = anchorBin;
    out->acceptFromBin = acceptFromBin;
    out->gateUs = gateUs;
    out->anchorUs = gateUs;
    out->anchorTolUs = gateTolUs;
    out->source = L3_BALL_ANCHOR_GATE;
    if (fitCfg == NULL || club == NULL || club->count == 0U || !(maxSigmaUs > 0.0F)) {
        return;
    }
    span.track = club;
    span.first = 0U;
    span.count = club->count;
    l3_impact_fit_track(fitCfg, L3_FIT_CLUB_IN, l3_fit_span_point, &span, club->count,
                        anchorBin * fitCfg->binWidthM, &est);
    if (est.why != L3_FIT_WHY_OK || !(est.sigmaUs <= maxSigmaUs)) {
        return;
    }
    anchorUs = l3_round_us(est.timeUs);
    if (anchorUs == 0U) {
        return;  /* not finite or not positive: keep the gate */
    }
    tol = 3.0F * est.sigmaUs;
    if (tol < (float)L3_BALL_ANCHOR_MIN_TOL_US) {
        tol = (float)L3_BALL_ANCHOR_MIN_TOL_US;
    }
    out->anchorUs = anchorUs;
    out->anchorTolUs = (uint32_t)(tol + 0.5F);
    out->anchorSigmaUs = est.sigmaUs;
    out->source = L3_BALL_ANCHOR_CLUB;
}

const char *l3_ball_anchor_source_name(uint8_t source)
{
    return (source == L3_BALL_ANCHOR_CLUB) ? "club" : "gate";
}

uint32_t l3_ball_anchor_struct_bytes(void)
{
    return (uint32_t)sizeof(l3_ball_anchor_t);
}
```

`makefile:94`: add `l3_ball_anchor.c` to `SOURCES` after `l3_impact_fit.c`. `firmware_host.py`: add `"l3_ball_anchor.c",` to `HOST_SOURCES` after `"l3_impact_fit.c"`; add near the hypothesis mirrors:

```python
BALL_ANCHOR_SOURCE_NAMES = ("gate", "club")


class BallAnchor(ctypes.Structure):
    """``l3_ball_anchor_t``: where and when the ball was struck."""

    _fields_ = [
        ("anchorBin", ctypes.c_float),
        ("acceptFromBin", ctypes.c_float),
        ("gateUs", ctypes.c_uint32),
        ("anchorUs", ctypes.c_uint32),
        ("anchorTolUs", ctypes.c_uint32),
        ("anchorSigmaUs", ctypes.c_float),
        ("source", ctypes.c_uint8),
    ]
```

and in the bindings table:

```python
    "l3_ball_anchor_make": (
        [_F32, _F32, _U32, _U32, _P(ImpactFitCfg), _P(ClubTrack), _F32, _P(BallAnchor)],
        None,
    ),
    "l3_ball_anchor_source_name": ([ctypes.c_uint8], ctypes.c_char_p),
    "l3_ball_anchor_struct_bytes": ([], _U32),
```

(Place `BallAnchor` after `ImpactFitCfg` and `ClubTrack` are defined; check the bindings table's exact helper names `_F32`, `_U32`, `_P` near line 1794.)

- [ ] **Step 4: Run the tests**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_anchor.py -v`
Expected: PASS. If `test_a_clean_club_approach_anchors_impact` fails on `club.count`, the club track's acquisition refused the synthetic approach: read `l3_track_cfg_defaults` and set the cfg field that blocks it in `approach()` (e.g. `minConfidence`), not the anchor code.

- [ ] **Step 5: Commit**

```bash
git add firmware/iwr6843/l3_ball_anchor.h firmware/iwr6843/l3_ball_anchor.c firmware/iwr6843/makefile src/openflight/iwr6843/firmware_host.py tests/test_iwr6843_firmware_ball_anchor.py
git commit -m "feat(iwr6843): impact anchor for the ball search, from the club when tight

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 4: Arm the hypotheses and the ball track with an anchor

**Files:**
- Modify: `firmware/iwr6843/l3_ball_hyp.{h,c}` (`l3_ball_hyps_t`, `l3_ball_hyps_arm`, `judge`, update's spawn/far-window), `firmware/iwr6843/l3_ball_track.{h,c}` (cfg, struct, `arm`), `firmware/iwr6843/l3_dump.c:3525-3549` (`l3_shotObserve`), `src/openflight/iwr6843/firmware_host.py`, `src/openflight/iwr6843/firmware_replay.py:1117,1377`
- Test: `tests/test_iwr6843_firmware_ball_hyp.py`, `tests/test_iwr6843_firmware_ball_track.py`, `tests/test_iwr6843_firmware_board_image.py`

**Interfaces:**
- Consumes: Task 3 `l3_ball_anchor_t`, `l3_ball_anchor_make`.
- Produces:
  - `void l3_ball_hyps_arm(l3_ball_hyps_t *hyps, const l3_ball_anchor_t *anchor);`
  - `void l3_ball_track_arm(l3_ball_track_t *track, const l3_ball_anchor_t *anchor, const l3_vec3_t *origin);`
  - `void l3_ball_track_anchor(const l3_ball_track_t *track, float teeBin, float acceptFromBin, uint32_t gateUs, const l3_impact_fit_cfg_t *fitCfg, const l3_club_track_t *club, l3_ball_anchor_t *out);`
  - `l3_ball_track_cfg_t` gains `uint32_t gateTolUs; float anchorMaxSigmaUs;` (after `skipClubClaim`, before `fit`); `l3_ball_track_t` gains `l3_ball_anchor_t anchor;` after `origin`.
  - `l3_ball_hyps_t` replaces `originBin`, `impactTimestampUs` with `l3_ball_anchor_t anchor;` (right after `armed`).
  - `l3_ball_hyps_cfg_t` loses `impactToleranceUs` (the tolerance comes with the anchor).
  - Python test helpers `arm(lib, hyps, origin_bin=46.0, gate_us=0, tol_us=15_000, accept_bin=None)` (hyp tests) and `Ball.arm(origin_bin, origin, impact_us)` keeping its signature (ball-track tests).

- [ ] **Step 1: Write the failing tests**

Ball-track test (`tests/test_iwr6843_firmware_ball_track.py`), add:

```python
def test_the_legacy_search_keeps_the_accept_bin_and_gate_time(lib):
    ball = Ball(lib)
    anchor = fw.BallAnchor(anchorBin=46.0, acceptFromBin=50.0, gateUs=99, anchorUs=77,
                           anchorTolUs=2000, anchorSigmaUs=100.0, source=1)
    lib.l3_ball_track_arm(ctypes.byref(ball.track), ctypes.byref(anchor), ctypes.byref(fw.Vec3(*ORIGIN)))
    assert ball.track.originBin == 50.0 and ball.track.impactTimestampUs == 99
    assert ball.track.anchor.anchorUs == 77
    assert ball.track.hyps.anchor.anchorBin == 46.0


def test_the_track_builds_its_anchor_from_its_own_cfg(lib):
    ball = Ball(lib)
    out = fw.BallAnchor()
    fit = fw.ImpactFitCfg()
    lib.l3_impact_fit_cfg_defaults(ctypes.byref(fit))
    lib.l3_ball_track_anchor(ctypes.byref(ball.track), 46.0, 50.0, 1234, ctypes.byref(fit), None, ctypes.byref(out))
    assert (out.anchorUs, out.anchorTolUs, out.acceptFromBin) == (1234, ball.track.cfg.gateTolUs, 50.0)
```

and change the existing test at line ~283 (`originBin == 50.0 and impactTimestampUs == 99`) only if it calls `l3_ball_track_arm` directly: route it through `Ball.arm`.

Hypothesis test: add

```python
def test_the_search_back_projects_to_the_tee_not_the_accept_bin(lib):
    """With a band the ball is accepted from the band's far edge, but it left the tee."""
    scene = TwoTracks(frames=8)
    hyps = make_hyps(lib)
    arm(lib, hyps, origin_bin=scene.origin_bin, gate_us=scene.gate_us, accept_bin=scene.origin_bin + 3.0)
    for f in scene.build():
        feed(lib, hyps, f.frame, f.timestamp_us, f.targets, f.club_index)
    v = verdict(lib, hyps)
    assert v.index >= 0
    assert abs(v.originOffsetUs) < 300.0
    assert min(bins(hyps.hyp[v.index])) >= scene.origin_bin + 3.0 - 1.0  # spawnBehind
```

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_track.py tests/test_iwr6843_firmware_ball_hyp.py -v`
Expected: FAIL (argument types / missing fields).

- [ ] **Step 3: Implement**

`l3_ball_hyp.h`: `#include "l3_ball_anchor.h"`; remove `impactToleranceUs` from `l3_ball_hyps_cfg_t`; in `l3_ball_hyps_t` replace `float originBin; uint32_t impactTimestampUs;` with `l3_ball_anchor_t anchor;  /* where and when the ball was struck; acceptFromBin is the old origin */`; change the arm prototype and its comment (`/* Forget every hypothesis and start looking from anchor->acceptFromBin, back-projecting to the anchor. */`).

`l3_ball_hyp.c`:
- defaults: delete `cfg->impactToleranceUs = 15000U;`.
- `l3_ball_hyps_arm(l3_ball_hyps_t *hyps, const l3_ball_anchor_t *anchor)`: `hyps->anchor = *anchor;` instead of the two fields.
- in `update`: every `hyps->originBin` → `hyps->anchor.acceptFromBin`; `hyps->impactTimestampUs` → `hyps->anchor.anchorUs`.
- in `judge`: fit from `hyps->anchor.anchorUs` (rename `atGate` → `atAnchor`), `originOffsetS = (hyps->anchor.anchorBin - atAnchor) / rate;`, tolerance `hyps->anchor.anchorTolUs`.

`l3_ball_track.h`: `#include "l3_ball_anchor.h"`; cfg fields after `skipClubClaim`:

```c
    /* The gate anchor's tolerance, and the club fit's sigma under which its
     * impact time anchors the search instead (0: never). */
    uint32_t gateTolUs;
    float    anchorMaxSigmaUs;
```

struct field after `origin`: `l3_ball_anchor_t anchor;   /* the search's; originBin and impactTimestampUs stay the legacy ones */`. New prototypes:

```c
/* IMPACT: start looking for a ball. The legacy acquisition starts from
 * anchor->acceptFromBin at anchor->gateUs; the hypotheses back-project to the
 * anchor. origin is the tee in the golf frame. */
void l3_ball_track_arm(l3_ball_track_t *track, const l3_ball_anchor_t *anchor,
                       const l3_vec3_t *origin);
/* The anchor for arming, from this track's gateTolUs and anchorMaxSigmaUs. */
void l3_ball_track_anchor(const l3_ball_track_t *track, float teeBin, float acceptFromBin,
                          uint32_t gateUs, const l3_impact_fit_cfg_t *fitCfg,
                          const l3_club_track_t *club, l3_ball_anchor_t *out);
```

`l3_ball_track.c`: defaults `cfg->gateTolUs = 15000U; cfg->anchorMaxSigmaUs = 3000.0F;`; reset `memset(&track->anchor, 0, sizeof(track->anchor));`;

```c
void l3_ball_track_arm(l3_ball_track_t *track, const l3_ball_anchor_t *anchor,
                       const l3_vec3_t *origin)
{
    l3_ball_track_reset(track);
    track->armed = 1U;
    track->originBin = anchor->acceptFromBin;
    track->origin = *origin;
    track->impactTimestampUs = anchor->gateUs;
    track->anchor = *anchor;
#if L3_BALL_HYPOTHESES
    l3_ball_hyps_arm(&track->hyps, anchor);
#endif
}

void l3_ball_track_anchor(const l3_ball_track_t *track, float teeBin, float acceptFromBin,
                          uint32_t gateUs, const l3_impact_fit_cfg_t *fitCfg,
                          const l3_club_track_t *club, l3_ball_anchor_t *out)
{
    l3_ball_anchor_make(teeBin, acceptFromBin, gateUs, track->cfg.gateTolUs, fitCfg, club,
                        track->cfg.anchorMaxSigmaUs, out);
}
```

`l3_dump.c` `l3_shotObserve` (line ~3541) replaces the arm call:

```c
        l3_ball_anchor_t anchor;

        l3_ball_track_anchor(&gBallTrack, (float)teeBin, l3_ballArmBin(teeBin),
                             in.impactTimestampUs, &gImpactFitCfg, &gClubTrack, &anchor);
        l3_ball_track_arm(&gBallTrack, &anchor, &gBallPosition);
```

(declare `anchor` at the top of the block per the file's C style).

`firmware_host.py`: `BallHypsCfg` drop `impactToleranceUs`; `BallHyps` fields become `("cfg", BallHypsCfg), ("armed", ctypes.c_uint8), ("anchor", BallAnchor), ("nextId", …), …`; `BallTrackCfg` add `("gateTolUs", ctypes.c_uint32), ("anchorMaxSigmaUs", ctypes.c_float)` after `skipClubClaim`; `BallTrack` add `("anchor", BallAnchor)` after `origin`; bindings `"l3_ball_hyps_arm": ([_P(BallHyps), _P(BallAnchor)], None)`, `"l3_ball_track_arm": ([_P(BallTrack), _P(BallAnchor), _P(Vec3)], None)`, `"l3_ball_track_anchor": ([_P(BallTrack), _F32, _F32, _U32, _P(ImpactFitCfg), _P(ClubTrack), _P(BallAnchor)], None)`. Move `BallAnchor` above `BallHyps` if needed.

`firmware_replay.py`: add a helper next to `_ball_arm_bin` and use it at both arm sites (lines ~1117 and ~1377; `fit_cfg` is the `ImpactFitCfg` built at ~965, `track` the club track):

```python
def _arm_ball(lib, ball_track, fit_cfg, track, band, destination, ball_position, gate_us) -> None:
    """l3_shotObserve's arm: the anchor from the tee and the club, then the track."""
    anchor = fw.BallAnchor()
    lib.l3_ball_track_anchor(
        ctypes.byref(ball_track),
        float(destination),
        _ball_arm_bin(band, destination),
        int(gate_us),
        ctypes.byref(fit_cfg),
        ctypes.byref(track),
        ctypes.byref(anchor),
    )
    lib.l3_ball_track_arm(ctypes.byref(ball_track), ctypes.byref(anchor), ctypes.byref(ball_position))
```

Test helpers: in `tests/test_iwr6843_firmware_ball_hyp.py`

```python
def arm(lib, hyps, origin_bin=46.0, gate_us=0, tol_us=15_000, accept_bin=None):
    anchor = fw.BallAnchor(
        anchorBin=origin_bin,
        acceptFromBin=origin_bin if accept_bin is None else accept_bin,
        gateUs=gate_us, anchorUs=gate_us, anchorTolUs=tol_us,
    )
    lib.l3_ball_hyps_arm(ctypes.byref(hyps), ctypes.byref(anchor))
```

and drop `impactToleranceUs` from `test_defaults`. In `tests/test_iwr6843_firmware_ball_track.py` `Ball.arm`:

```python
    def arm(self, origin_bin=ORIGIN_BIN, origin=ORIGIN, impact_us=IMPACT_US):
        anchor = fw.BallAnchor(anchorBin=origin_bin, acceptFromBin=origin_bin, gateUs=impact_us,
                               anchorUs=impact_us, anchorTolUs=self.track.cfg.gateTolUs)
        self.lib.l3_ball_track_arm(ctypes.byref(self.track), ctypes.byref(anchor), ctypes.byref(fw.Vec3(*origin)))
```

`tests/test_iwr6843_firmware_board_image.py` `OpaqueBallTrack.run` arms the same way (build a `BallAnchor` from `scene.origin_bin`, `scene.gate_us`, tolerance 15000). Grep `l3_ball_track_arm\|l3_ball_hyps_arm` across `tests/ src/ scripts/` and update every remaining caller.

- [ ] **Step 4: Run the tests and the recordings**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_track.py tests/test_iwr6843_firmware_ball_hyp.py tests/test_iwr6843_firmware_ball_anchor.py tests/test_iwr6843_firmware_board_image.py tests/test_iwr6843_firmware_replay.py -v`
Expected: all PASS — the legacy replay results and recording manifests are unchanged.

- [ ] **Step 5: Commit**

```bash
git add firmware/iwr6843 src/openflight/iwr6843 tests
git commit -m "feat(iwr6843): arm the ball search with an impact anchor apart from the accept bin

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 5: Metric hypothesis configuration, point coherence, shared points fit (G5)

**Files:**
- Modify: `firmware/iwr6843/l3_ball_hyp.{h,c}`, `src/openflight/iwr6843/firmware_host.py`, `src/openflight/iwr6843/firmware_replay.py` (`BallTuning`), `src/openflight/iwr6843/tunables.py:66-74`, `scripts/analysis/evaluate_iwr_tracking.py` (`--far-window-bins` → `--far-window-m`)
- Test: `tests/test_iwr6843_firmware_ball_hyp.py`, `tests/test_evaluate_iwr_tracking.py`

**Interfaces:**
- Produces:
  - `l3_ball_hyps_cfg_t` metric fields `spawnBehindM`, `spawnBeyondM`, `gateM`, `farWindowM` (replacing the `*Bins` ones; `maxMisses` stays until Task 7).
  - `l3_ball_hyps_t` derived bins (set by `l3_ball_hyps_init`): `float spawnBehindBins, spawnBeyondBins, gateBins, farWindowBins;` placed after `anchor`.
  - `l3_ball_hyp_point_t.coherence` (after `clubStat`).
  - `int32_t l3_ball_points_fit(const l3_ball_hyp_point_t *points, uint32_t count, uint32_t referenceUs, float *rateBinsPerS, float *binAtReference, float *residualBins);` — `l3_ball_hyp_fit` delegates to it.
  - `BallTuning.far_window_m` (replaces `far_window_bins`).

- [ ] **Step 1: Write the failing tests** (hyp test file)

```python
def test_metric_settings_become_bins_at_init(lib):
    hyps = make_hyps(lib, gateM=3 * BIN_M, spawnBehindM=2 * BIN_M, farWindowM=4 * BIN_M,
                     spawnBeyondM=8 * BIN_M)
    assert (hyps.gateBins, hyps.spawnBehindBins, hyps.farWindowBins, hyps.spawnBeyondBins) == pytest.approx(
        (3.0, 2.0, 4.0, 8.0)
    )


def test_a_point_keeps_its_targets_coherence(lib):
    hyps = make_hyps(lib)
    arm(lib, hyps)
    t = obs(1, 0, 47.0, 900.0, 5.0)
    t.coherence = 0.42
    feed(lib, hyps, 1, 0, [t])
    (hyp,) = active(hyps)
    assert hyp.points[0].coherence == pytest.approx(0.42)
```

Update `test_defaults` to the metric names and values (`spawnBehindM 0.046875`, `spawnBeyondM 0.46875`, `gateM 0.0703125`, `farWindowM 0`), and `test_the_far_window_keeps_near_returns_out_of_the_search` to set `farWindowM=<old bins> * BIN_M`.

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py -v`
Expected: FAIL (unknown fields).

- [ ] **Step 3: Implement**

`l3_ball_hyp.h` cfg: rename `spawnBehindBins → spawnBehindM`, `spawnBeyondBins → spawnBeyondM` (comment: "... used only with corridorGate off"), `gateBins → gateM`, `farWindowBins → farWindowM`, comments in metres. Point: add `float coherence;  /* the target's lag-1 coherence */` after `clubStat`. Hyps struct: after `anchor`:

```c
    /* cfg's metric settings in bins, from binWidthM at init */
    float    spawnBehindBins;
    float    spawnBeyondBins;
    float    gateBins;
    float    farWindowBins;
```

Prototype `l3_ball_points_fit` with a comment: `/* l3_ball_hyp_fit over any time-ordered point array. */`.

`l3_ball_hyp.c`: defaults `cfg->spawnBehindM = cfg->binWidthM; cfg->spawnBeyondM = 10.0F * cfg->binWidthM; cfg->gateM = 1.5F * cfg->binWidthM; cfg->farWindowM = 0.0F;` (binWidthM is set first in the defaults). `l3_ball_hyps_init` after copying cfg:

```c
    float binsPerM = (cfg->binWidthM > 0.0F) ? 1.0F / cfg->binWidthM : 0.0F;

    hyps->spawnBehindBins = cfg->spawnBehindM * binsPerM;
    hyps->spawnBeyondBins = cfg->spawnBeyondM * binsPerM;
    hyps->gateBins = cfg->gateM * binsPerM;
    hyps->farWindowBins = cfg->farWindowM * binsPerM;
```

Replace every `cfg->spawnBehindBins`, `cfg->spawnBeyondBins`, `cfg->gateBins`, `cfg->farWindowBins` in `update` and `window` with the `hyps->…Bins` fields (pass `hyps` to `l3_ball_hyps_window` instead of `cfg`). Rename the body of `l3_ball_hyp_fit` to `l3_ball_points_fit(points, count, …)` (loop over `points[i]`, `count`) and make `l3_ball_hyp_fit` `return l3_ball_points_fit(hyp->points, hyp->count, referenceUs, rateBinsPerS, binAtReference, residualBins);`. `l3_ball_hyps_append` sets `point->coherence = target->coherence;`.

Note: `l3_ball_track_init` copies `binWidthM` into `cfg.hyps` before `l3_ball_hyps_init` — the metric→bins conversion therefore uses the track's bin width. Keep that order.

`firmware_host.py`: mirror the renamed cfg fields, `BallHypPoint` gains `("coherence", ctypes.c_float)` after `clubStat`, `BallHyps` gains the four floats after `anchor`, binding `"l3_ball_points_fit": ([_P(BallHypPoint), _U32, _U32, _P(_F32), _P(_F32), _P(_F32)], ctypes.c_int32)` (match the existing `l3_ball_hyp_fit` binding's pointer style).

`firmware_replay.py` `BallTuning`: rename `far_window_bins` → `far_window_m`, `apply`: `cfg.hyps.farWindowM = self.far_window_m`. `tunables.py:66-70`: `hyps.spawnBehindM` (0.0, 0.2, 0.025), `hyps.spawnBeyondM` (0.2, 0.75, 0.05), `hyps.gateM` (0.025, 0.15, 0.025). Evaluator: `--far-window-m` (help "Hypothesis points only this many metres beyond the accept bin"), `parse_tuning` passes `far_window_m=args.far_window_m`; update the CLI pass-through test accordingly.

- [ ] **Step 4: Run the tests**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py tests/test_iwr6843_firmware_ball_track.py tests/test_evaluate_iwr_tracking.py tests/test_iwr6843_constants_fit.py tests/test_iwr6843_firmware_replay.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add firmware/iwr6843 src/openflight/iwr6843 scripts/analysis/evaluate_iwr_tracking.py tests
git commit -m "refactor(iwr6843): ball hypothesis settings in metres, point coherence, shared fit

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 6: Corridor gate (G1) and stalled-hypothesis drop (G2)

**Files:**
- Modify: `firmware/iwr6843/l3_ball_hyp.{h,c}`, `src/openflight/iwr6843/firmware_host.py`
- Test: `tests/test_iwr6843_firmware_ball_hyp.py`

**Interfaces:**
- Produces: cfg `uint32_t corridorGate; float anchorRangeTolM;` (append after `farWindowM`); derived nothing.

- [ ] **Step 1: Write the failing tests**

```python
def test_a_stationary_pair_cannot_start_once_the_corridor_has_moved_on(lib):
    """20260927_144220: two near-stationary returns held two of four slots."""
    hyps = make_hyps(lib)
    arm(lib, hyps, tol_us=2000)
    for k in range(1, 4):  # 33-39 ms after the anchor: lower edge >= 0.21 m (4.5 bins)
        ts = 30_000 + 3000 * k
        feed(lib, hyps, k, ts, [obs(k, ts, 46.2, 2000.0, 0.0), obs(k, ts, 48.0, 2000.0, 0.0)])
    assert not active(hyps)


def test_the_same_pair_starts_hypotheses_with_the_corridor_off(lib):
    hyps = make_hyps(lib, corridorGate=0)
    arm(lib, hyps, tol_us=2000)
    feed(lib, hyps, 1, 33_000, [obs(1, 33_000, 46.2, 2000.0, 0.0), obs(1, 33_000, 48.0, 2000.0, 0.0)])
    assert len(active(hyps)) == 2


@pytest.mark.parametrize(
    "metres, inside",
    [(10.0 * 0.010 - 0.1 + 0.002, True), (10.0 * 0.010 - 0.1 - 0.002, False),
     (100.0 * 0.014 + 0.1 - 0.002, True), (100.0 * 0.014 + 0.1 + 0.002, False)],
)
def test_the_corridor_edges(lib, metres, inside):
    """dt = 12 ms, tol = 2 ms: [10 x 10 ms - 0.1, 100 x 14 ms + 0.1] metres from the tee."""
    hyps = make_hyps(lib, spawnBeyondM=10.0)  # the off-mode band must not be what refuses
    arm(lib, hyps, tol_us=2000)
    feed(lib, hyps, 1, 12_000, [obs(1, 12_000, 46.0 + metres / BIN_M, 900.0, 40.0)])
    assert bool(active(hyps)) is inside


def test_the_corridor_holds_across_the_clock_wrap(lib):
    start = 2**32 - 4000
    hyps = make_hyps(lib)
    arm(lib, hyps, gate_us=start, tol_us=2000)
    ts = (start + 12_000) % 2**32
    feed(lib, hyps, 1, ts, [obs(1, ts, 46.0 + 0.4 / BIN_M, 900.0, 40.0)])
    assert len(active(hyps)) == 1


def test_a_hypothesis_that_stops_moving_is_dropped_at_three_points(lib):
    hyps = make_hyps(lib, corridorGate=0)
    arm(lib, hyps)
    for k, b in enumerate((47.0, 47.1, 47.15), start=1):
        feed(lib, hyps, k, 2000 * k, [obs(k, 2000 * k, b, 900.0, 0.0)])
    assert not active(hyps)
    assert hyps.dropped == 1
```

Revise `test_a_stationary_return_near_the_origin_is_never_the_ball`: it now asserts `verdict(...).index == -1` and drops the `assert active(hyps)` line (the stall is dropped, not followed). Run `test_only_targets_near_the_origin_start_a_hypothesis` and `test_the_start_band_moves_out_with_the_time_since_the_gate` with `corridorGate=0` (they describe the off-mode band).

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py -v`
Expected: the new tests FAIL.

- [ ] **Step 3: Implement**

`l3_ball_hyp.h` cfg, after `farWindowM`:

```c
    /* G1: a target must be explainable by an impact within the anchor's
     * tolerance and a speed in [minDepartureMps, maxSpeedMps], within
     * anchorRangeTolM. 0: the start band [accept - spawnBehind, accept +
     * spawnBeyond + maxSpeedMps x elapsed] as before. */
    uint32_t corridorGate;
    float    anchorRangeTolM;
```

`l3_ball_hyp.c`: defaults `cfg->corridorGate = 1U; cfg->anchorRangeTolM = 0.1F;`. Add:

```c
/* G1: some impact within the anchor's tolerance and some speed in
 * [minDepartureMps, maxSpeedMps] put a ball at rangeBin at timestampUs. */
static int32_t l3_ball_hyps_inCorridor(const l3_ball_hyps_t *hyps, float rangeBin,
                                       uint32_t timestampUs)
{
    const l3_ball_hyps_cfg_t *cfg = &hyps->cfg;
    float dtS = l3_ball_hyps_seconds(timestampUs, hyps->anchor.anchorUs);
    float tolS = (float)hyps->anchor.anchorTolUs * 1.0e-6F;
    float travelledM = (rangeBin - hyps->anchor.anchorBin) * cfg->binWidthM;
    float loM = cfg->minDepartureMps * (dtS - tolS) - cfg->anchorRangeTolM;
    float hiM = cfg->maxSpeedMps * (dtS + tolS) + cfg->anchorRangeTolM;

    return (travelledM >= loM && travelledM <= hiM) ? 1 : 0;
}

/* G2: a hypothesis whose fitted rate after three points is under the slowest
 * ball's is a still return, not a departure. */
static int32_t l3_ball_hyps_stalled(const l3_ball_hyps_cfg_t *cfg, const l3_ball_hyp_t *hyp)
{
    float rate;
    float at;
    float residual;

    if (hyp->count < 3U ||
        !l3_ball_hyp_fit(hyp, hyp->points[hyp->count - 1U].timestampUs, &rate, &at, &residual)) {
        return 0;
    }
    return (rate * cfg->binWidthM < cfg->minDepartureMps) ? 1 : 0;
}
```

In `l3_ball_hyps_update`: after the far-window block, mark corridor misses as taken for everyone (so neither assignment nor spawning uses them):

```c
    if (cfg->corridorGate) {
        for (j = 0U; j < n; j++) {
            if (!taken[j] && !l3_ball_hyps_inCorridor(hyps, targets[j].rangeBin, timestampUs)) {
                taken[j] = 1U;  /* no impact and speed explain it: never a ball point */
            }
        }
    }
```

After the coasting loop, drop stalled hypotheses:

```c
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        l3_ball_hyp_t *hyp = &hyps->hyp[i];

        if (hyp->active && l3_ball_hyps_stalled(cfg, hyp)) {
            hyp->active = 0U;
            hyps->dropped++;
        }
    }
```

Spawn: the far edge applies only with the gate off:

```c
        if (taken[j] || range < hyps->anchor.acceptFromBin - hyps->spawnBehindBins ||
            (!cfg->corridorGate && range > spawnHi)) {
            continue;
        }
```

`firmware_host.py`: append `("corridorGate", ctypes.c_uint32), ("anchorRangeTolM", ctypes.c_float)` to `BallHypsCfg`.

- [ ] **Step 4: Run the tests**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py tests/test_iwr6843_firmware_ball_track.py tests/test_iwr6843_firmware_board_image.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add firmware/iwr6843/l3_ball_hyp.h firmware/iwr6843/l3_ball_hyp.c src/openflight/iwr6843/firmware_host.py tests/test_iwr6843_firmware_ball_hyp.py
git commit -m "feat(iwr6843): ball hypotheses search a range-time corridor and drop stalled tracks

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 7: Time-based coasting with an impact region (G3)

**Files:**
- Modify: `firmware/iwr6843/l3_ball_hyp.{h,c}`, `src/openflight/iwr6843/firmware_host.py`, `src/openflight/iwr6843/tunables.py:70`
- Test: `tests/test_iwr6843_firmware_ball_hyp.py`

**Interfaces:**
- Produces: cfg `coastUs`, `impactCoastUs`, `impactRegionM` (replacing `maxMisses`, in its place); hyps derived `float impactRegionBins;` after `farWindowBins`.

- [ ] **Step 1: Write the failing tests**

```python
@pytest.mark.parametrize("frame_us", [2000, 3000])
def test_a_ball_missing_near_impact_survives_a_long_gap(lib, frame_us):
    """Five frames without the ball inside the impact region, at either profile."""
    scene = TwoTracks(frames=10, frame_us=frame_us, missing_ball=(3, 4, 5, 6, 7),
                      club_visible=False)
    hyps, frames = run(lib, scene)
    assert len(active(hyps)) == 1
    assert bins(active(hyps)[0]) == truth(frames)


def test_the_same_gap_beyond_the_impact_region_drops_it(lib):
    scene = TwoTracks(frames=10, frame_us=3000, missing_ball=(5, 6, 7, 8), club_visible=False)
    hyps, _ = run(lib, scene, impactRegionM=0.05)
    assert hyps.dropped >= 1


def test_coasting_is_by_time_not_frames(lib):
    """6 ms coast: three missing 2 ms frames (6 ms) keep it; three missing 3 ms
    frames (9 ms) drop it. A frame count would treat them the same."""
    kept = TwoTracks(frames=7, frame_us=2000, missing_ball=(3, 4, 5), club_visible=False)
    hyps, _ = run(lib, kept, impactRegionM=0.0)
    assert len(active(hyps)) == 1 and hyps.dropped == 0
    lost = TwoTracks(frames=7, frame_us=3000, missing_ball=(3, 4, 5), club_visible=False)
    hyps, _ = run(lib, lost, impactRegionM=0.0)
    assert hyps.dropped >= 1
```

Update `test_defaults` (`coastUs 6000`, `impactCoastUs 18000`, `impactRegionM 0.5`), and the two existing coast tests (`…coasts_over_two_missing_frames…`, `three_missing_frames_drop_it…`): run them with `impactRegionM=0.0` and `coastUs=2 * scene.frame_us` (their 2 ms scene) so they still describe the far-field coast.

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py -v`
Expected: FAIL.

- [ ] **Step 3: Implement**

`l3_ball_hyp.h` cfg: replace `uint32_t maxMisses;` with

```c
    uint32_t coastUs;             /* longest a hypothesis goes without a point ... */
    uint32_t impactCoastUs;       /* ... unless its newest point is short of the tee + */
    float    impactRegionM;       /* this: the ball is hidden near impact for longer */
```

hyps: `float impactRegionBins;` after `farWindowBins`. `l3_ball_hyp.c` defaults: `cfg->coastUs = 6000U; cfg->impactCoastUs = 18000U; cfg->impactRegionM = 0.5F;` (drop `maxMisses`); init: `hyps->impactRegionBins = cfg->impactRegionM * binsPerM;`. Coast loop:

```c
    /* Coast the hypotheses that found nothing; drop one whose newest point is
     * older than its coast: impactCoastUs inside the impact region, else coastUs. */
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        l3_ball_hyp_t *hyp = &hyps->hyp[i];
        const l3_ball_hyp_point_t *last;
        uint32_t limitUs;

        if (!hyp->active || fed[i]) {
            continue;
        }
        hyp->misses++;
        last = &hyp->points[hyp->count - 1U];
        limitUs = (last->rangeBin < hyps->anchor.anchorBin + hyps->impactRegionBins)
                      ? cfg->impactCoastUs
                      : cfg->coastUs;
        if ((int32_t)(timestampUs - last->timestampUs) > (int32_t)limitUs) {
            hyp->active = 0U;
            hyps->dropped++;
        }
    }
```

Update the `fastPending` comment (`classifyPoints + maxMisses` → "its coast"). `firmware_host.py`: `BallHypsCfg` replace `("maxMisses", c_uint32)` with the three fields in the same place; `BallHyps` add `("impactRegionBins", c_float)`. `tunables.py:70`: `_t("ball", "hyps.coastUs", "int", 0, 12000, 3000)` and add `_t("ball", "hyps.impactCoastUs", "int", 6000, 30000, 3000)`.

- [ ] **Step 4: Run the tests**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py tests/test_iwr6843_firmware_ball_track.py tests/test_iwr6843_constants_fit.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add firmware/iwr6843/l3_ball_hyp.h firmware/iwr6843/l3_ball_hyp.c src/openflight/iwr6843 tests/test_iwr6843_firmware_ball_hyp.py
git commit -m "feat(iwr6843): ball hypotheses coast by time, longer near impact

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 8: Deceleration reject (G4) and the back-projection-first score (A3)

**Files:**
- Modify: `firmware/iwr6843/l3_ball_hyp.{h,c}` (`judge`), `src/openflight/iwr6843/firmware_host.py`
- Test: `tests/test_iwr6843_firmware_ball_hyp.py`

**Interfaces:**
- Produces: cfg (appended) `float maxDecelMps2; float rangeNoiseM; float wBack, wVel, wResid, wDoppler, wCoherence, wWeaker;`; verdict (appended, before `waitingForFast` stays where it is — append after it) `float velocityConsistency; float coherence; uint8_t anchorSource; uint32_t recovered; uint32_t recoveredFirstFrame; uint32_t recoveredMask;` (the last three are written by Task 10).

- [ ] **Step 1: Write the failing tests**

```python
def mix(lib, fast=35.0, slow=20.0, points=6, frame_us=3000, **overrides):
    """One line that leaves at `fast` and continues at `slow`: two objects."""
    hyps = make_hyps(lib, classifyPoints=points, corridorGate=0, **overrides)
    arm(lib, hyps, tol_us=2000)
    r, t = 46.0, 0
    for k in range(1, points + 1):
        speed = fast if k <= points // 2 else slow
        t += frame_us
        r += speed * frame_us * 1e-6 / BIN_M
        feed(lib, hyps, k, t, [obs(k, t, r, 1500.0, speed)])
    return hyps


def test_a_two_object_mix_is_rejected(lib):
    assert verdict(lib, mix(lib)).index == -1


def test_the_mix_qualifies_with_the_reject_off(lib):
    assert verdict(lib, mix(lib, maxDecelMps2=0.0)).index >= 0


def test_a_drag_only_ball_is_not_rejected(lib):
    assert verdict(lib, mix(lib, fast=40.0, slow=39.5)).index >= 0


def test_an_origin_crossing_far_from_impact_loses_despite_a_better_residual(lib):
    """Two 42 m/s lines: A left the tee at the anchor (jittered), B left it
    12 ms later (clean). Both pass the 15 ms gate; A must win on back-projection."""
    hyps = make_hyps(lib, corridorGate=0)
    arm(lib, hyps, tol_us=15_000)
    per_us = 42.0 * 1e-6 / BIN_M
    jitter = [0.0, 0.3, -0.3, 0.3, -0.3, 0.3]
    for k in range(1, 7):
        ts = 12_000 + 2000 * k
        line_a = obs(k, ts, 46.0 + per_us * ts + jitter[k - 1], 1500.0, 42.0)
        line_b = obs(k, ts, 46.0 + per_us * (ts - 12_000), 1500.0, 42.0)
        feed(lib, hyps, k, ts, [line_a, line_b])
    v = verdict(lib, hyps)
    assert v.index >= 0
    assert abs(v.originOffsetUs) < 1500.0  # line A


def test_score_terms_are_reported(lib):
    hyps, _ = run(lib, TwoTracks(frames=6))
    v = verdict(lib, hyps)
    # 2 ms frames end 12 ms after the anchor, inside its 15 ms tolerance: no
    # point's implied speed is trusted, so the term is the neutral 0.5.
    assert v.velocityConsistency == 0.5
    assert v.coherence == pytest.approx(0.9)
    assert fw.BALL_ANCHOR_SOURCE_NAMES[v.anchorSource] == "gate"
    expected = 3 * (1 - abs(v.originOffsetUs) / 15_000) + 2 * v.velocityConsistency + (
        1 - v.residualBins / 1.0) + v.dopplerAgreement + 0.5 * v.coherence + 0.5 * v.weakerFraction
    assert v.score == pytest.approx(expected, rel=1e-4)


def test_velocity_consistency_reads_points_beyond_the_tolerance(lib):
    """Frames 3-24 ms after an anchor with a 2 ms tolerance: every implied speed counts."""
    hyps2 = make_hyps(lib)
    arm(lib, hyps2, tol_us=2000)
    for f in TwoTracks(frames=8, frame_us=3000).build():
        feed(lib, hyps2, f.frame, f.timestamp_us, f.targets, f.club_index)
    assert verdict(lib, hyps2).velocityConsistency == pytest.approx(1.0, abs=0.02)


@pytest.mark.parametrize("weight", ["wBack", "wVel", "wResid", "wDoppler", "wCoherence", "wWeaker"])
def test_each_weight_scales_only_its_term(lib, weight):
    base = verdict(lib, run(lib, TwoTracks(frames=6))[0])
    doubled = verdict(lib, run(lib, TwoTracks(frames=6), **{weight: 2 * DEFAULT_WEIGHTS[weight]})[0])
    term = TERMS[weight](base)
    assert doubled.score - base.score == pytest.approx(DEFAULT_WEIGHTS[weight] * term, abs=1e-4)
```

with module constants

```python
DEFAULT_WEIGHTS = {"wBack": 3.0, "wVel": 2.0, "wResid": 1.0, "wDoppler": 1.0, "wCoherence": 0.5, "wWeaker": 0.5}
TERMS = {
    "wBack": lambda v: 1 - abs(v.originOffsetUs) / 15_000,
    "wVel": lambda v: v.velocityConsistency,
    "wResid": lambda v: 1 - v.residualBins / 1.0,
    "wDoppler": lambda v: v.dopplerAgreement,
    "wCoherence": lambda v: v.coherence,
    "wWeaker": lambda v: v.weakerFraction,
}
```

Expected scores for the crossing test: A ≈ 3·~0.95 + 2·S_vel + 0.7 + …; B = 3·0.2 + … + 1.0 (and B's implied speeds from the anchor are inconsistent, lowering its S_vel). A's jitter keeps its residual (~0.3 bin) under the 1-bin gate.

Update `test_the_ball_hypothesis_is_classified_with_its_speed` only if its asserted fields moved (they do not; the verdict grows at the end).

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py -v`
Expected: FAIL.

- [ ] **Step 3: Implement**

`l3_ball_hyp.h` cfg (append):

```c
    /* G4: a hypothesis whose newer half is slower than its older half by
     * more than maxDecelMps2 x the time between them + 2 sigma (each half's
     * slope uncertainty from rangeNoiseM) is two objects. 0 disables. */
    float    maxDecelMps2;
    float    rangeNoiseM;
    /* A3 score weights: back-projection, implied-velocity consistency, fit
     * residual, Doppler agreement, lag-1 coherence, weaker than the club. */
    float    wBack;
    float    wVel;
    float    wResid;
    float    wDoppler;
    float    wCoherence;
    float    wWeaker;
```

verdict (append after `waitingForFast`):

```c
    float    velocityConsistency; /* 0..1: the points' implied launch speeds agree */
    float    coherence;           /* mean lag-1 coherence of its points */
    uint8_t  anchorSource;        /* L3_BALL_ANCHOR_* the search back-projected to */
    uint32_t recovered;           /* points the backward pass added at adoption */
    uint32_t recoveredFirstFrame;
    uint32_t recoveredMask;       /* bit k: frame recoveredFirstFrame + k was recovered */
```

`l3_ball_hyp.c` defaults: `cfg->maxDecelMps2 = 200.0F; cfg->rangeNoiseM = 0.012F; cfg->wBack = 3.0F; cfg->wVel = 2.0F; cfg->wResid = 1.0F; cfg->wDoppler = 1.0F; cfg->wCoherence = 0.5F; cfg->wWeaker = 0.5F;`. Helpers:

```c
static float l3_ball_hyps_unit(float x)
{
    return (x < 0.0F) ? 0.0F : ((x > 1.0F) ? 1.0F : x);
}

/* Sum of squared time deviations (s^2) and mean time (s from ref) of a run. */
static float l3_ball_hyps_timeSpread(const l3_ball_hyp_point_t *p, uint32_t n, uint32_t refUs,
                                     float *meanS)
{
    float mean = 0.0F;
    float sum = 0.0F;
    uint32_t k;

    for (k = 0U; k < n; k++) {
        mean += l3_ball_hyps_seconds(p[k].timestampUs, refUs);
    }
    mean /= (float)n;
    for (k = 0U; k < n; k++) {
        float d = l3_ball_hyps_seconds(p[k].timestampUs, refUs) - mean;

        sum += d * d;
    }
    *meanS = mean;
    return sum;
}

/* G4: 1 when the newer half is slower than drag and range noise allow. */
static int32_t l3_ball_hyps_decelerates(const l3_ball_hyps_cfg_t *cfg, const l3_ball_hyp_t *hyp)
{
    uint32_t older = hyp->count / 2U;
    uint32_t newer = hyp->count - older;
    uint32_t ref = hyp->points[0].timestampUs;
    float rateOld;
    float rateNew;
    float at;
    float residual;
    float meanOld;
    float meanNew;
    float spreadOld;
    float spreadNew;
    float sigmaDelta;

    if (!(cfg->maxDecelMps2 > 0.0F) || older < 2U) {
        return 0;
    }
    if (!l3_ball_points_fit(&hyp->points[0], older, ref, &rateOld, &at, &residual) ||
        !l3_ball_points_fit(&hyp->points[older], newer, ref, &rateNew, &at, &residual)) {
        return 0;
    }
    spreadOld = l3_ball_hyps_timeSpread(&hyp->points[0], older, ref, &meanOld);
    spreadNew = l3_ball_hyps_timeSpread(&hyp->points[older], newer, ref, &meanNew);
    sigmaDelta = cfg->rangeNoiseM * sqrtf(1.0F / spreadOld + 1.0F / spreadNew);
    return ((rateOld - rateNew) * cfg->binWidthM >
            cfg->maxDecelMps2 * (meanNew - meanOld) + 2.0F * sigmaDelta)
               ? 1
               : 0;
}

/* 1 - spread/mean of the implied launch speeds of the points later than the
 * anchor's tolerance (earlier ones divide by a time the tolerance swamps);
 * 0.5 with fewer than two such points. */
static float l3_ball_hyps_velocityConsistency(const l3_ball_hyps_t *hyps, const l3_ball_hyp_t *hyp,
                                              float rateMps)
{
    float v[L3_BALL_HYP_POINTS];
    float mean = 0.0F;
    float var = 0.0F;
    uint32_t n = 0U;
    uint32_t k;

    for (k = 0U; k < hyp->count; k++) {
        float dtS = l3_ball_hyps_seconds(hyp->points[k].timestampUs, hyps->anchor.anchorUs);

        if (dtS * 1.0e6F > (float)hyps->anchor.anchorTolUs) {
            v[n++] = (hyp->points[k].rangeBin - hyps->anchor.anchorBin) * hyps->cfg.binWidthM / dtS;
        }
    }
    if (n < 2U || !(rateMps > 0.0F)) {
        return 0.5F;
    }
    for (k = 0U; k < n; k++) {
        mean += v[k];
    }
    mean /= (float)n;
    for (k = 0U; k < n; k++) {
        var += (v[k] - mean) * (v[k] - mean);
    }
    return 1.0F - l3_ball_hyps_unit(sqrtf(var / (float)n) / rateMps);
}
```

In `judge`: after the residual gate add `if (l3_ball_hyps_decelerates(cfg, hyp)) { return 0; }`; in the per-point loop also sum `coherence += p->coherence;`; then

```c
    out->velocityConsistency = l3_ball_hyps_velocityConsistency(hyps, hyp, rateMps);
    out->coherence = coherence / (float)hyp->count;
    out->anchorSource = hyps->anchor.source;
    out->score =
        cfg->wBack * l3_ball_hyps_unit(1.0F - fabsf(out->originOffsetUs) /
                                                  (float)hyps->anchor.anchorTolUs) +
        cfg->wVel * out->velocityConsistency +
        cfg->wResid * l3_ball_hyps_unit(1.0F - residual / cfg->maxResidualBins) +
        cfg->wDoppler * out->dopplerAgreement + cfg->wCoherence * out->coherence +
        cfg->wWeaker * weakerFraction;
```

(`anchorTolUs` is never 0: the anchor builder always sets it; guard with `> 0U ? … : 0.0F` anyway.)

`firmware_host.py`: append the cfg floats to `BallHypsCfg`; append to `BallHypVerdict`: `("velocityConsistency", c_float), ("coherence", c_float), ("anchorSource", c_uint8), ("recovered", c_uint32), ("recoveredFirstFrame", c_uint32), ("recoveredMask", c_uint32)`.

- [ ] **Step 4: Run the tests**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_hyp.py tests/test_iwr6843_firmware_ball_track.py tests/test_iwr6843_firmware_board_image.py tests/test_iwr6843_firmware_replay.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add firmware/iwr6843/l3_ball_hyp.h firmware/iwr6843/l3_ball_hyp.c src/openflight/iwr6843/firmware_host.py tests/test_iwr6843_firmware_ball_hyp.py
git commit -m "feat(iwr6843): reject two-object ball tracks, score back-projection first

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 9: `l3_ball_recover` — target history and the backward pass

**Files:**
- Create: `firmware/iwr6843/l3_ball_recover.h`, `firmware/iwr6843/l3_ball_recover.c`
- Modify: `firmware/iwr6843/makefile` (SOURCES; `L3_FEATURE_DEFS` add `--define=L3_BALL_RECOVER=0`), `src/openflight/iwr6843/firmware_host.py` (sources, mirrors, bindings; the board-variant build defines — grep `L3_BALL_HYPOTHESES=0` in `firmware_host.py` and add `L3_BALL_RECOVER=0` beside it)
- Test: `tests/test_iwr6843_firmware_ball_recover.py` (new)

**Interfaces:**
- Consumes: `l3_ball_hyp_t`, `l3_ball_hyp_point_t`, `l3_ball_points_fit` (Task 5), `l3_track_wrapped_diff` (`l3_club_track.h`), `l3_target_obs_t`.
- Produces (C):

```c
#ifndef L3_BALL_RECOVER
#define L3_BALL_RECOVER L3_BALL_HYPOTHESES
#endif
#define L3_BALL_HISTORY_FRAMES  24U
#define L3_BALL_HISTORY_TARGETS 6U
typedef struct { float rangeBin; float dopplerAliasMps; float stat; float coherence; } l3_ball_history_target_t;
typedef struct { uint32_t frame; uint32_t timestampUs; uint8_t count; uint8_t clubMask;
                 l3_ball_history_target_t targets[L3_BALL_HISTORY_TARGETS]; } l3_ball_history_frame_t;
typedef struct { uint32_t next; uint32_t count; l3_ball_history_frame_t frames[L3_BALL_HISTORY_FRAMES]; } l3_ball_history_t;
typedef struct { float binWidthM; float velocitySpanMps; float gateM; float tieBins;
                 float maxResidualBins; float dopplerToleranceMps; } l3_ball_recover_cfg_t;
typedef struct { uint32_t count; uint32_t recovered; uint32_t firstFrame; uint32_t mask; float residualBins; } l3_ball_recover_result_t;
void l3_ball_recover_cfg_defaults(l3_ball_recover_cfg_t *cfg);
void l3_ball_history_reset(l3_ball_history_t *h);
void l3_ball_history_push(l3_ball_history_t *h, const l3_target_obs_t *targets, uint32_t n,
                          uint32_t frame, uint32_t timestampUs, uint32_t clubIndex);
const l3_ball_history_frame_t *l3_ball_history_at(const l3_ball_history_t *h, uint32_t index); /* 0 oldest; NULL past count */
uint32_t l3_ball_recover(const l3_ball_recover_cfg_t *cfg, const l3_ball_history_t *h,
                         const l3_ball_hyp_t *hyp, float acceptFromBin,
                         l3_ball_hyp_point_t *out, uint32_t cap, l3_ball_recover_result_t *result);
uint32_t l3_ball_history_struct_bytes(void);
```
- Produces (Python): `fw.BALL_HISTORY_FRAMES`, `fw.BALL_HISTORY_TARGETS`, `BallHistoryTarget`, `BallHistoryFrame`, `BallHistory`, `BallRecoverCfg`, `BallRecoverResult`, bindings.

- [ ] **Step 1: Write the failing tests** — `tests/test_iwr6843_firmware_ball_recover.py`:

```python
"""Tests for the IWR6843 ball recovery, firmware/iwr6843/l3_ball_recover.c.

Once a ball hypothesis is chosen, frames it has no point in are searched
backward along its line in a short target history; returns within a tight gate
join it, nearest first (Doppler only breaks ties), and the whole recovery is
undone when it spoils the fit.
"""

from __future__ import annotations

import ctypes

import pytest
from iwr6843_twotrack import BIN_M, NO_CLAIM, obs

from openflight.iwr6843 import firmware_host as fw

FRAME_US = 3000
SPEED = 40.0
STEP = SPEED * FRAME_US * 1e-6 / BIN_M
TEE = 46.0


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def cfg(lib, **overrides):
    c = fw.BallRecoverCfg()
    lib.l3_ball_recover_cfg_defaults(ctypes.byref(c))
    for k, v in overrides.items():
        setattr(c, k, v)
    return c


def ball_bin(k, t0=0):
    return TEE + STEP * k


def history(lib, frames, extra=None, club=None, start_frame=1, t0=0):
    """frames: {k: [bins]} per post frame k; club: {k: index claimed}."""
    h = fw.BallHistory()
    lib.l3_ball_history_reset(ctypes.byref(h))
    for k in range(start_frame, start_frame + len(frames)):
        ts = (t0 + k * FRAME_US) % 2**32
        targets = [obs(k, ts, b, 1000.0, SPEED) for b in frames[k]]
        arr = (fw.TargetObs * max(1, len(targets)))(*targets)
        claim = (club or {}).get(k, NO_CLAIM)
        lib.l3_ball_history_push(ctypes.byref(h), arr, len(targets), k, ts, claim)
    return h


def hyp_of(points, t0=0):
    """A hypothesis holding (k, bin) points."""
    hyp = fw.BallHyp()
    hyp.active = 1
    hyp.count = len(points)
    for i, (k, b) in enumerate(points):
        p = hyp.points[i]
        p.frame, p.timestampUs, p.rangeBin = k, (t0 + k * FRAME_US) % 2**32, b
        p.dopplerAliasMps, p.stat, p.coherence = SPEED, 1000.0, 0.9
    return hyp


def recover(lib, h, hyp, accept=TEE, cap=32, **overrides):
    out = (fw.BallHypPoint * cap)()
    res = fw.BallRecoverResult()
    n = lib.l3_ball_recover(ctypes.byref(cfg(lib, **overrides)), ctypes.byref(h), ctypes.byref(hyp),
                            accept, out, cap, ctypes.byref(res))
    return [(out[i].frame, round(out[i].rangeBin, 3)) for i in range(n)], res


def test_struct_layout(lib):
    assert ctypes.sizeof(fw.BallHistory) == lib.l3_ball_history_struct_bytes()


def test_frames_the_hypothesis_missed_are_recovered_in_order(lib):
    frames = {k: [ball_bin(k)] for k in range(1, 9)}
    h = history(lib, frames)
    hyp = hyp_of([(k, ball_bin(k)) for k in (1, 4, 6, 7, 8)])
    got, res = recover(lib, h, hyp)
    assert [f for f, _ in got] == list(range(1, 9))
    assert res.recovered == 3
    assert (res.firstFrame, res.mask) == (2, 0b1011)  # frames 2, 3, 5


def test_club_claimed_returns_are_never_recovered(lib):
    frames = {k: [ball_bin(k)] for k in range(1, 7)}
    h = history(lib, frames, club={3: 0})
    got, res = recover(lib, h, hyp_of([(k, ball_bin(k)) for k in (1, 2, 4, 5, 6)]))
    assert 3 not in [f for f, _ in got] and res.recovered == 0


def test_nothing_short_of_the_accept_bin_is_recovered(lib):
    frames = {k: [ball_bin(k)] for k in range(1, 7)}
    h = history(lib, frames)
    got, res = recover(lib, h, hyp_of([(k, ball_bin(k)) for k in (4, 5, 6)]), accept=ball_bin(3) + 0.1)
    assert res.recovered == 0


def test_the_nearest_candidate_wins_and_doppler_only_breaks_ties(lib):
    frames = {k: [ball_bin(k)] for k in range(1, 7)}
    frames[3] = [ball_bin(3) + 0.3, ball_bin(3) - 0.05]  # nearer one second in the list
    h = history(lib, frames)
    got, _ = recover(lib, h, hyp_of([(k, ball_bin(k)) for k in (1, 2, 4, 5, 6)]))
    assert dict(got)[3] == pytest.approx(ball_bin(3) - 0.05, abs=1e-3)


def test_an_equal_distance_decoy_loses_on_doppler(lib):
    h = fw.BallHistory()
    lib.l3_ball_history_reset(ctypes.byref(h))
    for k in range(1, 7):
        ts = k * FRAME_US
        targets = [obs(k, ts, ball_bin(k), 1000.0, SPEED)]
        if k == 3:  # decoy first, same distance, wrong Doppler
            targets = [obs(k, ts, ball_bin(k) + 0.1, 5000.0, 0.0), obs(k, ts, ball_bin(k) - 0.1, 900.0, SPEED)]
        arr = (fw.TargetObs * len(targets))(*targets)
        lib.l3_ball_history_push(ctypes.byref(h), arr, len(targets), k, ts, NO_CLAIM)
    got, _ = recover(lib, h, hyp_of([(k, ball_bin(k)) for k in (1, 2, 4, 5, 6)]))
    assert dict(got)[3] == pytest.approx(ball_bin(3) - 0.1, abs=1e-3)


def test_a_recovery_that_spoils_the_fit_is_undone_whole(lib):
    frames = {k: [ball_bin(k)] for k in range(1, 7)}
    frames[2] = [ball_bin(2) + 0.55]  # inside a wide gate, but off the line
    frames[3] = [ball_bin(3) + 0.55]
    h = history(lib, frames)
    hyp = hyp_of([(k, ball_bin(k)) for k in (1, 4, 5, 6)])
    got, res = recover(lib, h, hyp, gateM=0.6 * BIN_M, maxResidualBins=0.1)
    assert res.recovered == 0
    assert [f for f, _ in got] == [1, 4, 5, 6]


def test_the_history_ring_keeps_only_the_newest_frames(lib):
    n = fw.BALL_HISTORY_FRAMES + 5
    h = history(lib, {k: [ball_bin(k)] for k in range(1, n + 1)})
    assert h.count == fw.BALL_HISTORY_FRAMES
    oldest = lib.l3_ball_history_at(ctypes.byref(h), 0)
    assert oldest.contents.frame == 6
    assert not lib.l3_ball_history_at(ctypes.byref(h), fw.BALL_HISTORY_FRAMES)


def test_more_targets_than_slots_keep_the_strongest(lib):
    h = history(lib, {1: [50.0 + i for i in range(fw.BALL_HISTORY_TARGETS + 3)]})
    assert lib.l3_ball_history_at(ctypes.byref(h), 0).contents.count == fw.BALL_HISTORY_TARGETS


def test_the_merge_respects_the_capacity_and_keeps_the_newest(lib):
    frames = {k: [ball_bin(k)] for k in range(1, 9)}
    h = history(lib, frames)
    got, _ = recover(lib, h, hyp_of([(k, ball_bin(k)) for k in (1, 8)]), cap=4)
    assert len(got) == 4 and got[-1][0] == 8


def test_recovery_across_the_clock_wrap(lib):
    t0 = 2**32 - 4 * FRAME_US
    frames = {k: [ball_bin(k)] for k in range(1, 7)}
    h = history(lib, frames, t0=t0)
    got, res = recover(lib, h, hyp_of([(k, ball_bin(k)) for k in (1, 4, 5, 6)], t0=t0))
    assert res.recovered == 2


def test_without_a_fit_the_hypothesis_points_pass_through(lib):
    h = history(lib, {1: [ball_bin(1)]})
    got, res = recover(lib, h, hyp_of([(1, ball_bin(1))]))
    assert got == [(1, round(ball_bin(1), 3))] and res.recovered == 0
```

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_recover.py -v`
Expected: FAIL (`BallRecoverCfg` missing).

- [ ] **Step 3: Implement**

`firmware/iwr6843/l3_ball_recover.h` — the interface above with this header comment:

```c
/* IWR6843 ball recovery: the frames a chosen ball hypothesis missed.
 *
 * A short ring holds each post-impact frame's targets (the strongest
 * L3_BALL_HISTORY_TARGETS, the club's claimed one flagged). When a hypothesis
 * is adopted as the ball, every held frame up to its newest point without a
 * point of its own is searched along its fitted line: the nearest return
 * within gateM, beyond acceptFromBin and not the club's, joins; two within
 * tieBins of each other go to the one whose Doppler agrees with the rate
 * (Doppler only breaks ties: as a gate it made the ball worse). The merge is
 * kept only while the refit stays within maxResidualBins. Recovered points
 * carry no angles. Pure C, fixed size, no hardware.
 */
```

plus

```c
#if L3_BALL_RECOVER && !L3_BALL_HYPOTHESES
#error "L3_BALL_RECOVER needs L3_BALL_HYPOTHESES"
#endif
```

after the includes (`l3_ball_hyp.h`, `l3_observation.h`).

`firmware/iwr6843/l3_ball_recover.c`:

```c
#include "l3_ball_recover.h"

#include <math.h>
#include <string.h>

#include "l3_club_track.h"

void l3_ball_recover_cfg_defaults(l3_ball_recover_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->binWidthM = 6.0F / 128.0F;
    cfg->velocitySpanMps = 2.0F * L3_OBS_WAVELENGTH_M / (4.0F * 135.0e-6F);
    cfg->gateM = 0.6F * cfg->binWidthM;
    cfg->tieBins = 0.1F;
    cfg->maxResidualBins = 1.0F;
    cfg->dopplerToleranceMps = 2.5F;
}

void l3_ball_history_reset(l3_ball_history_t *h)
{
    memset(h, 0, sizeof(*h));
}

void l3_ball_history_push(l3_ball_history_t *h, const l3_target_obs_t *targets, uint32_t n,
                          uint32_t frame, uint32_t timestampUs, uint32_t clubIndex)
{
    l3_ball_history_frame_t *f = &h->frames[h->next];
    uint32_t k;

    memset(f, 0, sizeof(*f));
    f->frame = frame;
    f->timestampUs = timestampUs;
    for (k = 0U; k < n && k < L3_BALL_HISTORY_TARGETS; k++) {
        f->targets[k].rangeBin = targets[k].rangeBin;
        f->targets[k].dopplerAliasMps = targets[k].dopplerAliasMps;
        f->targets[k].stat = targets[k].stat;
        f->targets[k].coherence = targets[k].coherence;
        if (k == clubIndex) {
            f->clubMask |= (uint8_t)(1U << k);
        }
    }
    f->count = (uint8_t)k;
    h->next = (h->next + 1U) % L3_BALL_HISTORY_FRAMES;
    if (h->count < L3_BALL_HISTORY_FRAMES) {
        h->count++;
    }
}

const l3_ball_history_frame_t *l3_ball_history_at(const l3_ball_history_t *h, uint32_t index)
{
    if (index >= h->count) {
        return NULL;
    }
    return &h->frames[(h->next + L3_BALL_HISTORY_FRAMES - h->count + index) %
                      L3_BALL_HISTORY_FRAMES];
}

static int32_t l3_ball_recover_hasFrame(const l3_ball_hyp_t *hyp, uint32_t frame)
{
    uint32_t k;

    for (k = 0U; k < hyp->count; k++) {
        if (hyp->points[k].frame == frame) {
            return 1;
        }
    }
    return 0;
}

/* The candidate in f for a ball at predictedBin moving at rateMps; -1 when none. */
static int32_t l3_ball_recover_pick(const l3_ball_recover_cfg_t *cfg,
                                    const l3_ball_history_frame_t *f, float predictedBin,
                                    float acceptFromBin, float rateMps)
{
    float gateBins = cfg->gateM / cfg->binWidthM;
    int32_t best = -1;
    float bestErr = 0.0F;
    uint32_t k;

    for (k = 0U; k < f->count; k++) {
        const l3_ball_history_target_t *t = &f->targets[k];
        float err = fabsf(t->rangeBin - predictedBin);

        if ((f->clubMask & (1U << k)) != 0U || t->rangeBin < acceptFromBin || err > gateBins) {
            continue;
        }
        if (best >= 0 && fabsf(err - bestErr) <= cfg->tieBins) {
            int32_t agrees = l3_track_wrapped_diff(rateMps, t->dopplerAliasMps,
                                                   cfg->velocitySpanMps) <=
                             cfg->dopplerToleranceMps;
            int32_t bestAgrees =
                l3_track_wrapped_diff(rateMps, f->targets[best].dopplerAliasMps,
                                      cfg->velocitySpanMps) <= cfg->dopplerToleranceMps;

            if (agrees && !bestAgrees) {
                best = (int32_t)k;
                bestErr = err;
            } else if (agrees == bestAgrees && err < bestErr) {
                best = (int32_t)k;
                bestErr = err;
            }
            continue;
        }
        if (best < 0 || err < bestErr) {
            best = (int32_t)k;
            bestErr = err;
        }
    }
    return best;
}

/* Merge two time-ordered runs into out (at most cap, newest kept). */
static uint32_t l3_ball_recover_merge(const l3_ball_hyp_point_t *a, uint32_t na,
                                      const l3_ball_hyp_point_t *b, uint32_t nb,
                                      l3_ball_hyp_point_t *out, uint32_t cap)
{
    uint32_t total = na + nb;
    uint32_t skip = (total > cap) ? total - cap : 0U;
    uint32_t i = 0U;
    uint32_t j = 0U;
    uint32_t n = 0U;

    while (i < na || j < nb) {
        const l3_ball_hyp_point_t *p;

        if (j >= nb || (i < na && (int32_t)(a[i].timestampUs - b[j].timestampUs) <= 0)) {
            p = &a[i++];
        } else {
            p = &b[j++];
        }
        if (skip > 0U) {
            skip--;
            continue;
        }
        out[n++] = *p;
    }
    return n;
}

uint32_t l3_ball_recover(const l3_ball_recover_cfg_t *cfg, const l3_ball_history_t *h,
                         const l3_ball_hyp_t *hyp, float acceptFromBin,
                         l3_ball_hyp_point_t *out, uint32_t cap, l3_ball_recover_result_t *result)
{
    l3_ball_hyp_point_t found[L3_BALL_HISTORY_FRAMES];
    uint32_t newestUs;
    uint32_t refUs;
    uint32_t nFound = 0U;
    uint32_t n;
    float rate;
    float atRef;
    float residual;
    uint32_t i;

    memset(result, 0, sizeof(*result));
    if (hyp->count == 0U || cap == 0U) {
        return 0U;
    }
    refUs = hyp->points[0].timestampUs;
    newestUs = hyp->points[hyp->count - 1U].timestampUs;
    if (l3_ball_points_fit(hyp->points, hyp->count, refUs, &rate, &atRef, &residual)) {
        float rateMps = rate * cfg->binWidthM;

        for (i = 0U; i < h->count; i++) {
            const l3_ball_history_frame_t *f = l3_ball_history_at(h, i);
            float dtS;
            int32_t k;

            if ((int32_t)(f->timestampUs - newestUs) >= 0 ||
                l3_ball_recover_hasFrame(hyp, f->frame)) {
                continue;
            }
            dtS = (float)(int32_t)(f->timestampUs - refUs) * 1.0e-6F;
            k = l3_ball_recover_pick(cfg, f, atRef + rate * dtS, acceptFromBin, rateMps);
            if (k < 0) {
                continue;
            }
            memset(&found[nFound], 0, sizeof(found[nFound]));
            found[nFound].frame = f->frame;
            found[nFound].timestampUs = f->timestampUs;
            found[nFound].rangeBin = f->targets[k].rangeBin;
            found[nFound].dopplerAliasMps = f->targets[k].dopplerAliasMps;
            found[nFound].stat = f->targets[k].stat;
            found[nFound].coherence = f->targets[k].coherence;
            nFound++;
        }
    }
    if (nFound > 0U) {
        n = l3_ball_recover_merge(hyp->points, hyp->count, found, nFound, out, cap);
        if (l3_ball_points_fit(out, n, out[0].timestampUs, &rate, &atRef, &residual) &&
            residual <= cfg->maxResidualBins) {
            result->count = n;
            result->residualBins = residual;
            result->firstFrame = found[0].frame;
            for (i = 0U; i < nFound; i++) {
                uint32_t bit = found[i].frame - result->firstFrame;

                if (bit < 32U) {
                    result->mask |= 1U << bit;
                }
            }
            result->recovered = nFound;
            return n;
        }
    }
    n = l3_ball_recover_merge(hyp->points, hyp->count, NULL, 0U, out, cap);
    result->count = n;
    return n;
}

uint32_t l3_ball_history_struct_bytes(void)
{
    return (uint32_t)sizeof(l3_ball_history_t);
}
```

(The capped merge may drop recovered points; `recovered` still counts what was found — adjust `result->recovered` to the number of `found` points that made it into `out` if the capacity test shows a mismatch. With the real capacity of 32 and at most 8 + 24 points it never caps.)

`makefile`: add `l3_ball_recover.c` to `SOURCES` after `l3_ball_hyp.c`; `L3_FEATURE_DEFS ?= --define=L3_BALL_HYPOTHESES=0 --define=L3_BALL_RECOVER=0 --define=L3_TRIG_TRACE_DEPTH=24U` and the comment above it showing how to turn both on. `firmware_host.py`: `"l3_ball_recover.c"` in `HOST_SOURCES`; constants `BALL_HISTORY_FRAMES = 24`, `BALL_HISTORY_TARGETS = 6`; mirrors

```python
class BallHistoryTarget(ctypes.Structure):
    """``l3_ball_history_target_t``."""

    _fields_ = [("rangeBin", ctypes.c_float), ("dopplerAliasMps", ctypes.c_float),
                ("stat", ctypes.c_float), ("coherence", ctypes.c_float)]


class BallHistoryFrame(ctypes.Structure):
    """``l3_ball_history_frame_t``."""

    _fields_ = [("frame", ctypes.c_uint32), ("timestampUs", ctypes.c_uint32),
                ("count", ctypes.c_uint8), ("clubMask", ctypes.c_uint8),
                ("targets", BallHistoryTarget * BALL_HISTORY_TARGETS)]


class BallHistory(ctypes.Structure):
    """``l3_ball_history_t``: the post-impact target ring."""

    _fields_ = [("next", ctypes.c_uint32), ("count", ctypes.c_uint32),
                ("frames", BallHistoryFrame * BALL_HISTORY_FRAMES)]


class BallRecoverCfg(ctypes.Structure):
    """``l3_ball_recover_cfg_t``."""

    _fields_ = [("binWidthM", ctypes.c_float), ("velocitySpanMps", ctypes.c_float),
                ("gateM", ctypes.c_float), ("tieBins", ctypes.c_float),
                ("maxResidualBins", ctypes.c_float), ("dopplerToleranceMps", ctypes.c_float)]


class BallRecoverResult(ctypes.Structure):
    """``l3_ball_recover_result_t``."""

    _fields_ = [("count", ctypes.c_uint32), ("recovered", ctypes.c_uint32),
                ("firstFrame", ctypes.c_uint32), ("mask", ctypes.c_uint32),
                ("residualBins", ctypes.c_float)]
```

bindings: `l3_ball_recover_cfg_defaults ([_P(BallRecoverCfg)], None)`, `l3_ball_history_reset ([_P(BallHistory)], None)`, `l3_ball_history_push ([_P(BallHistory), _P(TargetObs), _U32, _U32, _U32, _U32], None)`, `l3_ball_history_at ([_P(BallHistory), _U32], _P(BallHistoryFrame))`, `l3_ball_recover ([_P(BallRecoverCfg), _P(BallHistory), _P(BallHyp), _F32, _P(BallHypPoint), _U32, _P(BallRecoverResult)], _U32)`, `l3_ball_history_struct_bytes ([], _U32)`.

- [ ] **Step 4: Run the tests**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_recover.py tests/test_iwr6843_firmware_board_image.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add firmware/iwr6843/l3_ball_recover.h firmware/iwr6843/l3_ball_recover.c firmware/iwr6843/makefile src/openflight/iwr6843/firmware_host.py tests/test_iwr6843_firmware_ball_recover.py
git commit -m "feat(iwr6843): post-impact target history and backward ball recovery

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 10: Wire recovery into the ball track; board image; replay reporting (B2–B4, P1)

**Files:**
- Modify: `firmware/iwr6843/l3_ball_track.{h,c}`, `firmware/iwr6843/l3_dump.c:3661`, `src/openflight/iwr6843/firmware_host.py`, `src/openflight/iwr6843/firmware_replay.py` (~1872 ball params, `ReplayResult`, result construction), `tests/test_iwr6843_firmware_board_image.py`
- Test: `tests/test_iwr6843_firmware_ball_track.py`, `tests/test_iwr6843_firmware_board_image.py`

**Interfaces:**
- Consumes: Task 9 module, Task 8 verdict fields.
- Produces:
  - `l3_ball_track_cfg_t` (after `hyps`, under `#if L3_BALL_RECOVER`): `uint32_t recover; float historySnr; l3_ball_recover_cfg_t rec;`
  - `l3_ball_track_t` (after `verdict`, under `#if L3_BALL_RECOVER`): `l3_ball_history_t history;`
  - `float l3_ball_track_extract_snr(const l3_ball_track_cfg_t *cfg, float searchSnr);`
  - `ReplayResult.recovered_frames: tuple[int, ...] = ()`

- [ ] **Step 1: Write the failing tests** (ball-track test file)

```python
def joint(ball, scene):
    frames = scene.build()
    for f in frames:
        arr = (fw.TargetObs * max(1, len(f.targets)))(*f.targets)
        ball.lib.l3_ball_track_update_joint(ctypes.byref(ball.track), arr, len(f.targets),
                                            f.frame, f.timestamp_us, fw.TRACK_NO_TARGET)
    return frames


def test_adoption_recovers_the_frames_the_search_missed(lib):
    """Four strong still returns fill every slot in frames 1-3, so the ball's
    hypothesis only starts at frame 3 (the stills drop as stalled there). The
    history still holds the ball's frames 1 and 2: adoption recovers them."""
    ball = Ball(lib, useHypotheses=1)
    ball.arm()
    step = 42.0 * FRAME_US * 1e-6 / BIN_M
    for k in range(1, 9):
        ts = IMPACT_US + k * FRAME_US
        targets = [obs(k, ts, ORIGIN_BIN + d, 9000.0, 0.0) for d in (12.0, 13.0, 14.0, 15.0)] if k <= 3 else []
        targets.append(obs(k, ts, ORIGIN_BIN + step * k, 1500.0, 42.0))
        arr = (fw.TargetObs * len(targets))(*targets)
        lib.l3_ball_track_update_joint(ctypes.byref(ball.track), arr, len(targets), k, ts,
                                       fw.TRACK_NO_TARGET)
    v = ball.track.verdict
    assert ball.track.confirmed
    assert (v.recovered, v.recoveredFirstFrame, v.recoveredMask) == (2, 1, 0b11)
    assert ball.track.core.count == 6  # frames 1-6: two recovered + the four adopted


def test_recovery_off_seeds_only_the_hypothesis_points(lib):
    ball = Ball(lib, useHypotheses=1, recover=0)
    ball.arm()
    joint(ball, TwoTracks(origin_bin=ORIGIN_BIN, gate_us=IMPACT_US, frame_us=FRAME_US, frames=8,
                          club_visible=False))
    assert ball.track.confirmed and ball.track.verdict.recovered == 0


def test_the_history_is_fed_while_searching(lib):
    ball = Ball(lib, useHypotheses=1)
    ball.arm()
    joint(ball, TwoTracks(origin_bin=ORIGIN_BIN, gate_us=IMPACT_US, frame_us=FRAME_US, frames=3,
                          club_visible=False))
    assert ball.track.history.count == 3


def test_extract_snr_is_the_lower_history_threshold_only_with_recovery(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    assert lib.l3_ball_track_extract_snr(ctypes.byref(cfg), 1.0) == 1.0  # historySnr 0: same
    cfg.historySnr = 0.7
    assert lib.l3_ball_track_extract_snr(ctypes.byref(cfg), 1.0) == pytest.approx(0.7)
    cfg.recover = 0
    assert lib.l3_ball_track_extract_snr(ctypes.byref(cfg), 1.0) == 1.0


def test_targets_under_the_search_snr_reach_only_the_history(lib):
    ball = Ball(lib, useHypotheses=1, historySnr=0.5)
    ball.arm()
    weak = target(1, ORIGIN_BIN + 3.0)
    weak.snr = 0.6  # above historySnr, under snr (1.0)
    arr = (fw.TargetObs * 1)(weak)
    lib.l3_ball_track_update_joint(ctypes.byref(ball.track), arr, 1, 1, FRAME_US, fw.TRACK_NO_TARGET)
    assert ball.track.history.count == 1
    assert not any(ball.track.hyps.hyp[i].active for i in range(fw.BALL_HYP_MAX))
```

`obs` comes from `iwr6843_twotrack` (already imported by this test file). Why the scene works: in frame 1 the four stills (strongest first) take every slot and none is evictable (each was fed); in frame 2 each still is extended; in frame 3 each gets its third point and G2 drops all four as stalled, after which the ball's frame-3 return spawns. Frames 3–6 classify it (4 points); adoption walks the history back and finds the ball exactly on the line in frames 1 and 2.

Board-image test: replace the saved-bytes sum in `test_the_board_ball_track_drops_exactly_the_hypotheses` with

```python
    saved = ctypes.sizeof(fw.BallHyps) + ctypes.sizeof(fw.BallHypVerdict)
    saved += ctypes.sizeof(fw.BallHypsCfg)  # the track's copy of its cfg
    saved += ctypes.sizeof(fw.BallHistory)  # L3_BALL_RECOVER
    saved += ctypes.sizeof(fw.BallRecoverCfg) + 8  # recover, historySnr
```

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_iwr6843_firmware_ball_track.py tests/test_iwr6843_firmware_board_image.py -v`
Expected: FAIL.

- [ ] **Step 3: Implement**

`l3_ball_track.h`: `#include "l3_ball_recover.h"`; in the cfg after the `hyps` block:

```c
#if L3_BALL_RECOVER
    /* Recover the frames the adopted hypothesis missed from the post-impact
     * history (l3_ball_recover.h). historySnr under snr (0: the same) lets the
     * history hold weaker returns than the searches see. */
    uint32_t recover;
    float    historySnr;
    l3_ball_recover_cfg_t rec;    /* binWidthM and velocitySpanMps come from core */
#endif
```

struct after `verdict` block: `#if L3_BALL_RECOVER\n    l3_ball_history_t history;\n#endif`. Prototype + comment:

```c
/* The extraction threshold for the post window: searchSnr, or the history's
 * lower historySnr with recovery on. */
float l3_ball_track_extract_snr(const l3_ball_track_cfg_t *cfg, float searchSnr);
```

`l3_ball_track.c`:
- defaults: `#if L3_BALL_RECOVER cfg->recover = 1U; cfg->historySnr = 0.0F; l3_ball_recover_cfg_defaults(&cfg->rec); #endif`
- init: `#if L3_BALL_RECOVER track->cfg.rec.binWidthM = cfg->core.binWidthM; track->cfg.rec.velocitySpanMps = cfg->core.velocitySpanMps; track->cfg.rec.maxResidualBins = track->cfg.hyps.maxResidualBins; track->cfg.rec.dopplerToleranceMps = track->cfg.hyps.dopplerToleranceMps; l3_ball_history_reset(&track->history); #endif`
- reset: `#if L3_BALL_RECOVER l3_ball_history_reset(&track->history); #endif`
- extract snr:

```c
float l3_ball_track_extract_snr(const l3_ball_track_cfg_t *cfg, float searchSnr)
{
#if L3_BALL_RECOVER
    if (cfg->recover && cfg->historySnr > 0.0F && cfg->historySnr < searchSnr) {
        return cfg->historySnr;
    }
#else
    (void)cfg;
#endif
    return searchSnr;
}
```

- in `l3_ball_track_update_joint`, in the searching branch before `l3_ball_hyps_update`:

```c
#if L3_BALL_RECOVER
    if (track->cfg.recover) {
        static l3_target_obs_t searchable[L3_OBS_MAX_TARGETS];
        uint32_t kept = 0U;
        uint32_t keptClub = L3_TRACK_NO_TARGET;
        uint32_t j;

        l3_ball_history_push(&track->history, targets, n, frame, timestampUs, clubIndex);
        if (track->cfg.historySnr > 0.0F && track->cfg.historySnr < track->cfg.snr) {
            /* The history holds the weaker returns; the searches see snr. */
            for (j = 0U; j < n && j < L3_OBS_MAX_TARGETS; j++) {
                if (targets[j].snr >= track->cfg.snr) {
                    if (j == clubIndex) {
                        keptClub = kept;
                    }
                    searchable[kept++] = targets[j];
                }
            }
            targets = searchable;
            n = kept;
            clubIndex = keptClub;
        }
    }
#endif
```

(`targets`, `n`, `clubIndex` are parameters; assigning them locally is fine in C. The board compiles this out; on the host the static buffer is acceptable.)

- in `l3_ball_track_adopt`, choose the seed points:

```c
    const l3_ball_hyp_point_t *points = hyp->points;
    uint32_t count = hyp->count;
#if L3_BALL_RECOVER
    static l3_ball_hyp_point_t merged[L3_TRACK_POINTS];
    l3_ball_recover_result_t rec;

    if (track->cfg.recover) {
        count = l3_ball_recover(&track->cfg.rec, &track->history, hyp,
                                track->anchor.acceptFromBin, merged, L3_TRACK_POINTS, &rec);
        points = merged;
        track->verdict.recovered = rec.recovered;
        track->verdict.recoveredFirstFrame = rec.firstFrame;
        track->verdict.recoveredMask = rec.mask;
    }
#endif
```

and loop `for (k = 0U; k < count; k++) { const l3_ball_hyp_point_t *p = &points[k]; …` with `seed.coherence = p->coherence;` (was 1.0F).

- `l3_ball_track_format_status`: append ` rec=%u` with `#if L3_BALL_HYPOTHESES (unsigned)track->verdict.recovered #else 0U #endif` (compute into a local `unsigned recovered` before the `snprintf`).

`l3_dump.c:3661`: `params.snr = l3_ball_track_extract_snr(&gBallTrack.cfg, (gBallSnr > 0.0F) ? gBallSnr : gBallTrackCfg.snr);`

`firmware_host.py`: `BallTrackCfg` append `("recover", c_uint32), ("historySnr", c_float), ("rec", BallRecoverCfg)` after `hyps`; `BallTrack` append `("history", BallHistory)` after `verdict` (move the recovery mirrors above `BallTrackCfg`); binding `"l3_ball_track_extract_snr": ([_P(BallTrackCfg), _F32], _F32)`.

`firmware_replay.py`: line ~1872 `ball_params = fw.ObsParams(params.stat, lib.l3_ball_track_extract_snr(ctypes.byref(ball_track.cfg), ball_track.cfg.snr), params.loopPeriodS, params.subBin)`; `ReplayResult` add `recovered_frames: tuple[int, ...] = ()  # frames the backward pass added at adoption` and where the result is built:

```python
        recovered_frames=tuple(
            int(ball_track.verdict.recoveredFirstFrame) + bit
            for bit in range(32)
            if ball_track.verdict.recoveredMask >> bit & 1
        ),
```

- [ ] **Step 4: Run the whole firmware suite**

Run: `uv run pytest tests/ -k "iwr6843 or evaluate_iwr" -v`
Expected: PASS (legacy recordings unchanged: `useHypotheses` is still 0).

- [ ] **Step 5: Commit**

```bash
git add firmware/iwr6843 src/openflight/iwr6843 tests
git commit -m "feat(iwr6843): the ball track keeps a target history and recovers at adoption

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 11: Evaluator switches (E4) and the net diagnostic (E3)

**Files:**
- Modify: `src/openflight/iwr6843/firmware_replay.py` (`BallTuning`), `scripts/analysis/evaluate_iwr_tracking.py` (`parse_tuning`, `main`, `Outcome`, `evaluate`, new `net_reached`)
- Test: `tests/test_evaluate_iwr_tracking.py`, `tests/test_iwr6843_firmware_replay.py` (BallTuning apply)

**Interfaces:**
- Produces: `BallTuning` fields `corridor_gate: bool | None`, `impact_coast_ms: float | None`, `max_decel_mps2: float | None`, `classify_points: int | None`, `recover: bool | None`, `recover_gate_m: float | None`, `history_snr: float | None` (plus the existing ones and `far_window_m`); `net_reached(ball_points, launch_mps, impact_us, tee_m, net_range_m, frame_us) -> bool | None`; `Outcome.net_reached: bool | None = None`; CLI `--corridor-gate on|off`, `--impact-coast-ms`, `--max-decel`, `--classify-points`, `--recover on|off`, `--recover-gate-m`, `--history-snr`, `--net-range-m`.

- [ ] **Step 1: Write the failing tests**

`tests/test_iwr6843_firmware_replay.py`:

```python
def test_ball_tuning_writes_every_new_switch(lib):
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    fr.BallTuning(corridor_gate=False, impact_coast_ms=24.0, max_decel_mps2=0.0, classify_points=6,
                  recover=False, recover_gate_m=0.05, history_snr=0.7, far_window_m=0.1).apply(cfg)
    assert (cfg.hyps.corridorGate, cfg.hyps.impactCoastUs, cfg.hyps.maxDecelMps2, cfg.hyps.classifyPoints) == (
        0, 24_000, 0.0, 6)
    assert (cfg.recover, cfg.historySnr) == (0, pytest.approx(0.7))
    assert cfg.rec.gateM == pytest.approx(0.05) and cfg.hyps.farWindowM == pytest.approx(0.1)
```

(use that file's existing `lib` fixture and imports.)

`tests/test_evaluate_iwr_tracking.py`:

```python
def pts(*rows):
    return [SimpleNamespace(timestamp_us=us, range_m=m) for us, m in rows]


def test_net_reached_when_the_track_arrives_on_time(ev):
    # tee 2.0 m, net 4.5 m, 50 m/s from impact 0: due at 50 ms; 3 ms frames, +-6 ms
    assert ev.net_reached(pts((47_000, 4.40), (50_000, 4.50)), 50.0, 0, 2.0, 4.5, 3000) is True
    assert ev.net_reached(pts((60_000, 4.50)), 50.0, 0, 2.0, 4.5, 3000) is False
    assert ev.net_reached(pts((30_000, 3.5)), 50.0, 0, 2.0, 4.5, 3000) is False  # never got there
    assert ev.net_reached([], None, 0, 2.0, 4.5, 3000) is None  # no launch
```

and extend the CLI pass-through test to the new flags (assert the `BallTuning` built by `parse_tuning` has each value).

- [ ] **Step 2: Run to verify they fail**

Run: `uv run pytest tests/test_evaluate_iwr_tracking.py tests/test_iwr6843_firmware_replay.py -k "tuning or net_reached or cli" -v`
Expected: FAIL.

- [ ] **Step 3: Implement**

`BallTuning` (keep the docstring, add one line per new switch):

```python
    corridor_gate: bool | None = None
    impact_coast_ms: float | None = None
    max_decel_mps2: float | None = None
    classify_points: int | None = None
    recover: bool | None = None
    recover_gate_m: float | None = None
    history_snr: float | None = None

    def apply(self, cfg: fw.BallTrackCfg) -> None:
        """Write the set overrides into a ball-track configuration."""
        hyps = cfg.hyps
        if self.fast_ball_mps is not None:
            hyps.fastBallMps = self.fast_ball_mps
        if self.fast_support_fraction is not None:
            hyps.fastSupportFraction = self.fast_support_fraction
        if self.min_departure_mps is not None:
            cfg.minDepartureMps = self.min_departure_mps
            hyps.minDepartureMps = self.min_departure_mps
        if self.far_window_m is not None:
            hyps.farWindowM = self.far_window_m
        if self.corridor_gate is not None:
            hyps.corridorGate = 1 if self.corridor_gate else 0
        if self.impact_coast_ms is not None:
            hyps.impactCoastUs = round(self.impact_coast_ms * 1000)
        if self.max_decel_mps2 is not None:
            hyps.maxDecelMps2 = self.max_decel_mps2
        if self.classify_points is not None:
            hyps.classifyPoints = self.classify_points
        if self.recover is not None:
            cfg.recover = 1 if self.recover else 0
        if self.recover_gate_m is not None:
            cfg.rec.gateM = self.recover_gate_m
        if self.history_snr is not None:
            cfg.historySnr = self.history_snr
```

Evaluator:

```python
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
```

`evaluate(..., net_range_m: float | None = None)`: when given, `frame_us` = median of consecutive post-frame timestamp differences (`int(np.median(np.diff([f.timestamp_us for f in post])))` with ≥ 2 post frames, else None → `net_reached` None), `tee_m = tee_bin * bin_width_m()`, `impact_us = int(result.shot.impactTimestampUs)`, pass `result.ball_points`. `Outcome.net_reached: bool | None = None`; `summarize` adds `"net_reached": sum(1 for o in outcomes if o.net_reached)` only when any outcome has a non-None value.

CLI flags (each with a one-line help): `--corridor-gate {on,off}`, `--impact-coast-ms float`, `--max-decel float`, `--classify-points int`, `--recover {on,off}`, `--recover-gate-m float`, `--history-snr float`, `--net-range-m float`; `parse_tuning` maps on/off to bool; `main` passes `net_range_m=args.net_range_m` to `evaluate`.

- [ ] **Step 4: Run tests and lint**

Run: `uv run pytest tests/test_evaluate_iwr_tracking.py tests/test_iwr6843_firmware_replay.py -v && uv run pylint src/openflight/ --fail-under=9 && uv run ruff check src/openflight/ && uv run ruff format --check src/openflight/`
Expected: PASS, pylint ≥ 9.

- [ ] **Step 5: Commit**

```bash
git add src/openflight/iwr6843/firmware_replay.py scripts/analysis/evaluate_iwr_tracking.py tests
git commit -m "feat(eval): ball search switches for ablation and the net diagnostic

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```

---

### Task 12: Evaluate, ablate, decide, record

**Files:**
- Create: `docs/superpowers/specs/2026-10-01-ball-search-hypotheses-<variant>.json` per run
- Modify: `docs/superpowers/specs/2026-10-01-iwr-gap-tolerant-ball-search-design.md` (Results), and only if E2 passes, `firmware/iwr6843/l3_ball_track.c` (`useHypotheses` default) with its test

- [ ] **Step 1: Full run** (ask the user for `<SESSIONS>` and, for E3, the radar-to-net distance)

```bash
uv run python scripts/analysis/evaluate_iwr_tracking.py <SESSIONS> --ball-hypotheses on --json docs/superpowers/specs/2026-10-01-ball-search-hypotheses-all.json --accept docs/superpowers/specs/2026-10-01-ball-search-baseline-legacy.json --net-range-m <NET_M>
```

- [ ] **Step 2: Ablation** — one run each, `--ball-hypotheses on` plus: `--corridor-gate off`; `--impact-coast-ms 6`; `--max-decel 0`; `--recover off`; `--classify-points 6`; `--history-snr 0.7`. Save each JSON as `…-hypotheses-<flag>.json`.

- [ ] **Step 3: Board bytes** — build the board variant with both switches on (`make L3_FEATURE_DEFS="--define=L3_BALL_HYPOTHESES=1 --define=L3_BALL_RECOVER=1"` if the TI toolchain is available; otherwise report `ctypes.sizeof(fw.BallHyps)+…` from Task 10's board-image test) and record the DATA_RAM added by each switch.

- [ ] **Step 4: Record** in the spec's Results: a table of the split (present ok/wrong/none, absent ok/wrong/none, club) for legacy, all-on and each ablation; `accept_split` lines; the net column; capture-by-capture changes for `20260927_144220_262_005` and `20260927_183542_142_009`; the board bytes.

- [ ] **Step 5: Decide.** If `--accept` printed nothing: set `cfg->useHypotheses = 1U;` in `l3_ball_track_cfg_defaults`, update the default assertion in `tests/test_iwr6843_firmware_ball_track.py`, run `uv run pytest tests/ -v`, and note in the spec that the board image still compiles the search out pending the RAM decision. If it printed problems: leave the default at 0 and write which switch moved which number.

- [ ] **Step 6: Commit**

```bash
git add docs/superpowers/specs firmware/iwr6843/l3_ball_track.c tests/test_iwr6843_firmware_ball_track.py
git commit -m "docs(iwr6843): gap-tolerant ball search results and default decision

Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
```
