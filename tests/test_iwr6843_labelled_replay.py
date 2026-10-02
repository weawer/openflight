"""The firmware's tracks against the hand labels committed beside the recordings.

Every ``*.l3dump`` in ``tests/radar/recordings`` that has a reviewed
``<dump>.labels.json`` is replayed with its manifest configuration. The
firmware must cover the labelled frames within the file's tolerances, track
nothing on an object labelled empty, and not score below the committed
baseline. To accept a deliberate change run
``uv run python scripts/analysis/fit_constants.py --update-baseline``.
"""

from __future__ import annotations

import functools
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import pytest

from openflight.iwr6843 import firmware_host as fw, firmware_replay as fr, label_scoring as ls
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.monitor import SELF_TRIGGER_TEE_LEAD_BINS
from openflight.iwr6843.self_trigger import FIRMWARE_TRIGGER_DEFAULT_SNR, TEE_BAND_DEFAULT_BINS

needs_compiler = pytest.mark.skipif(
    fw.host_compiler() is None, reason="no C compiler for the firmware modules"
)

_RECORDINGS = ls.reviewed_recordings(fr.RECORDINGS_DIR) if fr.RECORDINGS_DIR.exists() else []
# 2026-09 home sessions where the golfer's body fills the bins just short of
# the ball, where the club's approach passes (see its manifest). Kept apart so
# the main folder's baselines and counts are untouched.
GOLFER_DIR = fr.RECORDINGS_DIR / "golfer_2026-09"
_BASELINE = ls.load_baseline(fr.RECORDINGS_DIR) if fr.RECORDINGS_DIR.exists() else {}


@needs_compiler
@pytest.mark.parametrize(
    ("path", "config", "labels"), _RECORDINGS, ids=[r[0].name for r in _RECORDINGS]
)
def test_firmware_tracks_match_the_labels(path, config, labels):
    result = fr.replay_dump(path.read_bytes(), config)
    scores = ls.score_labels(labels, result)
    failures = ls.check_against_baseline(path.name, labels, scores, _BASELINE)
    assert failures == [], f"{path.name}: {failures}; {scores}"


# The share of labelled swings the kiosk's self-trigger must fire on inside
# the launch window. 2026-09-30: aimed at the ball's range (less the lead) it
# fired within two frames of launch on 19 of 34, because the club's radar
# range at impact is 3-12 bins short of the ball's (median 7.4).
KIOSK_TRIGGER_MIN_SHARE = 0.85
# The window is lopsided on purpose. A fire up to EARLY_FRAMES before launch
# still records the launch and most of the flight in the 16 post frames
# (L3_DEFAULT_POST_FRAMES); a late fire loses ball frames. In the early
# 2026-08-09 captures the club is invisible (to the labeller too) for the 3-5
# frames before launch, so no rule that watches the club lands closer.
EARLY_FRAMES = 4
LATE_FRAMES = 2


def _kiosk_config(config: fr.ReplayConfig, ball_bin: int) -> fr.ReplayConfig:
    """What the kiosk sends for a ball at ``ball_bin``: ``triggerCfg`` aimed
    SELF_TRIGGER_TEE_LEAD_BINS short of it at the default snr, the default tee
    band, no locked-ball destination and no manifest overrides; nothing forces
    impact, so the self-trigger alone decides when the capture freezes."""
    return replace(
        config,
        tee_bin=ball_bin - SELF_TRIGGER_TEE_LEAD_BINS,
        dest_bin=None,
        snr=FIRMWARE_TRIGGER_DEFAULT_SNR,
        band_bins=TEE_BAND_DEFAULT_BINS,
        post_from_frame=None,
        overrides={},
    )


@dataclass(frozen=True)
class _KioskSwing:
    """One labelled swing replayed at the kiosk's settings."""

    name: str
    offset: int | None  # fired frame - labelled launch frame; None: never fired
    by_leave: bool  # the ball-leave fallback fired it
    labelled_mps: float  # the labelled ball's radial speed at launch
    pre_scored_max: int  # most bins scored on one armed pre-impact frame
    post_scored_max: int  # most bins scored on one post-impact frame
    launch_mps: float | None  # the replayed launch's radial speed; None: no launch
    pre_snapshots_max: int = 0  # most club snapshots queued on one pre-impact frame


def _labelled_radial_mps(raw: bytes, labels) -> float:
    """Median step of the first six labelled ball points, in m/s at the dump's
    frame period: robust to one misclicked point (20260809_114539's first
    point is 4 bins off its flight)."""
    period_s = float(np.median(np.diff(fr.frame_timestamps_us(parse_dump(raw)[0])))) * 1e-6
    points = labels.ball[:6]
    steps = [
        (b.range_bin - a.range_bin) / (b.frame - a.frame)
        for a, b in zip(points, points[1:])
        if b.frame > a.frame
    ]
    return float(np.median(steps)) * fr.RANGE_SPAN_M / fr.DEFAULT_FFT_SIZE / period_s


@functools.cache
def _reviewed(directory: Path) -> tuple:
    """A folder's reviewed recordings; the main folder's are loaded once at import."""
    if directory == fr.RECORDINGS_DIR:
        return tuple(_RECORDINGS)
    return tuple(ls.reviewed_recordings(directory)) if directory.exists() else ()


@functools.cache
def _kiosk_swings(directory: Path = fr.RECORDINGS_DIR) -> tuple[_KioskSwing, ...]:
    """Every labelled swing in ``directory`` replayed at the kiosk's settings.
    The ball's bin is its first labelled point (the tape a correctly measured
    tee gives), the launch frame that point's frame."""
    judged = []
    for path, config, labels in _reviewed(directory):
        if not labels.ball:
            continue
        raw = path.read_bytes()
        launch = labels.ball[0]
        result = fr.replay_dump(raw, _kiosk_config(config, int(round(launch.range_bin))))
        fired = result.fired_frame
        judged.append(
            _KioskSwing(
                name=path.name,
                offset=None if fired is None else fired - launch.frame,
                by_leave=fired is not None and result.leave_frame == fired,
                labelled_mps=_labelled_radial_mps(raw, labels),
                # The frame that fires is scored as a pre-impact frame.
                pre_scored_max=max(
                    (f.scored_bins for f in result.frames if fired is None or f.frame <= fired),
                    default=0,
                ),
                post_scored_max=max(
                    (f.scored_bins for f in result.frames if fired is not None and f.frame > fired),
                    default=0,
                ),
                pre_snapshots_max=max(
                    (f.angle_snapshots for f in result.frames if fired is None or f.frame <= fired),
                    default=0,
                ),
                # Radial, as the labels are range only: the 3D speed also
                # carries the angle fit (20260824_111428: 60 m/s over three
                # points at confidence 0.02, radial 50 against 47 labelled).
                launch_mps=None if result.launch is None else result.launch.radial_speed_mps,
            )
        )
    return tuple(judged)


def _kiosk_fire_offsets(directory: Path = fr.RECORDINGS_DIR) -> tuple[tuple[str, int | None], ...]:
    return tuple((swing.name, swing.offset) for swing in _kiosk_swings(directory))


@needs_compiler
def test_the_kiosk_self_trigger_fires_at_the_labelled_launch():
    judged = _kiosk_fire_offsets()
    assert len(judged) >= 30
    near = [
        name
        for name, offset in judged
        if offset is not None and -EARLY_FRAMES <= offset <= LATE_FRAMES
    ]
    missed = [(name, offset) for name, offset in judged if name not in near]
    assert len(near) >= KIOSK_TRIGGER_MIN_SHARE * len(judged), (
        f"fired from {EARLY_FRAMES} frames before to {LATE_FRAMES} after launch on "
        f"{len(near)}/{len(judged)}; missed (fire - launch frames): {missed}"
    )


# The ball tracker looks for the ball within 8 bins of its rest bin after the
# fire; at the labelled launch rates (1.5-3.6 bins a frame, median 2.8) three
# frames is as late as a fire can come and still hand it the ball.
LATEST_FIRE_FRAMES = 3
# Re-baselined 2026-09-30 for the scan plan (l3_scan.h), which scores 27-29
# bins before impact and 16 after so the board keeps up with its 3 ms frames:
# scoring the whole window every frame fired all 34, none late, but only in
# the replay -- on the board it starved the CLI and fired nothing. Speeding up
# the per-bin residual can widen the spans back and tighten these again.
# 2026-10-01: the club-in floor rose 10 -> 17 m/s so the downrange crossing
# into the top of a backswing stops firing; 20260824_120840 (its club in
# under 15 m/s on this replay) no longer fires.
MAX_UNFIRED = 3  # was 2
MAX_TOO_LATE = 1  # was 0


@needs_compiler
def test_the_labelled_swings_fire_at_the_kiosk_settings_before_the_ball_is_lost():
    """When the club rules miss (the club unseen before launch, as in the early
    2026-08-09 captures), the ball leaving still fires, late but in time."""
    judged = _kiosk_fire_offsets()
    unfired = [name for name, offset in judged if offset is None]
    too_late = [
        (name, offset)
        for name, offset in judged
        if offset is not None and offset > LATEST_FIRE_FRAMES
    ]
    assert len(unfired) <= MAX_UNFIRED and len(too_late) <= MAX_TOO_LATE, (
        f"never fired: {unfired}; fired more than {LATEST_FIRE_FRAMES} frames after "
        f"launch: {too_late}"
    )


# A launch this far off the labelled radial speed is not the ball. The club's
# follow-through steps out at up to ~34 m/s on the labels; the ball leaves at
# 40-56 m/s.
LAUNCH_TOLERANCE = 0.25
# Re-baselined with the scan plan (see MAX_UNFIRED): the whole window gave 34
# good launches and none wrong; 16 bins following the ball after impact give
# 25, with 20260824_120840 taking its club (19 m/s for 45.5).
MIN_GOOD_LAUNCHES = 25  # was 34
MAX_WRONG_LAUNCHES = 1  # was 0


def _launch_verdict(swing: _KioskSwing) -> str:
    if swing.launch_mps is None:
        return "none"
    off = abs(swing.launch_mps - swing.labelled_mps)
    return "good" if off <= LAUNCH_TOLERANCE * swing.labelled_mps else "wrong"


@needs_compiler
def test_no_labelled_swing_reports_the_club_as_the_ball_at_the_kiosk_settings():
    """A slow return confirmed as the ball is the club's follow-through: at the
    kiosk's settings 20260809_110338 and 20260824_111428 reported ~21 m/s."""
    wrong = [
        (s.name, round(s.labelled_mps, 1), round(s.launch_mps, 1))
        for s in _kiosk_swings()
        if _launch_verdict(s) == "wrong"
    ]
    assert len(wrong) <= MAX_WRONG_LAUNCHES, (
        f"launch off the labelled speed (labelled, reported): {wrong}"
    )


@needs_compiler
def test_the_labelled_swings_report_their_launch_at_the_kiosk_settings():
    swings = _kiosk_swings()
    good = [s.name for s in swings if _launch_verdict(s) == "good"]
    assert len(good) >= MIN_GOOD_LAUNCHES, (
        f"{len(good)}/{len(swings)} launches within {LAUNCH_TOLERANCE:.0%} of the labels; "
        f"not good: {[(s.name, _launch_verdict(s)) for s in swings if s.name not in good]}"
    )


@needs_compiler
def test_a_swing_the_ball_leaving_fired_gets_its_launch():
    """After the fallback's late fire the ball is already 4-6 bins out and too
    smeared (confidence 0.0-0.17) for the tracker to start on it, so the club's
    follow-through was taken instead. The fallback's own two points are the
    ball: the tracker starts from them."""
    rescued = [s for s in _kiosk_swings() if s.by_leave]
    assert len(rescued) >= 5
    bad = [
        (s.name, _launch_verdict(s), round(s.labelled_mps, 1), s.launch_mps)
        for s in rescued
        if _launch_verdict(s) != "good"
    ]
    assert bad == [], f"fallback-fired swings without a good launch: {bad}"


# The board scores a range bin in ~73 us (triggerLog perf, 2026-09-30) and has
# 3 ms a frame, which must also leave the CLI and the trigger notices time:
# the detect task outranks them, and the tee band's whole-window scoring (the
# trigger region plus all 53 bins, ~5.1 ms) starved them the moment the
# trigger was armed, so the board answered nothing and fired nothing.
PRE_IMPACT_BIN_BUDGET = 29  # ~2.1 ms: 27 on a swing frame, 29 with an idle frame's map chunk
POST_IMPACT_BIN_BUDGET = 16  # ~1.2 ms


@needs_compiler
def test_every_armed_frame_scores_within_the_boards_budget():
    over = [
        (s.name, s.pre_scored_max, s.post_scored_max)
        for s in _kiosk_swings()
        if s.pre_scored_max > PRE_IMPACT_BIN_BUDGET or s.post_scored_max > POST_IMPACT_BIN_BUDGET
    ]
    assert over == [], (
        f"bins scored per frame over {PRE_IMPACT_BIN_BUDGET} before impact or "
        f"{POST_IMPACT_BIN_BUDGET} after (dump, pre max, post max): {over[:5]}"
    )


# The club's angle estimate (~1.6 ms before the steering table) left the
# decision path: a pre-impact frame takes at most one channel snapshot (the
# associated target's) and queues it for the angle task (l3_angle_queue.h).
PRE_IMPACT_SNAPSHOT_BUDGET = 1


@needs_compiler
def test_a_pre_impact_frame_queues_at_most_one_club_snapshot():
    over = [
        (s.name, s.pre_snapshots_max)
        for s in _kiosk_swings()
        if s.pre_snapshots_max > PRE_IMPACT_SNAPSHOT_BUDGET
    ]
    assert over == [], f"club snapshots per pre-impact frame over the budget: {over[:5]}"
    assert any(s.pre_snapshots_max == 1 for s in _kiosk_swings()), "a club is tracked somewhere"


def test_the_replays_pre_impact_path_queues_the_club_angle_and_never_estimates_it():
    """The replay mirrors the board: no angle estimate on the decision path."""
    import inspect  # pylint: disable=import-outside-toplevel

    # replay_dump is the pre-impact loop; post-impact frames (whose ball
    # angles stay inline, as on the board) are _replay_post_frame's.
    source = inspect.getsource(fr.replay_dump)
    assert "l3_angle_queue_push(" in source
    assert "_estimate_angles(" not in source


# --- the golfer in the approach (golfer_2026-09) -------------------------------
#
# Found 2026-10-01: at the kiosk's settings the club was lost as it passed the
# golfer. The trigger's floor (the median of the bins short of the ball) read
# the body (~80 dB) over the club's approach (71-78 dB against 65 dB of air),
# and once extracted the club lost its track to the body's far stronger
# return (snr 100-870 against 10-96). 21 of 37 swings fired within the window
# and 10 never fired. The gates are the main folder's.


def _golfer_swings() -> tuple[_KioskSwing, ...]:
    swings = _kiosk_swings(GOLFER_DIR)
    if not swings:
        pytest.skip(f"no labelled swings under {GOLFER_DIR}")
    return swings


@needs_compiler
def test_the_kiosk_self_trigger_fires_at_launch_with_the_golfer_in_the_approach():
    swings = _golfer_swings()
    near = [
        s.name for s in swings if s.offset is not None and -EARLY_FRAMES <= s.offset <= LATE_FRAMES
    ]
    missed = [(s.name, s.offset) for s in swings if s.name not in near]
    assert len(near) >= KIOSK_TRIGGER_MIN_SHARE * len(swings), (
        f"fired from {EARLY_FRAMES} frames before to {LATE_FRAMES} after launch on "
        f"{len(near)}/{len(swings)}; missed (fire - launch frames): {missed}"
    )


@needs_compiler
def test_the_swings_past_the_golfer_fire_before_the_ball_is_lost():
    swings = _golfer_swings()
    unfired = [s.name for s in swings if s.offset is None]
    too_late = [
        (s.name, s.offset) for s in swings if s.offset is not None and s.offset > LATEST_FIRE_FRAMES
    ]
    assert len(unfired) <= MAX_UNFIRED and len(too_late) <= MAX_TOO_LATE, (
        f"never fired: {unfired}; fired more than {LATEST_FIRE_FRAMES} frames after "
        f"launch: {too_late}"
    )


@needs_compiler
def test_the_swings_past_the_golfer_report_the_ball_not_the_club():
    swings = _golfer_swings()
    wrong = [
        (s.name, round(s.labelled_mps, 1), round(s.launch_mps, 1))
        for s in swings
        if _launch_verdict(s) == "wrong"
    ]
    assert len(wrong) <= MAX_WRONG_LAUNCHES, f"launch off the labelled speed: {wrong}"


@needs_compiler
def test_the_swings_past_the_golfer_score_within_the_boards_budget():
    over = [
        (s.name, s.pre_scored_max, s.post_scored_max)
        for s in _golfer_swings()
        if s.pre_scored_max > PRE_IMPACT_BIN_BUDGET or s.post_scored_max > POST_IMPACT_BIN_BUDGET
    ]
    assert over == [], f"bins scored per frame over the budget (dump, pre, post): {over[:5]}"


@needs_compiler
def test_nothing_fires_on_a_capture_labelled_empty():
    """No swing, only the golfer at address: the kiosk must not fire."""
    recordings = _reviewed(GOLFER_DIR)
    swings = [labels for _, _, labels in recordings if labels.ball]
    empty = [
        (path, config) for path, config, labels in recordings if not labels.ball and not labels.club
    ]
    if not empty or not swings:
        pytest.skip(f"no empty and labelled captures under {GOLFER_DIR}")
    # Where the ball sat on the session's swings.
    ball_bin = int(round(float(np.median([labels.ball[0].range_bin for labels in swings]))))
    fired = []
    for path, config in empty:
        result = fr.replay_dump(path.read_bytes(), _kiosk_config(config, ball_bin))
        if result.fired_frame is not None:
            fired.append((path.name, result.fired_frame))
    assert fired == [], f"fired on captures labelled empty: {fired}"
