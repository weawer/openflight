"""Confirm mode (``trackCfg confirm``, firmware/iwr6843/l3_confirm.c) replayed
on every recording with a verdict: the self-trigger tells the host only once
the ball's flight confirms a fire.

At the kiosk's snr 1 the club rules and the ball-leave fallback also fire on
a backswing, the golfer shifting or an empty lane: a brief real mover chained
through noise points to the ball. Each of those fires blinded the radars for
~7.8 s on 2026-10-03, losing the downswing that followed. None of them
launches a ball, so with confirm mode none may reach the host, while the
labelled real swings must still confirm.
"""

from __future__ import annotations

import csv
import functools
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from test_iwr6843_labelled_replay import GOLFER_DIR, _kiosk_config, _reviewed

from openflight.iwr6843 import firmware_host as fw, firmware_replay as fr
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.self_trigger import FIRMWARE_TRIGGER_DEFAULT_SNR, TEE_BAND_DEFAULT_BINS

needs_compiler = pytest.mark.skipif(
    fw.host_compiler() is None, reason="no C compiler for the firmware modules"
)

ROOT = Path(__file__).resolve().parents[1]
# 2026-10-03 kiosk session (2 ms frames, tee at bin 42): 18 self-trigger fires
# during backswing-only tests and real swings; the OPS accepted these five.
KIOSK_DIR = ROOT / "openflight_sessions" / "backswing_2ms_20261003"
KIOSK_TEE_BIN = 42
KIOSK_OPS_ACCEPTED = {2, 4, 10, 15, 16}
# 2026-10-04 TrackMan comparison (2 ms frames, tee at bin 36).
TRACKMAN_DIR = ROOT / "tests" / "radar" / "trackman_20261004_095102"
TRACKMAN_TEE_BIN = 36
# Fires TrackMan saw no shot for: the takeaway, or a shot TrackMan read but
# whose dump holds no ball.
TRACKMAN_FALSE = {"early_trigger", "matched_no_ball"}

# Of the labelled swings the kiosk's self-trigger fires on, the share whose
# ball flight must confirm. 2026-10-06: 52 of 58. The six lost are fires 4
# frames before launch or 2 after, where the ball tracker found no
# candidate in the band's lee (a ball-tracker timing limit, not the rule).
CONFIRM_MIN_SHARE = 0.85


def _confirm(config: fr.ReplayConfig) -> fr.ReplayConfig:
    return replace(config, confirm=True)


def _kiosk_at(tee_bin: int) -> fr.ReplayConfig:
    return fr.ReplayConfig(
        tee_bin=tee_bin, snr=FIRMWARE_TRIGGER_DEFAULT_SNR, band_bins=TEE_BAND_DEFAULT_BINS
    )


@functools.cache
def _labelled_swings() -> tuple[tuple[str, fr.ReplayResult, fr.ReplayResult], ...]:
    """Every labelled swing with a ball, replayed at the kiosk's settings with
    confirm mode off and on: (name, off, on)."""
    swings = []
    for directory in (fr.RECORDINGS_DIR, GOLFER_DIR):
        for path, config, labels in _reviewed(directory):
            if not labels.ball:
                continue
            kiosk = _kiosk_config(config, int(round(labels.ball[0].range_bin)))
            raw = path.read_bytes()
            swings.append(
                (path.name, fr.replay_dump(raw, kiosk), fr.replay_dump(raw, _confirm(kiosk)))
            )
    return tuple(swings)


def _empty_captures() -> list[tuple[str, fr.ReplayConfig, Path]]:
    """The golfer folder's captures labelled with neither club nor ball, at
    the kiosk's settings with the ball at the folder's median launch bin."""
    reviewed = _reviewed(GOLFER_DIR)
    launches = [labels.ball[0].range_bin for _p, _c, labels in reviewed if labels.ball]
    if not launches:
        return []
    ball_bin = int(round(float(np.median(launches))))
    return [
        (path.name, _kiosk_config(config, ball_bin), path)
        for path, config, labels in reviewed
        if not labels.ball and not labels.club
    ]


def _kiosk_dumps(accepted: bool) -> list[Path]:
    if not KIOSK_DIR.exists():
        return []
    return [
        path
        for path in sorted(KIOSK_DIR.glob("*.l3dump"))
        if (int(path.stem.rsplit("_", 1)[-1]) in KIOSK_OPS_ACCEPTED) == accepted
    ]


def _trackman_rows() -> list[dict]:
    table = TRACKMAN_DIR / "trackman_openflight_aligned.csv"
    if not table.exists():
        return []
    with table.open(encoding="utf-8", newline="") as handle:
        return [row for row in csv.DictReader(handle) if row["iwr_dump_path"]]


def _trackman_dump(row: dict) -> Path:
    return TRACKMAN_DIR / "iwr6843" / Path(row["iwr_dump_path"]).name


def _trackman(false: bool) -> list[dict]:
    return [row for row in _trackman_rows() if (row["match_status"] in TRACKMAN_FALSE) == false]


def _never_confirmed(result: fr.ReplayResult) -> bool:
    """Nothing reached the host: no fire, or a fire the flight rejected."""
    return result.confirm_frame is None and result.confirm_verdict in ("idle", "rejected")


@needs_compiler
def test_confirm_mode_fires_the_same_candidates_as_before():
    """Confirmation only delays the host's notice: the ring freezes on the
    same frame and the shot's tracks are the same."""
    swings = _labelled_swings()
    assert swings
    for name, off, on in swings:
        assert on.fired_frame == off.fired_frame, name
        assert on.launch == off.launch, name
        assert off.confirm_verdict == "idle", f"{name}: confirm is off by default"


@needs_compiler
def test_most_labelled_swings_confirm_within_the_window():
    fired = [(name, on) for name, _off, on in _labelled_swings() if on.fired_frame is not None]
    confirmed = [(name, on) for name, on in fired if on.confirm_verdict == "confirmed"]

    assert len(confirmed) >= CONFIRM_MIN_SHARE * len(fired), (
        f"{len(confirmed)} of {len(fired)} confirmed; lost: "
        + ", ".join(
            f"{name} {on.confirm_status}" for name, on in fired if (name, on) not in confirmed
        )
    )
    for name, on in confirmed:
        assert on.confirm_frame > on.fired_frame, name
        assert "why=flight" in on.confirm_status, name


@needs_compiler
def test_every_labelled_candidate_gets_a_verdict():
    """The host must hear Triggered or Rejected: a candidate left pending
    would keep the ring frozen until the host's own timeout."""
    for name, _off, on in _labelled_swings():
        if on.fired_frame is not None:
            assert on.confirm_verdict in ("confirmed", "rejected"), f"{name}: {on.confirm_status}"


@needs_compiler
@pytest.mark.parametrize(
    ("name", "config", "path"), _empty_captures(), ids=[c[0] for c in _empty_captures()]
)
def test_an_empty_capture_never_confirms(name, config, path):
    del name
    assert _never_confirmed(fr.replay_dump(path.read_bytes(), _confirm(config)))


@needs_compiler
@pytest.mark.parametrize("path", _kiosk_dumps(accepted=False), ids=lambda p: p.stem)
def test_a_kiosk_fire_the_ops_rejected_never_confirms(path):
    """2026-10-03: backswing-only tests and empty fires the OPS found no ball in."""
    assert _never_confirmed(fr.replay_dump(path.read_bytes(), _confirm(_kiosk_at(KIOSK_TEE_BIN))))


@needs_compiler
def test_kiosk_fires_the_ops_rejected_still_fire_candidates():
    """The previous test means something: several of those dumps do fire."""
    fired = [
        path
        for path in _kiosk_dumps(accepted=False)
        if fr.replay_dump(path.read_bytes(), _kiosk_at(KIOSK_TEE_BIN)).fired_frame is not None
    ]
    assert len(fired) >= 5


@needs_compiler
@pytest.mark.parametrize(
    "row", _trackman(false=True), ids=lambda r: f"{r['tm_sequence']}-{r['match_status']}"
)
def test_a_trackman_fire_with_no_ball_never_confirms(row):
    raw = _trackman_dump(row).read_bytes()
    assert _never_confirmed(fr.replay_dump(raw, _confirm(_kiosk_at(TRACKMAN_TEE_BIN))))


@needs_compiler
def test_trackman_shots_whose_dump_holds_the_launch_confirm():
    """2026-10-06: 47, 56 and 57 confirm. The other real shots fired before
    the dump's window held the launch (impact_before_window) or never fired."""
    confirmed = {
        row["tm_sequence"]
        for row in _trackman(false=False)
        if fr.replay_dump(
            _trackman_dump(row).read_bytes(), _confirm(_kiosk_at(TRACKMAN_TEE_BIN))
        ).confirm_verdict
        == "confirmed"
    }
    assert {"47", "56", "57"} <= confirmed


# --- a backswing stand-in on white noise -------------------------------------
#
# The recordings are 50-100 ms long, too short for a fire that builds up over
# seconds of idle. White noise at the kiosk's snr with one brief real mover
# (two frames, a bin apart, coherent Doppler) reproduces the field's
# signature: the club track acquires the mover, then walks through noise
# points to the ball and fires (2026-10-06: 7 of 60 seeds).

_SIM_FRAMES = 130
_SIM_MOVER_FRAME = 80
_SIM_PERIOD_US = 2000
_SIM_SEEDS = range(60)


def _simulated_backswing(seed: int, template: dict) -> tuple[dict, np.ndarray]:
    rng = np.random.default_rng(1000 + seed)
    shape = (_SIM_FRAMES, 36, 4, 53)
    cube = ((rng.standard_normal(shape) + 1j * rng.standard_normal(shape)) * 100).astype(
        np.complex64
    )
    loops = np.arange(36) // 3
    for k, local_bin in enumerate((10, 11)):
        phase = np.exp(1j * (1.2 * loops + rng.uniform(0.0, 2.0 * np.pi)))
        cube[_SIM_MOVER_FRAME + k, :, :, local_bin] += (400.0 * phase)[:, None]
    meta = dict(template)
    meta.pop("clutter_map", None)
    meta.pop("retention", None)
    meta.update(
        n_frames=_SIM_FRAMES,
        range_bin_starts=(20,) * _SIM_FRAMES,
        range_bin_counts=(53,) * _SIM_FRAMES,
        frame_time_offsets_us=tuple(i * _SIM_PERIOD_US for i in range(_SIM_FRAMES)),
        frame_period_us=_SIM_PERIOD_US,
        trigger_frame=0,
    )
    return meta, cube


@needs_compiler
@pytest.mark.skipif(not KIOSK_DIR.exists(), reason="needs the 2026-10-03 kiosk dumps")
def test_a_simulated_backswing_fires_candidates_but_never_confirms(monkeypatch):
    template, _cube = parse_dump(_kiosk_dumps(accepted=False)[0].read_bytes())
    config = _confirm(_kiosk_at(KIOSK_TEE_BIN))
    fired = 0
    for seed in _SIM_SEEDS:
        meta, cube = _simulated_backswing(seed, template)
        monkeypatch.setattr(fr, "parse_dump", lambda _raw, m=meta, c=cube: (m, c))
        result = fr.replay_dump(b"", config)
        fired += result.fired_frame is not None
        assert _never_confirmed(result), f"seed {seed}: {result.confirm_status}"
    assert fired >= 3, "the stand-in no longer fires, so it tests nothing"
