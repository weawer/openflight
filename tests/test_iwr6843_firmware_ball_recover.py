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
    got, res = recover(lib, h, hyp_of([(k, ball_bin(k)) for k in (1, 8)]), cap=4)
    assert len(got) == 4 and got[-1][0] == 8
    assert [f for f, _ in got] == [5, 6, 7, 8]
    assert (res.recovered, res.firstFrame, res.mask) == (3, 5, 0b111)  # only what was kept


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


def test_marking_the_club_flags_its_index_on_the_newest_frame(lib):
    """The club is followed after the ball's update, so its claim reaches the
    history afterwards: l3_ball_history_mark_club flags it on the frame just pushed."""
    h = history(lib, {1: [50.0, 51.0, 52.0], 2: [53.0, 54.0, 55.0]})
    lib.l3_ball_history_mark_club(ctypes.byref(h), 2, 1)
    newest = lib.l3_ball_history_at(ctypes.byref(h), 1).contents
    older = lib.l3_ball_history_at(ctypes.byref(h), 0).contents
    assert newest.clubMask == 0b10
    assert older.clubMask == 0


def test_marking_a_stale_frame_or_an_absent_index_does_nothing(lib):
    h = history(lib, {1: [50.0, 51.0], 2: [53.0, 54.0]})
    lib.l3_ball_history_mark_club(ctypes.byref(h), 1, 0)  # not the newest frame
    lib.l3_ball_history_mark_club(ctypes.byref(h), 3, 0)  # not pushed yet
    lib.l3_ball_history_mark_club(ctypes.byref(h), 2, 2)  # past the frame's count
    lib.l3_ball_history_mark_club(ctypes.byref(h), 2, NO_CLAIM)
    assert [lib.l3_ball_history_at(ctypes.byref(h), i).contents.clubMask for i in (0, 1)] == [0, 0]


def test_marking_an_empty_history_does_nothing(lib):
    h = fw.BallHistory()
    lib.l3_ball_history_reset(ctypes.byref(h))
    lib.l3_ball_history_mark_club(ctypes.byref(h), 0, 0)
    assert bytes(h) == bytes(fw.BallHistory())


def test_a_marked_club_return_is_not_recovered(lib):
    frames = {k: [ball_bin(k)] for k in range(1, 7)}
    h = fw.BallHistory()
    lib.l3_ball_history_reset(ctypes.byref(h))
    for k in range(1, 7):
        ts = k * FRAME_US
        arr = (fw.TargetObs * 1)(obs(k, ts, frames[k][0], 1000.0, SPEED))
        lib.l3_ball_history_push(ctypes.byref(h), arr, 1, k, ts, NO_CLAIM)
        if k == 3:
            lib.l3_ball_history_mark_club(ctypes.byref(h), 3, 0)
    got, res = recover(lib, h, hyp_of([(k, ball_bin(k)) for k in (1, 2, 4, 5, 6)]))
    assert 3 not in [f for f, _ in got] and res.recovered == 0
