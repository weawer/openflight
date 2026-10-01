"""The club's pending angles, firmware/iwr6843/l3_angle_queue.c.

The club angle (~1.6 ms on the R4F before the steering table) was estimated
on the detect task every frame the club track took a point, though the fire
decision uses range only. It now leaves the decision path: the detect task
queues the point's channel snapshot keyed by the point's timestamp; a
low-priority angle task estimates it in spare time and sets the angles on
the point with that timestamp; a fire frame drains the queue itself (after
the freeze request) so the shot freezes a trajectory with every angle.

- FIFO order; a full queue drops its oldest (counted): the delivery fit
  reads the newest points
- apply sets the angles on the point with the job's timestamp, not the
  newest; a point no longer in the track (reset, rolled off) is stale and
  touches nothing; a snapshot the estimator refuses is counted as failed
- the queue path gives exactly the immediate estimate's angles
"""

from __future__ import annotations

import ctypes

import numpy as np
import pytest

from openflight.iwr6843 import firmware_host as fw
from tests.test_iwr6843_firmware_club_track import FRAME_US, Tracker, target

DEPTH = fw.L3_ANGLE_QUEUE_DEPTH


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


@pytest.fixture()
def cal(lib):
    out = fw.RadarCal()
    lib.l3_cal_identity(ctypes.byref(out), fw.CAL_MAX_VIRTUAL)
    return out


def queue(lib) -> fw.AngleQueue:
    q = fw.AngleQueue()
    lib.l3_angle_queue_init(ctypes.byref(q))
    return q


def snapshot(lib, seed: int, ntx: int = 3) -> fw.AngleSnapshot:
    """A plane wave from a random direction on every channel, estimable."""
    rng = np.random.default_rng(seed)
    snap = fw.AngleSnapshot()
    lib.l3_angle_snapshot_init(ctypes.byref(snap), ntx, 4)
    phase_step = rng.uniform(-1.0, 1.0)
    for k in range(ntx * 4):
        value = np.exp(1j * phase_step * k) * 100.0
        snap.channel[k] = fw.Cpx(float(value.real), float(value.imag))
    snap.lag1PhaseRad = 0.3
    snap.radialVelocityMps = 20.0
    snap.chirpPeriodS = 45e-6
    return snap


def push(lib, q, timestamp_us: int, snap: fw.AngleSnapshot) -> int:
    return lib.l3_angle_queue_push(ctypes.byref(q), timestamp_us, ctypes.byref(snap))


def pop(lib, q) -> fw.AngleJob | None:
    job = fw.AngleJob()
    return job if lib.l3_angle_queue_pop(ctypes.byref(q), ctypes.byref(job)) else None


def track_of(lib, frames: int) -> Tracker:
    tr = Tracker(lib)
    for frame in range(1, frames + 1):
        assert tr.update(frame, [target(frame, 20.0 + frame)])
    return tr


# --- the queue ----------------------------------------------------------------


def test_jobs_come_out_in_the_order_they_went_in(lib):
    q = queue(lib)
    for k in range(3):
        assert push(lib, q, 1000 * (k + 1), snapshot(lib, k)) == 1
    assert lib.l3_angle_queue_pending(ctypes.byref(q)) == 3
    assert [pop(lib, q).timestampUs for _ in range(3)] == [1000, 2000, 3000]
    assert pop(lib, q) is None
    assert q.queued == 3


def test_a_full_queue_drops_its_oldest_job(lib):
    q = queue(lib)
    for k in range(DEPTH):
        assert push(lib, q, k + 1, snapshot(lib, k)) == 1
    assert push(lib, q, 999, snapshot(lib, 99)) == 0, "queued, after dropping the oldest"
    assert q.dropped == 1
    assert lib.l3_angle_queue_pending(ctypes.byref(q)) == DEPTH
    out = [pop(lib, q).timestampUs for _ in range(DEPTH)]
    assert out == [*range(2, DEPTH + 1), 999]


def test_the_queue_holds_every_point_the_delivery_fit_reads():
    """l3_track_delivery reads the newest 8 points: a queue of at least 8
    loses none of them however far the angle task falls behind."""
    assert DEPTH >= 8


# --- applying a job -------------------------------------------------------------


def test_a_job_sets_the_angles_on_the_point_with_its_timestamp(lib, cal):
    q = queue(lib)
    tr = track_of(lib, 5)
    third = tr.points()[2]
    job = fw.AngleJob(timestampUs=third.timestampUs, snapshot=snapshot(lib, 1))
    obs = fw.AngleObs()

    assert (
        lib.l3_angle_queue_apply(
            ctypes.byref(q),
            ctypes.byref(cal),
            ctypes.byref(job),
            ctypes.byref(tr.track),
            ctypes.byref(obs),
        )
        == 1
    )

    points = tr.points()
    assert points[2].anglesValid != 0
    assert points[2].elevationRad == pytest.approx(obs.elevationRad)
    assert all(p.anglesValid == 0 for i, p in enumerate(points) if i != 2), "only that point"
    assert q.done == 1


def test_the_queue_path_gives_exactly_the_immediate_estimate(lib, cal):
    q = queue(lib)
    queued, immediate = track_of(lib, 4), track_of(lib, 4)
    snap = snapshot(lib, 2)
    newest = queued.points()[-1]
    job = fw.AngleJob(timestampUs=newest.timestampUs, snapshot=snap)
    lib.l3_angle_queue_apply(
        ctypes.byref(q),
        ctypes.byref(cal),
        ctypes.byref(job),
        ctypes.byref(queued.track),
        ctypes.byref(fw.AngleObs()),
    )
    obs = fw.AngleObs()
    assert lib.l3_angle_estimate(ctypes.byref(cal), ctypes.byref(snap), ctypes.byref(obs)) == 1
    flags = (fw.ANGLE_AZIMUTH if obs.azimuthValid else 0) | (
        fw.ANGLE_ELEVATION if obs.elevationValid else 0
    )
    lib.l3_track_set_angles(ctypes.byref(immediate.track), obs.azimuthRad, obs.elevationRad, flags, obs.confidence)
    assert bytes(queued.points()[-1]) == bytes(immediate.points()[-1])


def test_a_point_no_longer_in_the_track_is_stale_and_touches_nothing(lib, cal):
    q = queue(lib)
    tr = track_of(lib, 3)
    gone = tr.points()[0].timestampUs
    lib.l3_track_reset(ctypes.byref(tr.track))
    tr.update(10, [target(10, 30.0)])
    before = [bytes(p) for p in tr.points()]
    job = fw.AngleJob(timestampUs=gone, snapshot=snapshot(lib, 3))

    assert (
        lib.l3_angle_queue_apply(
            ctypes.byref(q),
            ctypes.byref(cal),
            ctypes.byref(job),
            ctypes.byref(tr.track),
            ctypes.byref(fw.AngleObs()),
        )
        == -1
    )

    assert [bytes(p) for p in tr.points()] == before
    assert q.stale == 1 and q.done == 0


def test_a_snapshot_the_estimator_refuses_is_counted_as_failed(lib, cal):
    q = queue(lib)
    tr = track_of(lib, 3)
    bad = fw.AngleSnapshot()  # ntx 0: nothing to estimate
    job = fw.AngleJob(timestampUs=tr.points()[1].timestampUs, snapshot=bad)

    assert (
        lib.l3_angle_queue_apply(
            ctypes.byref(q),
            ctypes.byref(cal),
            ctypes.byref(job),
            ctypes.byref(tr.track),
            ctypes.byref(fw.AngleObs()),
        )
        == 0
    )

    assert tr.points()[1].anglesValid == 0
    assert q.failed == 1


def test_find_point_locates_a_timestamp_or_says_it_is_gone(lib):
    tr = track_of(lib, 4)
    index = ctypes.c_uint32()
    assert lib.l3_track_find_point(ctypes.byref(tr.track), 3 * FRAME_US, ctypes.byref(index)) == 1
    assert index.value == 2
    assert lib.l3_track_find_point(ctypes.byref(tr.track), 99 * FRAME_US, ctypes.byref(index)) == 0


def test_the_queue_layout_matches_the_c(lib):
    assert ctypes.sizeof(fw.AngleQueue) == lib.l3_angle_queue_size()
    assert ctypes.sizeof(fw.AngleJob) == lib.l3_angle_job_size()


# --- the angle task's side: peek, estimate outside the lock, take ------------------
#
# The angle task runs below the detect task, so a fire frame can preempt it
# mid-estimate. Were the job popped, the fire's drain would not see it and
# the shot would freeze without that angle. So the task peeks (the job stays
# queued), estimates outside the lock, then finishes: only if its job is
# still the oldest is it taken and applied; else the drain already applied
# it and the task's result is superseded.


def finish(lib, q, job, estimated, obs, tr) -> int:
    return lib.l3_angle_queue_finish(
        ctypes.byref(q), ctypes.byref(job), estimated, ctypes.byref(obs), ctypes.byref(tr.track)
    )


def test_a_peek_leaves_the_job_queued(lib):
    q = queue(lib)
    push(lib, q, 1000, snapshot(lib, 1))
    job = fw.AngleJob()
    assert lib.l3_angle_queue_peek(ctypes.byref(q), ctypes.byref(job)) == 1
    assert job.timestampUs == 1000
    assert lib.l3_angle_queue_pending(ctypes.byref(q)) == 1
    assert lib.l3_angle_queue_peek(ctypes.byref(queue(lib)), ctypes.byref(job)) == 0


def test_finishing_the_oldest_job_takes_it_and_applies_it(lib, cal):
    q = queue(lib)
    tr = track_of(lib, 3)
    stamp = tr.points()[1].timestampUs
    push(lib, q, stamp, snapshot(lib, 2))
    job = fw.AngleJob()
    lib.l3_angle_queue_peek(ctypes.byref(q), ctypes.byref(job))
    obs = fw.AngleObs()
    estimated = lib.l3_angle_estimate(
        ctypes.byref(cal), ctypes.byref(job.snapshot), ctypes.byref(obs)
    )

    assert finish(lib, q, job, estimated, obs, tr) == 1

    assert lib.l3_angle_queue_pending(ctypes.byref(q)) == 0
    assert tr.points()[1].elevationRad == pytest.approx(obs.elevationRad)
    assert q.done == 1


def test_a_job_the_drain_already_applied_is_superseded(lib, cal):
    """The fire frame drained the queue while the task was estimating."""
    q = queue(lib)
    tr = track_of(lib, 3)
    stamp = tr.points()[2].timestampUs
    push(lib, q, stamp, snapshot(lib, 3))
    job = fw.AngleJob()
    lib.l3_angle_queue_peek(ctypes.byref(q), ctypes.byref(job))
    drained = pop(lib, q)
    assert (
        lib.l3_angle_queue_apply(
            ctypes.byref(q),
            ctypes.byref(cal),
            ctypes.byref(drained),
            ctypes.byref(tr.track),
            ctypes.byref(fw.AngleObs()),
        )
        == 1
    )
    after_drain = bytes(tr.points()[2])
    other = fw.AngleObs(elevationRad=0.5, elevationValid=1)

    assert finish(lib, q, job, 1, other, tr) == -2

    assert bytes(tr.points()[2]) == after_drain, "the drain's angles stand"
    assert q.done == 1


def test_a_newer_job_at_the_head_is_not_taken_by_an_older_result(lib):
    q = queue(lib)
    tr = track_of(lib, 3)
    push(lib, q, tr.points()[0].timestampUs, snapshot(lib, 4))
    job = fw.AngleJob()
    lib.l3_angle_queue_peek(ctypes.byref(q), ctypes.byref(job))
    pop(lib, q)
    push(lib, q, tr.points()[1].timestampUs, snapshot(lib, 5))

    assert finish(lib, q, job, 1, fw.AngleObs(), tr) == -2

    assert lib.l3_angle_queue_pending(ctypes.byref(q)) == 1, "the newer job stays"


# --- the board ----------------------------------------------------------------------

import re  # noqa: E402
from pathlib import Path  # noqa: E402

BOARD = Path(__file__).parents[1] / "firmware" / "iwr6843"


def _board() -> str:
    return (BOARD / "l3_dump.c").read_text(encoding="utf-8")


def _board_function(signature: str) -> str:
    source = _board()
    body = source[source.index(signature + "\n{") :]
    return body[: body.index("\n}\n")]


def test_the_decision_path_queues_the_club_angle_instead_of_estimating_it():
    trigger = _board_function("static void l3_considerSelfTrigger(uint32_t slot)")
    assert "l3_angle_estimate(" not in trigger
    snap = trigger.index("l3_channelSnapshot(&frame,")
    assert snap < trigger.index("l3_angleQueuePush(newest.timestampUs, &snapshot);")
    # Never shed when behind: the snapshot is cheap, every point keeps its angle.
    block = trigger[
        trigger.rindex("if (appended && gClubTrack.lastTargetIndex < found", 0, snap) : snap
    ]
    assert "gDetectBehind" not in block


def test_a_fire_freezes_first_then_drains_the_angles_before_the_shot_freezes():
    trigger = _board_function("static void l3_considerSelfTrigger(uint32_t slot)")
    order = [
        "gHwaFreezeRequested = 1U;",
        r'l3_queueNotice("Triggered\n");',
        "l3_angleQueueDrain();",
        "(void)l3_track_delivery(&gClubTrack, 8U, &gDelivery);\n    }",
        "l3_shotObserve(teeBin, fired, impactUs);",
    ]
    positions = [trigger.index(marker) for marker in order]
    assert positions == sorted(positions), order


def test_the_angle_task_peeks_estimates_unlocked_and_finishes_locked():
    task = _board_function("static void l3_angleTask(UArg arg0, UArg arg1)")
    peek = task.index("l3_angle_queue_peek(&gAngleQueue, &job)")
    estimate = task.index("l3_angle_estimate(&gRadarCal, &job.snapshot, &obs)")
    finish = task.index("l3_angle_queue_finish(&gAngleQueue, &job, estimated, &obs, &gClubTrack)")
    assert peek < estimate < finish
    assert task.rindex("Task_disable()", 0, peek) < peek < task.index("Task_restore(", peek)
    assert task.index("Task_restore(", peek) < estimate, "the estimate runs unlocked"
    assert task.rindex("Task_disable()", 0, finish) > estimate


def test_the_angle_task_runs_below_the_cli_on_a_stack_outside_data_ram():
    source = _board()
    assert re.search(r"#define L3_ANGLE_TASK_PRIORITY\s+2U", source)
    assert re.search(r"#define L3_CLI_TASK_PRIORITY\s+3", source)
    assert "static uint8_t gAngleTaskStack[L3_ANGLE_TASK_STACK_BYTES] L3_HSRAM_DIAG;" in source
    assert "static l3_angle_queue_t gAngleQueue L3_HSRAM_DIAG;" in source
    assert "Task_create(l3_angleTask, &taskParams, NULL);" in source


def test_the_board_builds_the_queue_and_reports_it():
    assert "l3_angle_queue.c" in (BOARD / "makefile").read_text(encoding="utf-8")
    assert "l3_angle_queue.c" in fw.HOST_SOURCES
    assert "angles queued=%u done=%u stale=%u failed=%u dropped=%u pending=%u" in _board()


def test_a_drained_angle_carries_its_confidence_onto_the_point(lib, cal):
    """l3_angle_queue_apply passes obs->confidence to the point."""
    q = queue(lib)
    tr = track_of(lib, 4)
    newest = tr.points()[-1]
    job = fw.AngleJob(timestampUs=newest.timestampUs, snapshot=snapshot(lib, 2))
    obs = fw.AngleObs()
    assert (
        lib.l3_angle_queue_apply(
            ctypes.byref(q), ctypes.byref(cal), ctypes.byref(job), ctypes.byref(tr.track), ctypes.byref(obs)
        )
        == 1
    )
    assert obs.confidence > 0.0
    assert tr.points()[-1].angleConfidence == pytest.approx(obs.confidence)
