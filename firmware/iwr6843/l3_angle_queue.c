/* IWR6843 club angles off the decision path. See l3_angle_queue.h. */
#include "l3_angle_queue.h"

#include <stddef.h>
#include <string.h>

#include "l3_observation.h"

uint32_t l3_angle_queue_size(void)
{
    return (uint32_t)sizeof(l3_angle_queue_t);
}

uint32_t l3_angle_job_size(void)
{
    return (uint32_t)sizeof(l3_angle_job_t);
}

void l3_angle_queue_init(l3_angle_queue_t *queue)
{
    memset(queue, 0, sizeof(*queue));
}

int32_t l3_angle_queue_push(l3_angle_queue_t *queue, uint32_t timestampUs,
                            const l3_angle_snapshot_t *snapshot)
{
    int32_t room = 1;
    l3_angle_job_t *job;

    if (queue->count == L3_ANGLE_QUEUE_DEPTH) {
        /* Full: the oldest goes; the delivery fit reads the newest points. */
        queue->head = (queue->head + 1U) % L3_ANGLE_QUEUE_DEPTH;
        queue->count--;
        queue->dropped++;
        room = 0;
    }
    job = &queue->jobs[(queue->head + queue->count) % L3_ANGLE_QUEUE_DEPTH];
    job->timestampUs = timestampUs;
    job->snapshot = *snapshot;
    queue->count++;
    queue->queued++;
    return room;
}

int32_t l3_angle_queue_pop(l3_angle_queue_t *queue, l3_angle_job_t *out)
{
    if (queue->count == 0U) {
        return 0;
    }
    *out = queue->jobs[queue->head];
    queue->head = (queue->head + 1U) % L3_ANGLE_QUEUE_DEPTH;
    queue->count--;
    return 1;
}

uint32_t l3_angle_queue_pending(const l3_angle_queue_t *queue)
{
    return queue->count;
}

int32_t l3_angle_queue_peek(const l3_angle_queue_t *queue, l3_angle_job_t *out)
{
    if (queue->count == 0U) {
        return 0;
    }
    *out = queue->jobs[queue->head];
    return 1;
}

/* A job's estimate onto its point, counted: 1 applied, 0 the estimator
 * refused it, -1 its point is gone. */
static int32_t l3_angle_queue_record(l3_angle_queue_t *queue, const l3_angle_job_t *job,
                                     int32_t estimated, const l3_angle_obs_t *obs,
                                     l3_club_track_t *track)
{
    uint32_t index;
    uint8_t flags = 0U;

    if (!l3_track_find_point(track, job->timestampUs, &index)) {
        queue->stale++;
        return -1;
    }
    if (!estimated) {
        queue->failed++;
        return 0;
    }
    if (obs->azimuthValid) {
        flags |= L3_OBS_ANGLE_AZIMUTH;
    }
    if (obs->elevationValid) {
        flags |= L3_OBS_ANGLE_ELEVATION;
    }
    (void)l3_track_set_point_angles(track, index, obs->azimuthRad, obs->elevationRad, flags,
                                    obs->confidence);
    queue->done++;
    return 1;
}

int32_t l3_angle_queue_apply(l3_angle_queue_t *queue, const l3_radar_cal_t *cal,
                             const l3_angle_job_t *job, l3_club_track_t *track,
                             l3_angle_obs_t *obs)
{
    int32_t estimated = l3_angle_estimate(cal, &job->snapshot, obs);

    return l3_angle_queue_record(queue, job, estimated, obs, track);
}

int32_t l3_angle_queue_finish(l3_angle_queue_t *queue, const l3_angle_job_t *job,
                              int32_t estimated, const l3_angle_obs_t *obs,
                              l3_club_track_t *track)
{
    if (queue->count == 0U || queue->jobs[queue->head].timestampUs != job->timestampUs) {
        return -2; /* a fire frame's drain applied it meanwhile */
    }
    queue->head = (queue->head + 1U) % L3_ANGLE_QUEUE_DEPTH;
    queue->count--;
    return l3_angle_queue_record(queue, job, estimated, obs, track);
}
