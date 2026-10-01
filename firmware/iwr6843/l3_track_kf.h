/* IWR6843 club trajectory reconstruction: a constant-velocity EKF over the
 * club track's held points and an RTS smoother after it.
 *
 * State [x y z vx vy vz], golf frame. Each point is measured as the range,
 * azimuth and elevation of its raw golf-frame position: range trusted
 * (rangeSigmaM), the angles weak (angleSigmaRad / max(angleConfidence,
 * minAngleConfidence)) and applied only when their innovation passes a 2-dof
 * chi-square gate (chi2Gate). A point failing it still updates on its range,
 * so the range track is never lost. The prediction uses each point's own time
 * step, so frames the track coasted through are handled. The whole track is
 * available when this runs (the replay and viewer run it once, after the shot; the board does not), so the RTS smoother runs
 * over it and each point's filteredPosition is the smoothed one.
 *
 * Range rate is not a measurement: radialVelocityMps is derived from the same
 * ranges, and using both would count them twice.
 *
 * On too few points or a numerical failure every point is left unfiltered
 * (filteredPosition = position): never a NaN. Pure C, no allocation: the
 * caller owns the work area (static on the board).
 */
#ifndef L3_TRACK_KF_H
#define L3_TRACK_KF_H

#include <stdint.h>

#include "l3_club_track.h"

#define L3_TRACK_KF_STATES 6U

typedef struct {
    float   x[L3_TRACK_POINTS][L3_TRACK_KF_STATES];   /* filtered, then smoothed */
    float   P[L3_TRACK_POINTS][L3_TRACK_KF_STATES][L3_TRACK_KF_STATES];
    float   xp[L3_TRACK_POINTS][L3_TRACK_KF_STATES];  /* predicted from the point before */
    float   Pp[L3_TRACK_POINTS][L3_TRACK_KF_STATES][L3_TRACK_KF_STATES];
    float   dt[L3_TRACK_POINTS];                      /* seconds since the point before */
    uint8_t accepted[L3_TRACK_POINTS];
} l3_track_kf_work_t;

enum {
    L3_TRACK_KF_WHY_NONE = 0,
    L3_TRACK_KF_WHY_OK,
    L3_TRACK_KF_WHY_FEW_POINTS,   /* under three held points */
    L3_TRACK_KF_WHY_DIVERGED,     /* a non-positive variance or a non-finite state */
    L3_TRACK_KF_WHY_COUNT
};

typedef struct {
    uint32_t points;
    uint32_t accepted;            /* points whose angles updated the state */
    uint8_t  why;
} l3_track_kf_result_t;

void l3_track_kf_cfg_defaults(l3_track_kf_cfg_t *cfg);
uint32_t l3_track_kf_work_bytes(void);
/* Filter and smooth the held points; write filteredPosition, filterAccepted
 * and filterHypothesis (DIRECT when its angles were used, NONE when not,
 * UNFILTERED on failure). Returns the accepted count, 0 on failure. */
uint32_t l3_track_kf_run(const l3_track_kf_cfg_t *cfg, l3_club_track_t *track,
                         l3_track_kf_work_t *work, l3_track_kf_result_t *out);
/* l3_track_delivery over the reconstruction: each reconstructed point's
 * filteredPosition, with its measured angle flags; a point left unfiltered
 * reads exactly as l3_track_delivery reads it. */
uint32_t l3_track_delivery_filtered(const l3_club_track_t *track, uint32_t maxPoints,
                                    l3_delivery_t *out);
const char *l3_track_kf_why_name(uint8_t why);


#endif /* L3_TRACK_KF_H */
