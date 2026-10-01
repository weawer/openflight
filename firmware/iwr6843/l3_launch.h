/* IWR6843 launch: the ball's fitted departure, walked back to impact.
 * Pure C, no hardware. */
#ifndef L3_LAUNCH_H
#define L3_LAUNCH_H

#include <stdint.h>

#include "l3_club_track.h"
#include "l3_frames.h"

/* Ball launch from the earliest clean flight, extrapolated to impact. */
typedef struct {
    uint32_t  points;
    l3_vec3_t velocity;           /* m/s, golf frame */
    l3_vec3_t launchPosition;     /* the fitted line at the impact time */
    float     speedMps;
    float     radialSpeedMps;     /* range-only fit, for cross-checking */
    float     hlaRad;             /* horizontal launch, positive right */
    float     vlaRad;             /* vertical launch, positive up */
    float     residualM;
    float     confidence;
    float     angleRmsRad;        /* the direction fit's RMS angle residual */
    uint8_t   speedValid;
    uint8_t   hlaValid;
    uint8_t   vlaValid;
    uint8_t   anglesAccepted;     /* ball points whose angles the direction fit kept */
    uint8_t   angleWhy;           /* L3_BALL_FIT_WHY_*: none until l3_ball_track_reconstruct */
} l3_launch_t;

/* The launch from a delivery fit over the ball's points: the fitted line is
 * anchored at its newest point, so it is walked back to impactTimestampUs. */
void l3_launch_from_delivery(const l3_delivery_t *fit, uint32_t impactTimestampUs,
                             l3_launch_t *out);
/* "launch points=5 speed=61.20 radial=58.90 hla=1.20 vla=12.40 residualmm=... conf=... angles=6 rms=3.10 why=ok valid=shv" */
int32_t l3_launch_format(const l3_launch_t *launch, char *out, uint32_t cap);

#endif /* L3_LAUNCH_H */
