/* IWR6843 ball direction fit: the ball's path from the tee, from its points'
 * ranges (trusted) and angles (not).
 *
 * Over the ~30 ms a ball track spans, gravity moves the ball ~3 mm, well under
 * a range bin, so the path is a straight line from the tee:
 * p(t) = tee + s(t) u(HLA, VLA). Each point's measured range fixes s (the
 * forward root of |tee + s u| = r), which leaves the direction's two angles
 * as the only unknowns. They are found by a coarse-to-fine grid search that
 * scores each point's measured direction against the direct return at p and
 * against its floor reflection (mirrored about the floor at radarHeightM),
 * the nearer counting, weighted by the point's angle confidence under a Huber
 * cap. A point further than gateK sigma from that fit is rejected and the
 * fit is redone without it. Too few kept points, too large a residual or a
 * best direction on the search limit gives no angles rather than a confident
 * wrong one. So does a direction the geometry cannot pin: the radar looks
 * nearly down the flight line, so an angle error shows at the antenna at a
 * fraction of its size and the residual stays at noise level. The cost's
 * curvature at the optimum (finite differences, 0.5 deg) gives the direction's
 * covariance, scaled by the observed scatter relative to angleSigmaRad; a sigma over
 * maxAngleSigmaRad is uncertain. Checked after the grid-edge and scatter checks.
 *
 * The tee: the ball track's origin gives the tee's slant range and bearing;
 * its height is teeBallHeightM above the floor, the antenna radarHeightM.
 *
 * Writes filteredPosition, filterAccepted and filterHypothesis onto every held
 * point: on the line when the fit is valid, unfiltered when not. Pure C, no
 * hardware, no allocation.
 */
#ifndef L3_BALL_FIT_H
#define L3_BALL_FIT_H

#include <stdint.h>

#include "l3_club_track.h"
#include "l3_frames.h"

typedef struct {
    float    angleSigmaRad;   /* one point's angle scatter */
    float    gateK;           /* rejected beyond gateK sigma of the fit */
    float    huberK;          /* quadratic within huberK sigma, linear beyond */
    uint32_t minAccepted;     /* fewer kept angles: no direction */
    float    maxRmsRad;       /* kept points' RMS beyond this: scatter, no direction */
    float    imageSepMinRad;  /* direct and reflection closer than this: ambiguous */
    float    radarHeightM;    /* antenna centre above the floor */
    float    teeBallHeightM;  /* ball centre above the floor at rest */
    float    hlaMinRad;       /* search limits */
    float    hlaMaxRad;
    float    vlaMinRad;
    float    vlaMaxRad;
    uint32_t gridSteps;       /* intervals per axis per level (at most 16) */
    uint32_t gridLevels;      /* each level spans one step either side of the last best (1 to 4) */
    float    maxAngleSigmaRad; /* either launch angle's 1-sigma beyond this: uncertain, no direction */
} l3_ball_fit_cfg_t;

enum {
    L3_BALL_FIT_WHY_NONE = 0,     /* not run */
    L3_BALL_FIT_WHY_OK,
    L3_BALL_FIT_WHY_FEW_ANGLES,   /* under minAccepted angles, before or after the gate */
    L3_BALL_FIT_WHY_SCATTER,      /* the kept angles' RMS exceeds maxRmsRad */
    L3_BALL_FIT_WHY_GRID_EDGE,    /* the best direction is on a search limit */
    L3_BALL_FIT_WHY_NO_TEE,       /* the origin gives no tee (range under the tee height) */
    L3_BALL_FIT_WHY_UNCERTAIN,    /* the geometry cannot pin the direction: a sigma exceeds maxAngleSigmaRad */
    L3_BALL_FIT_WHY_COUNT
};

typedef struct {
    float     hlaRad;         /* horizontal launch, positive right */
    float     vlaRad;         /* vertical launch, positive up */
    float     rmsRad;         /* kept points' weighted RMS angle residual */
    float     hlaSigmaRad;    /* 1-sigma of hlaRad from the cost's curvature */
    float     vlaSigmaRad;    /* 1-sigma of vlaRad */
    l3_vec3_t tee;            /* the anchor, golf frame */
    uint32_t  used;           /* points whose angles were weighed */
    uint32_t  accepted;       /* of those, kept by the gate */
    uint32_t  evaluations;    /* candidate directions scored */
    uint8_t   valid;
    uint8_t   why;            /* L3_BALL_FIT_WHY_* */
} l3_ball_fit_t;

void l3_ball_fit_cfg_defaults(l3_ball_fit_cfg_t *cfg);
/* The most candidate directions one run can score: two passes of every level,
 * plus the nine of the curvature estimate. */
uint32_t l3_ball_fit_max_evaluations(const l3_ball_fit_cfg_t *cfg);
/* Unit vector for (HLA, VLA) in the golf frame. */
void l3_ball_fit_direction(float hlaRad, float vlaRad, l3_vec3_t *u);
/* Fit the core's held points from origin (the ball track's golf-frame origin)
 * and write each point's reconstruction. Returns the accepted count, 0 when
 * the fit is not valid (out->why says why). */
uint32_t l3_ball_fit_run(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *origin,
                         l3_club_track_t *core, l3_ball_fit_t *out);
const char *l3_ball_fit_why_name(uint8_t why);

#endif /* L3_BALL_FIT_H */
