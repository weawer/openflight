/* IWR6843 club track: a persistent trajectory history with predictive
 * association, over the observation layer's targets.
 *
 * The trigger's trackBin/trackAge is enough to fire on; it is not a
 * measurement. This keeps the last L3_TRACK_POINTS observations of the
 * clubhead as a circular history, associates each frame's targets to the
 * track by predicting where the club should be (range, then Doppler
 * continuity and signal quality) instead of taking the strongest return, so
 * the shaft, hands and body cannot steal the track, and fits range against
 * time over the recent points for a club speed that does not depend on the
 * aliased Doppler. Angles are carried through for the 3D trajectory to come.
 *
 * Bins are GLOBAL range-FFT bins. Toward the ball means a rising bin. Pure C.
 */
#ifndef L3_CLUB_TRACK_H
#define L3_CLUB_TRACK_H

#include <stdint.h>

#include "l3_frames.h"
#include "l3_observation.h"

#define L3_TRACK_POINTS 32U
#define L3_TRACK_NO_TARGET 0xFFFFFFFFU
/* Fit points counted as "enough" for full confidence. */
#define L3_TRACK_FULL_POINTS 8U
/* After a release, acquisition skips a target that looks like the released
 * return: inside its gate AND within this much aliased Doppler of it. A club
 * sweeping through that range reads at a different Doppler and is taken. */
#define L3_TRACK_RELEASE_DOPPLER_TOL_MPS 2.0F
/* After impact the club stalls and its peak jitters: l3_track_follow looks
 * this far behind the track's last point. */
#define L3_TRACK_FOLLOW_RETREAT_BINS 1.0F
/* ... and no further ahead than its impact speed allows, plus this: after
 * impact the club only slows, and the ball leaves faster than the club arrived
 * (smash factor > 1), so a return beyond is the ball's. Half a bin covers the
 * sub-bin jitter of a club still at its impact speed. */
#define L3_TRACK_FOLLOW_LEAD_BINS 0.5F

typedef struct {
    uint32_t frame;
    uint32_t timestampUs;
    float    rangeBin;            /* global, sub-bin */
    float    rangeM;
    float    radialVelocityMps;   /* from the range rate, not Doppler */
    float    dopplerAliasMps;
    float    azimuthRad;          /* positive right */
    float    elevationRad;        /* positive up */
    uint8_t  anglesValid;         /* L3_OBS_ANGLE_* bits measured for this point */
    float    energy;
    float    coherence;
    float    confidence;
    l3_vec3_t position;           /* GOLF frame metres from the antenna; an angle
                                   * not measured is taken as boresight */
} l3_track_point_t;

typedef struct {
    float    binWidthM;           /* range per bin */
    float    gateBins;            /* association gate around the prediction */
    uint32_t maxMisses;           /* frames coasted on the prediction */
    float    minConfidence;       /* acquire only a target this confident */
    float    weightRange;         /* score = wR * rangeErrBins + ... */
    float    weightVelocity;      /*       + wV * wrapped Doppler diff / span */
    float    weightQuality;       /*       + wQ * (1 - confidence) */
    float    velocitySpanMps;     /* Doppler alias span (2 * wavelength / 4T) */
    l3_radar_cal_t cal;           /* attitude, offsets and range bias for positions */
    /* Acquisition prefers a target whose aliased Doppler reads at least this
     * (m/s): a body standing in the lane reads under 1 m/s, a clubhead reads
     * anywhere across the alias span, so the club is missed on a frame only
     * when it happens to alias near zero. 0 disables the preference. */
    float    minAcquireDopplerMps;
    /* Beyond this 3D fit residual (metres) the angles are not trusted: the
     * delivery falls back to the radial speed and marks path and attack
     * invalid instead of reporting a precise-looking wrong direction. */
    float    maxAngleResidualM;
    /* The clubhead approaches the ball: it moves into ascending bins.
     * With ascendingOnly, association never takes a candidate whose rounded
     * bin is below the track's last point's; with maxSameBinPoints (0
     * disables), one more consecutive point than that in the same rounded bin
     * says the track is not the club -- a hand or body beside the ball reads
     * as a mover (it can alias to several m/s) and wins acquisition, but it
     * stays put -- and the track is released so the club can be acquired. */
    uint32_t ascendingOnly;
    uint32_t maxSameBinPoints;
} l3_track_cfg_t;

/* Club delivery from a regression of position against time over the newest
 * points: the velocity vector at the newest point (GOLF frame), and from it
 * club speed, club path (horizontal, positive right) and angle of attack
 * (vertical, positive up). Which of the three are valid depends on which
 * angles the fitted points carried: range alone gives the radial speed,
 * elevation adds the attack angle, azimuth adds the path. */
typedef struct {
    uint32_t points;              /* points the fit used */
    uint32_t azimuthPoints;       /* of those, with a measured azimuth */
    uint32_t elevationPoints;     /* ... with a measured elevation */
    l3_vec3_t velocity;           /* m/s, golf frame */
    l3_vec3_t position;           /* fitted position at the newest point's time */
    uint32_t timestampUs;         /* the newest point's time */
    float    speedMps;            /* |velocity| */
    float    radialSpeedMps;      /* from the range-only fit, for cross-checking */
    float    pathRad;
    float    attackRad;
    float    residualM;           /* RMS 3D fit residual */
    float    confidence;          /* 0..1: residual, point count and point quality */
    uint8_t  speedValid;
    uint8_t  pathValid;
    uint8_t  attackValid;
} l3_delivery_t;

enum {
    L3_TRACK_WHY_NONE = 0,
    L3_TRACK_WHY_ACQUIRED,
    L3_TRACK_WHY_ASSOCIATED,
    L3_TRACK_WHY_COASTED,       /* nothing in the gate; predicted forward */
    L3_TRACK_WHY_DROPPED,       /* coasted too long */
    L3_TRACK_WHY_IDLE,          /* no track and nothing confident enough */
    L3_TRACK_WHY_RELEASED,      /* held one bin too long: dropped for reacquisition */
    L3_TRACK_WHY_COUNT
};

typedef struct {
    l3_track_cfg_t cfg;
    uint8_t  active;
    uint8_t  why;                 /* last update */
    uint32_t next;                /* ring write index */
    uint32_t count;               /* points held, at most L3_TRACK_POINTS */
    uint32_t total;               /* points appended since init */
    uint32_t misses;
    uint32_t lastFrame;
    float    lastBin;
    float    velocityBinsPerFrame;
    float    predictedBin;
    uint32_t lastTargetIndex;     /* index into the last update's targets that was
                                   * appended, L3_TRACK_NO_TARGET when none */
    l3_track_point_t points[L3_TRACK_POINTS];
    uint32_t counters[L3_TRACK_WHY_COUNT];
    int32_t  sameBin;             /* rounded bin of the newest point ... */
    uint32_t sameBinCount;        /* ... and how many consecutive points share it */
    uint8_t  following;           /* l3_track_follow has taken over (after impact) */
    float    followBinsPerFrame;  /* the approach speed at impact: the follow's cap */
    /* The last released track's final bin and Doppler, which acquisition
     * avoids (see L3_TRACK_RELEASE_DOPPLER_TOL_MPS) until something else is
     * acquired; releasedValid is 0 when nothing was released. */
    uint8_t  releasedValid;
    float    releasedBin;
    float    releasedDopplerMps;
} l3_club_track_t;

void l3_track_cfg_defaults(l3_track_cfg_t *cfg);
void l3_track_init(l3_club_track_t *track, const l3_track_cfg_t *cfg);
/* Forget the track and its history; keep the configuration and counters. */
void l3_track_reset(l3_club_track_t *track);
/* One frame's targets (strongest first, from l3_obs_extract). Returns 1
 * when a point was appended. Frames without a call are frames without
 * observations; the prediction uses frame numbers, so call once per frame. */
int32_t l3_track_update(l3_club_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                        uint32_t frame, uint32_t timestampUs);
/* After impact: continue an active track by association alone -- never
 * acquire, never release (a club slowing after impact repeats its bin) --
 * taking the STRONGEST target between L3_TRACK_FOLLOW_RETREAT_BINS behind the
 * last point and where the club would be at its impact speed (frozen on the
 * first call) plus L3_TRACK_FOLLOW_LEAD_BINS, never a third consecutive point
 * in one bin: of the two tracks visible after impact the club is the
 * stronger, the ball the weaker, and the club only slows while the ball
 * leaves faster. lastTargetIndex
 * says which of this frame's targets it claimed. Returns 1 when a point was
 * appended. */
int32_t l3_track_follow(l3_club_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                        uint32_t frame, uint32_t timestampUs);
/* Angles for the point the last update appended (the target at
 * lastTargetIndex), measured after association so only one target per frame
 * needs an angle estimate. Recomputes that point's golf-frame position.
 * Returns 0 when the last update appended nothing. */
int32_t l3_track_set_angles(l3_club_track_t *track, float azimuthRad, float elevationRad,
                            uint8_t anglesValid);
/* Point index 0 is the oldest held. Returns 0 when out of range. */
int32_t l3_track_point(const l3_club_track_t *track, uint32_t index, l3_track_point_t *out);
/* The delivery from the newest maxPoints points (at least 3). Returns the
 * points used, 0 when too few; out is fully written either way. */
uint32_t l3_track_delivery(const l3_club_track_t *track, uint32_t maxPoints, l3_delivery_t *out);
/* The same fit over points [first, first + count) in oldest-first order, so a
 * caller can fit the EARLIEST points (the ball's first clean flight) as well
 * as the newest. timestampUs and position refer to the last point fitted;
 * fullPoints is the count that earns full confidence for this kind of fit. */
uint32_t l3_track_delivery_range(const l3_club_track_t *track, uint32_t first, uint32_t count,
                                 uint32_t fullPoints, l3_delivery_t *out);
/* "delivery points=8 az=8 el=8 speed=22.40 radial=22.00 path=2.10 attack=-3.40
 *  residual=0.012 conf=0.81 valid=spa" */
int32_t l3_track_format_delivery(const l3_delivery_t *delivery, char *out, uint32_t cap);
/* Least-squares fit of rangeBin against time over the newest maxPoints
 * points (at least 3). Returns the points used, 0 when too few; slope in
 * bins per second, residual as RMS bins. */
uint32_t l3_track_fit(const l3_club_track_t *track, uint32_t maxPoints, float *slopeBinsPerS,
                      float *residualBins);
/* |fitted slope| in m/s over the newest maxPoints, 0 without a fit. */
float l3_track_speed_mps(const l3_club_track_t *track, uint32_t maxPoints);
const char *l3_track_why_name(uint8_t why);
/* "clubtrack active=1 count=14 bin=44.20 dist=3.8 vel=1.92 speed=22.4 ..." */
int32_t l3_track_format_status(const l3_club_track_t *track, uint32_t destBin, char *out,
                               uint32_t cap);
int32_t l3_track_format_point(const l3_track_point_t *point, uint32_t destBin, char *out,
                              uint32_t cap);

#endif /* L3_CLUB_TRACK_H */
