/* IWR6843 ball hypotheses: the ball found after impact, not assumed.
 *
 * After the gate fires, the club carries on through impact and is usually
 * the strongest, most confident departing return. Taking the most confident
 * target in the departure band therefore follows the club. Instead, up to
 * L3_BALL_HYP_MAX candidate trajectories are kept that start near the origin;
 * each frame's targets are assigned to them jointly with the club track, whose
 * claimed target never becomes a ball point (a merged return is a missed
 * frame); and only once a hypothesis holds enough points is it judged
 * (l3_ball_hyps_classify): origin crossing near the gate time, a physical
 * range rate, a straight fit, Doppler agreeing with the rate, and a return
 * weaker than the club's. Prediction, gates and fits run on timestamps, not
 * frame counts. Pure C, fixed size, no hardware.
 */
#ifndef L3_BALL_HYP_H
#define L3_BALL_HYP_H

#include <stdint.h>

#include "l3_ball_anchor.h"
#include "l3_observation.h"

/* Build switch. The search is off at run time (l3_ball_track_cfg_defaults)
 * until the recorded captures say otherwise, so the board image compiles it
 * out of l3_ball_track_t to save DATA_RAM (L3_FEATURE_DEFS in the makefile);
 * the host build keeps it for the replay and the tests. With it at 0 the
 * functions below still build but nothing calls them. */
#ifndef L3_BALL_HYPOTHESES
#define L3_BALL_HYPOTHESES 1
#endif

#define L3_BALL_HYP_MAX    4U
#define L3_BALL_HYP_POINTS 8U
#define L3_BALL_HYP_NONE   0xFFFFFFFFU

typedef struct {
    uint32_t frame;
    uint32_t timestampUs;
    float    rangeBin;            /* global, sub-bin */
    float    dopplerAliasMps;
    float    stat;
    float    clubStat;            /* the club's claimed return that frame, 0 without one */
    float    coherence;           /* the target's lag-1 coherence */
    float    azimuthRad;
    float    elevationRad;
    uint8_t  anglesValid;         /* L3_OBS_ANGLE_* bits */
    float    angleConfidence;     /* l3_angle_estimate's confidence for these angles */
} l3_ball_hyp_point_t;

typedef struct {
    uint8_t  active;
    uint8_t  count;
    uint8_t  misses;              /* consecutive frames without a point */
    uint32_t id;                  /* spawn order: the tie-break */
    uint32_t lastTargetIndex;     /* this frame's target, L3_BALL_HYP_NONE when none */
    l3_ball_hyp_point_t points[L3_BALL_HYP_POINTS];  /* oldest first; the oldest slides out */
} l3_ball_hyp_t;

typedef struct {
    float    binWidthM;           /* l3_ball_track_init copies these two from its core */
    float    velocitySpanMps;
    float    spawnBehindM;        /* a hypothesis starts from origin - this ... */
    float    spawnBeyondM;        /* ... to origin + this, + maxSpeedMps x the time
                                   * since the gate (a late gate finds the ball out);
                                   * used only with corridorGate off */
    float    gateM;               /* association half-width at zero elapsed time ... */
    float    gateMps;             /* ... growing by this speed uncertainty over the gap */
    uint32_t coastUs;             /* longest a hypothesis goes without a point ... */
    uint32_t impactCoastUs;       /* ... unless its newest point is short of the tee + */
    float    impactRegionM;       /* this: the ball is hidden near impact for longer */
    uint32_t classifyPoints;      /* points before a hypothesis may be the ball */
    float    minDepartureMps;
    float    maxSpeedMps;
    float    maxResidualBins;     /* RMS about the fitted line */
    float    dopplerToleranceMps; /* a point agrees when its Doppler is this close to the rate */
    /* Fastest credible, the Pi detector's rule (tracking.find_ball_from_power):
     * the club's follow-through and the flying tee outlast the ball and win on
     * score, so when the best-scoring hypothesis is slower than fastBallMps, a
     * qualifying hypothesis at or above it with at least fastSupportFraction of
     * the most points any qualifying hypothesis holds is the ball instead; and
     * a slow winner waits while an unclassified hypothesis is already moving
     * that fast. 0 turns the rule off. */
    float    fastBallMps;
    float    fastSupportFraction;
    /* Far window: targets short of origin + farWindowM (metres) are the club's, the
     * impact echo's or the golfer's and never become hypothesis points, so the
     * ball is taken only once it is clear of the merged bins. 0 turns it off. */
    float    farWindowM;
    /* G1: a target must be explainable by an impact within the anchor's
     * tolerance and a speed in [minDepartureMps, maxSpeedMps], within
     * anchorRangeTolM. 0: the start band [accept - spawnBehind, accept +
     * spawnBeyond + maxSpeedMps x elapsed] as before. */
    uint32_t corridorGate;
    float    anchorRangeTolM;
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
} l3_ball_hyps_cfg_t;

typedef struct {
    int32_t  index;               /* hypothesis index, -1 when none qualifies */
    uint32_t points;
    float    rateMps;             /* fitted range rate */
    float    originOffsetUs;      /* origin crossing minus the gate time */
    float    residualBins;
    float    dopplerAgreement;    /* 0..1 */
    float    weakerFraction;      /* 0..1 of the frames with a club return; 0.5 without */
    float    score;
    uint32_t waitingForFast;      /* 1 when a slow winner is held back for a fast
                                   * hypothesis still gathering points (index -1) */
    float    velocityConsistency; /* 0..1: the points' implied launch speeds agree */
    float    coherence;           /* mean lag-1 coherence of its points */
    uint8_t  anchorSource;        /* L3_BALL_ANCHOR_* the search back-projected to */
    uint32_t recovered;           /* points the backward pass added at adoption */
    uint32_t recoveredFirstFrame;
    uint32_t recoveredMask;       /* bit k: frame recoveredFirstFrame + k was recovered */
} l3_ball_hyp_verdict_t;

typedef struct {
    l3_ball_hyps_cfg_t cfg;
    uint8_t  armed;
    l3_ball_anchor_t anchor;      /* where and when the ball was struck; acceptFromBin is the old origin */
    /* cfg's metric settings in bins, from binWidthM at init */
    float    spawnBehindBins;
    float    spawnBeyondBins;
    float    gateBins;
    float    farWindowBins;
    float    impactRegionBins;
    uint32_t nextId;
    uint32_t spawned;
    uint32_t dropped;             /* coasted out or evicted */
    l3_ball_hyp_t hyp[L3_BALL_HYP_MAX];
} l3_ball_hyps_t;

void l3_ball_hyps_cfg_defaults(l3_ball_hyps_cfg_t *cfg);
void l3_ball_hyps_init(l3_ball_hyps_t *hyps, const l3_ball_hyps_cfg_t *cfg);
/* Forget every hypothesis and start looking from anchor->acceptFromBin, back-projecting to the anchor. */
void l3_ball_hyps_arm(l3_ball_hyps_t *hyps, const l3_ball_anchor_t *anchor);
/* One post-impact frame's targets (strongest first) and the index of the one
 * the club track claimed (L3_TRACK_NO_TARGET, or anything >= n, for none).
 * Returns the hypotheses active afterwards. */
uint32_t l3_ball_hyps_update(l3_ball_hyps_t *hyps, const l3_target_obs_t *targets, uint32_t n,
                             uint32_t frame, uint32_t timestampUs, uint32_t clubIndex);
/* Least squares of range against time over the hypothesis's points, time
 * measured from referenceUs: rate in bins per second, the fitted range at the
 * reference time and the RMS residual. Returns 0 with fewer than 2 points or
 * no spread in time. */
int32_t l3_ball_hyp_fit(const l3_ball_hyp_t *hyp, uint32_t referenceUs, float *rateBinsPerS,
                        float *binAtReference, float *residualBins);
/* l3_ball_hyp_fit over any time-ordered point array. */
int32_t l3_ball_points_fit(const l3_ball_hyp_point_t *points, uint32_t count, uint32_t referenceUs,
                           float *rateBinsPerS, float *binAtReference, float *residualBins);
/* Angles for the point hypothesis `index` appended this frame. Returns 0 when
 * it appended nothing this frame or the index is out of range. */
int32_t l3_ball_hyps_set_angles(l3_ball_hyps_t *hyps, uint32_t index, float azimuthRad,
                                float elevationRad, uint8_t anglesValid,
                                float angleConfidence);
/* The ball among the hypotheses holding at least classifyPoints points:
 * fitted over them from the gate time, it must move outward at
 * minDepartureMps..maxSpeedMps, cross the origin within the anchor tolerance of
 * the gate time and fit within maxResidualBins. The best score wins:
 * (1 - residual / maxResidualBins) + the fraction of points whose Doppler
 * agrees with the rate + half the fraction of club frames where it was the
 * weaker return, unless fastBallMps prefers a faster one (see the cfg).
 * out->index is -1 when none qualifies or the winner waits for a fast one. */
void l3_ball_hyps_classify(const l3_ball_hyps_t *hyps, l3_ball_hyp_verdict_t *out);
/* sizeof(l3_ball_hyps_t), for the ctypes mirror's layout check. */
uint32_t l3_ball_hyps_struct_bytes(void);

#endif /* L3_BALL_HYP_H */
