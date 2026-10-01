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
/* Global range-FFT bins the standing-return counts cover. */
#define L3_TRACK_GLOBAL_BINS 128U
/* A track moving at least this many bins a frame that predicts the club within
 * L3_TRACK_STANDING_PASS_ERR_BINS of a target may take it even in a standing
 * bin: the club sweeps through the bin of a return standing there. */
#define L3_TRACK_STANDING_PASS_BINS_PER_FRAME 1.0F
#define L3_TRACK_STANDING_PASS_ERR_BINS 1.0F
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
/* The club's speed at impact, for l3_track_follow's cap, is fitted over this
 * many of its newest points. */
#define L3_TRACK_FOLLOW_FIT_POINTS 4U
/* After impact the club is no faster than it arrived: re-acquisition takes a
 * return whose rate from the ball is at most this times the approach speed. */
#define L3_TRACK_FOLLOW_MAX_RATIO 1.10F
#define L3_TRACK_FOLLOW_UNKNOWN_APPROACH_MPS 70.0F  /* no approach measured: the fastest club (l3_impact_fit clubMaxMps) */
/* A point taken beyond the band after impact (re-acquired, or re-emerged onto
 * a track the band hid) is tentative: the next associated point must lie at
 * least this far downrange of it, or the point is withdrawn. */
#define L3_TRACK_TENTATIVE_ADVANCE_BINS 1.0F
/* Candidate approaches (acquireMaxStepBins > 0): how many are held, and the
 * most frames between a candidate and the point that confirms it. */
#define L3_TRACK_CANDIDATES 4U
#define L3_TRACK_CANDIDATE_MAX_GAP_FRAMES 2U

/* What reconstruction (l3_ball_fit.h, l3_track_kf.h) made of a point. */
enum {
    L3_FILTER_HYP_NONE = 0,      /* reconstructed, but its angles were not used
                                  * (none measured, no weight, or gated out) */
    L3_FILTER_HYP_DIRECT,        /* its angles were used as the direct return */
    L3_FILTER_HYP_IMAGE,         /* ... as the floor reflection */
    L3_FILTER_HYP_AMBIGUOUS,     /* ... direct and reflection too close to tell */
    L3_FILTER_HYP_UNFILTERED,    /* not reconstructed: filteredPosition is position */
    L3_FILTER_HYP_COUNT
};

/* The club reconstruction's constants (l3_track_kf.h). */
typedef struct {
    float accelSigmaMps2;        /* white-acceleration process noise */
    float rangeSigmaM;           /* one range measurement */
    float angleSigmaRad;         /* one angle at full angle confidence */
    float minAngleConfidence;    /* floor on the confidence dividing angleSigmaRad */
    float chi2Gate;              /* 2-dof gate on the angle pair's innovation */
    float initPositionSigmaM;    /* first point's position uncertainty */
    float initVelocitySigmaMps;  /* first point's velocity uncertainty (starts at 0) */
} l3_track_kf_cfg_t;

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
    float     angleConfidence;    /* l3_angle_estimate's confidence for these angles, 0 none */
    l3_vec3_t filteredPosition;   /* reconstructed GOLF-frame position; position when
                                   * filterHypothesis is L3_FILTER_HYP_UNFILTERED */
    uint8_t   filterAccepted;     /* 1 when the reconstruction used this point's angles */
    uint8_t   filterHypothesis;   /* L3_FILTER_HYP_* */
} l3_track_point_t;

typedef struct {
    float    binWidthM;           /* range per bin */
    float    gateBins;            /* association gate around the prediction */
    uint32_t maxMisses;           /* frames coasted on the prediction */
    float    minConfidence;       /* acquire only a target this confident */
    float    weightRange;         /* score = wR * rangeErrBins + ... */
    float    weightVelocity;      /*       + wV * wrapped Doppler diff / span */
    float    weightQuality;       /*       + wQ * (1 - confidence) */
    float    weightStrength;      /*       + wS * (1 / snr): prefer MTI-strong targets */
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
    /* After impact (l3_track_follow) the club only slows: its aliased Doppler
     * (m/s, wrapped over the alias span) falls from the last point's by at most
     * followDopplerTolMps and rises by at most followDopplerRiseMps. A return
     * outside that (the shaft and hands lag the head and read another velocity)
     * is not the club however strong it is. followDopplerTolMps 0 disables. */
    float    followDopplerTolMps;
    float    followDopplerRiseMps;
    /* Before impact the club sweeps several bins a frame, so its approach
     * never holds a bin for two consecutive points: with this many
     * consecutive points in one rounded bin, one more releases the track (the
     * hands or body, strong and steady, outranked the weak club at
     * acquisition). Stricter than maxSameBinPoints, which after impact allows
     * the stall beside the ball two points. 0 disables. */
    uint32_t approachMaxSameBinPoints;
    /* A return that has stood in its bin (a target within one bin of it) for
     * this many consecutive frames before this one is a standing return -- the
     * hands, body or the stall beside the ball -- and never a candidate: the
     * club is in any one bin for about a frame, however strong the standing
     * return is and whatever its Doppler reads. Acquisition, association and
     * following skip it (association keeps the track's own bin, which the
     * same-bin rules handle); 0 disables. */
    uint32_t standingFrames;
    /* Acquisition by candidate approach. The golfer's body, in the bins short
     * of the ball, returns far stronger than the club and reads as a slow
     * mover, so the most confident target of one frame is often the body. A
     * target at least acquireMinConfidence becomes a candidate; a track starts
     * only when a later target (within L3_TRACK_CANDIDATE_MAX_GAP_FRAMES)
     * steps on from it by acquireMinStepBins..acquireMaxStepBins a frame with
     * an aliased Doppler within acquireDopplerTolMps of it. The body never
     * steps. acquireMaxStepBins 0 (the default) acquires the best target of
     * one frame, as before; 4.5 is a 70 m/s club's step. Off because on the
     * labelled swings it also confirmed hops between still returns. As
     * before (minConfidence, the mover preference). */
    float    acquireMinStepBins;
    float    acquireMaxStepBins;
    float    acquireDopplerTolMps;
    float    acquireMinConfidence;
    /* Of several confirming pairs, the one whose step per frame is nearest this
     * (with association's Doppler, quality and strength terms). */
    float    acquireExpectedStepBins;
    /* The club reconstruction (l3_track_kf.h). The ball's core carries it too,
     * unused: the ball has its own fit (l3_ball_fit.h). */
    l3_track_kf_cfg_t kf;
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

/* One point of a list the delivery fit reads, by index; 0 when out of range. */
typedef int32_t (*l3_point_at_fn)(const void *ctx, uint32_t index, l3_track_point_t *out);

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

/* The track as it was before a tentative point (see l3_track_follow), put
 * back when the point is withdrawn. */
typedef struct {
    uint8_t  active;
    uint8_t  following;
    uint32_t misses;
    uint32_t lastFrame;
    float    lastBin;
    float    velocityBinsPerFrame;
    float    followBinsPerS;
    int32_t  sameBin;
    uint32_t sameBinCount;
} l3_track_held_t;

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
    uint8_t  tentative;           /* the newest point awaits confirmation (held below) */
    float    followBinsPerS;      /* the club's fitted speed at impact: the follow's cap */
    /* The last released track's final bin and Doppler, which acquisition
     * avoids (see L3_TRACK_RELEASE_DOPPLER_TOL_MPS) until something else is
     * acquired; releasedValid is 0 when nothing was released. */
    uint8_t  releasedValid;
    float    releasedBin;
    float    releasedDopplerMps;
    l3_track_held_t held;         /* the track before its tentative point */
    /* Per global bin: consecutive frames a target has stood within one bin of
     * it (see standingFrames). Kept across releases and resets. */
    uint8_t  standHold[L3_TRACK_GLOBAL_BINS];
    /* Candidate approaches awaiting a confirming step (acquireMaxStepBins). */
    uint32_t candidateCount;
    l3_target_obs_t candidates[L3_TRACK_CANDIDATES];
    /* The last update's targets' bins and frame: a confirming step must land
     * where no other return stood the frame before. */
    uint32_t prevFrame;
    uint32_t prevCount;
    float    prevBins[L3_OBS_MAX_TARGETS];
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
/* The scene after impact, as l3_track_follow needs it. NULL: plain follow. */
typedef struct {
    uint8_t  bandValid;
    float    bandHiBin;          /* the tee band's far edge (global bin) */
    float    originBin;          /* the ball's bin at rest */
    uint32_t impactTimestampUs;
    float    approachBinsPerS;   /* the club's range rate arriving, > 0 when known */
    float    ballBinsPerS;       /* the ball track's current rate, 0 when unknown */
    uint32_t ballClaimIndex;     /* this frame's ball target, L3_TRACK_NO_TARGET for none */
    uint32_t frameUs;            /* nominal frame period */
    uint8_t  approachKnown;      /* 1: approachBinsPerS was measured (the delivery's
                                  * speed); 0: it is the unknown-approach ceiling */
} l3_follow_ctx_t;

/* After impact: continue an active track by association alone -- never
 * release (a club slowing after impact repeats its bin) -- taking the
 * STRONGEST target between L3_TRACK_FOLLOW_RETREAT_BINS behind the last point
 * and where the club would be at its impact speed (fitted in bins per second
 * on the first call; the context's approach speed when the fit gives none)
 * plus L3_TRACK_FOLLOW_LEAD_BINS, never a third consecutive point in one bin:
 * of the two tracks visible after impact the club is the stronger, the ball
 * the weaker, and the club only slows while the ball leaves faster.
 * With a context (ctx != NULL) the scene after impact is used as well:
 *  - the ball's claimed target (ballClaimIndex) is never the club, nor is a
 *    return that left the last point at the ball's rate or faster;
 *  - while the last point is short of the tee band's far edge, a frame with
 *    nothing in the follow window takes the return beyond the band that
 *    re-acquisition would (below; also slower than the ball from the last
 *    point) onto the same track, then follows at the approach speed as a
 *    re-acquired club does; with none it coasts instead of counting
 *    toward a drop, for as long as crossing the band at the impact speed
 *    takes, plus a frame;
 *  - an inactive track is re-acquired from the strongest departing return
 *    beyond the band (the ball's bin without a band) whose rate from the ball
 *    since the impact time is positive, at most L3_TRACK_FOLLOW_MAX_RATIO
 *    times the approach speed, slower than the ball, not the ball's target
 *    and at least cfg.minConfidence;
 *  - with the approach unknown (approachKnown 0: the approach is only the
 *    ceiling) and no ball rate yet, nothing is re-acquired or re-emerges;
 *  - a re-acquired or re-emerged point is TENTATIVE (track->tentative): kept
 *    on the track, but confirmed only when the next associated point lies at
 *    least L3_TRACK_TENTATIVE_ADVANCE_BINS downrange of it. When the next
 *    point does not (that frame then takes nothing), or when the miss rules
 *    would drop the track before a next point comes, the tentative point is
 *    withdrawn -- popped from the ring, count and total back by one -- and
 *    the track is as it was before it (track->held): inactive after a
 *    re-acquisition, hidden and coasting after a re-emergence, the frames
 *    since counted as misses. A static return beyond the band is thus never a
 *    club point. Withdrawing a point from a full ring does not bring back the
 *    oldest point its append overwrote. l3_fit_span_after leaves out a point
 *    still tentative.
 * With NULL it only continues an active track and never acquires.
 * lastTargetIndex says which of this frame's targets it claimed. Returns 1
 * when a point was appended. */
int32_t l3_track_follow(l3_club_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                        uint32_t frame, uint32_t timestampUs, const l3_follow_ctx_t *ctx);
/* Range rate (bins/s) over the newest points: the fit of up to four when three
 * or more are held, the two newest otherwise, 0 below two. */
float l3_track_recent_rate(const l3_club_track_t *track);
/* Angles for the point the last update appended (the target at
 * lastTargetIndex), measured after association so only one target per frame
 * needs an angle estimate. Recomputes that point's golf-frame position.
 * Returns 0 when the last update appended nothing. */
int32_t l3_track_set_angles(l3_club_track_t *track, float azimuthRad, float elevationRad,
                            uint8_t anglesValid, float angleConfidence);
/* The same for any stored point (index 0 is the oldest held): sets its angles
 * and their confidence, recomputes its golf-frame position and marks it
 * unfiltered (a new angle voids an earlier reconstruction). Returns 0 when
 * index is not stored. */
int32_t l3_track_set_point_angles(l3_club_track_t *track, uint32_t index, float azimuthRad,
                                  float elevationRad, uint8_t anglesValid, float angleConfidence);
/* A stored point for reconstruction to write into (index 0 oldest); NULL when
 * not stored. */
l3_track_point_t *l3_track_point_mut(l3_club_track_t *track, uint32_t index);
/* Mark one point, or every held point, not reconstructed. */
void l3_track_point_unfilter(l3_track_point_t *point);
void l3_track_unfilter_all(l3_club_track_t *track);
/* The index of the first of the newest maxPoints held points. */
uint32_t l3_track_newest_first(const l3_club_track_t *track, uint32_t maxPoints);
/* Point index 0 is the oldest held. Returns 0 when out of range. */
int32_t l3_track_point(const l3_club_track_t *track, uint32_t index, l3_track_point_t *out);
/* The index (0 oldest) of the point with timestampUs: 1, or 0 when no held
 * point has it (a reset, or rolled off the history). */
int32_t l3_track_find_point(const l3_club_track_t *track, uint32_t timestampUs,
                            uint32_t *index);
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
/* The 3D fit of points [first, last) read through pointAt: what
 * l3_track_delivery_range does for a track, for any point list. Returns the
 * points used, 0 below three. */
uint32_t l3_delivery_fit(l3_point_at_fn pointAt, const void *ctx, uint32_t first, uint32_t last,
                         uint32_t fullPoints, float binWidthM, float maxAngleResidualM,
                         l3_delivery_t *out);
/* Append a point made elsewhere: located with this track's calibration,
 * lastBin and lastFrame updated. */
void l3_track_append_point(l3_club_track_t *track, const l3_track_point_t *point);
int32_t l3_track_format_delivery(const l3_delivery_t *delivery, char *out, uint32_t cap);
/* Least-squares fit of rangeBin against time over the newest maxPoints
 * points (at least 3). Returns the points used, 0 when too few; slope in
 * bins per second, residual as RMS bins. */
uint32_t l3_track_fit(const l3_club_track_t *track, uint32_t maxPoints, float *slopeBinsPerS,
                      float *residualBins);
/* |fitted slope| in m/s over the newest maxPoints, 0 without a fit. */
float l3_track_speed_mps(const l3_club_track_t *track, uint32_t maxPoints);
/* |a - b| the smaller way round a circle of the given span: the difference of
 * two aliased Doppler readings, or of a speed and an aliased reading (they
 * agree when the speed wraps onto the reading). 0 for a span <= 0. */
float l3_track_wrapped_diff(float a, float b, float span);
const char *l3_track_why_name(uint8_t why);
/* "clubtrack active=1 count=14 bin=44.20 dist=3.8 vel=1.92 speed=22.4 ..." */
int32_t l3_track_format_status(const l3_club_track_t *track, uint32_t destBin, char *out,
                               uint32_t cap);
int32_t l3_track_format_point(const l3_track_point_t *point, uint32_t destBin, char *out,
                              uint32_t cap);

#endif /* L3_CLUB_TRACK_H */
