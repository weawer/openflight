/* IWR6843 ball track: the departing ball after impact.
 *
 * Kept apart from the club track even though both share the trajectory core
 * (l3_club_track.c: ring, predictive association, range-over-time fit),
 * because what is looked for differs. Before impact the target is the
 * approaching club; after it, a coherent return leaving the ball's origin
 * fast: acquired only at or beyond the origin bin, within a short gate, and
 * confirmed by the range rate its second point shows. Until then only
 * candidates at least a bin BEYOND the reference (the origin, then the first
 * point) are offered, which excludes the impact echo, the resting club and
 * the clubhead's follow-through behind the ball; once flying, the ball never
 * comes back toward the radar, so a candidate behind the last point is never
 * it. A return that fails the departure test is dropped and the search
 * restarts.
 *
 * The launch is fitted over the EARLIEST clean points of the flight, not
 * the newest, because drag takes speed off from the first metre: the
 * velocity vector extrapolated to the impact time gives ball speed, the
 * horizontal launch angle (positive right) and the vertical launch angle
 * (positive up) in the golf frame. Pure C, no hardware.
 */
#ifndef L3_BALL_TRACK_H
#define L3_BALL_TRACK_H

#include <stdint.h>

#include "l3_club_track.h"
#include "l3_ball_hyp.h"
#include "l3_frames.h"
#include "l3_ball_fit.h"
#include "l3_launch.h"

typedef struct {
    l3_track_cfg_t core;          /* association gate, misses, weights, calibration */
    float    minDepartureMps;     /* range rate the second point must show */
    float    maxSpeedMps;         /* physical ceiling; faster is not a ball */
    float    originGateBins;      /* acquire within this many bins beyond the origin */
    float    minDepartureBins;    /* ... and at least this many beyond it: the impact
                                   * echo and the resting club sit at the origin */
    float    displaceConfidence;  /* an unconfirmed first point with no confirming
                                   * point gives way to a departure in the origin
                                   * gate this confident (and more than it) */
    uint32_t launchPoints;        /* earliest points fitted for the launch */
    float    snr;                 /* extraction threshold over the floor for the post
                                   * window: a departing ball is a weak return */
    /* Search for the ball with the hypotheses (l3_ball_hyp.h) instead of
     * taking the most confident departing target; set from the recorded
     * captures (docs/superpowers/specs/2026-09-28-iwr-joint-club-ball-tracking.md). */
    uint32_t useHypotheses;
    /* Once confirmed, skip the club's claimed target while another candidate
     * is in the gate. */
    uint32_t skipClubClaim;
    /* The ball's direction: the tee-anchored fit over every held point
     * (l3_ball_fit.h), once per shot by l3_ball_track_reconstruct. */
    l3_ball_fit_cfg_t fit;
#if L3_BALL_HYPOTHESES
    l3_ball_hyps_cfg_t hyps;      /* binWidthM and velocitySpanMps come from core */
#endif
} l3_ball_track_cfg_t;

enum {
    L3_BALL_TRACK_WHY_NONE = 0,
    L3_BALL_TRACK_WHY_UNARMED,     /* no impact yet */
    L3_BALL_TRACK_WHY_NO_CANDIDATE,/* nothing beyond the origin */
    L3_BALL_TRACK_WHY_ACQUIRED,
    L3_BALL_TRACK_WHY_CONFIRMED,   /* the second point departs fast enough */
    L3_BALL_TRACK_WHY_TOO_SLOW,    /* it did not: dropped, searching again */
    L3_BALL_TRACK_WHY_TOO_FAST,    /* faster than any ball: dropped */
    L3_BALL_TRACK_WHY_TRACKED,
    L3_BALL_TRACK_WHY_COASTED,
    L3_BALL_TRACK_WHY_LOST,        /* the flight left the window or the track dropped */
    L3_BALL_TRACK_WHY_SEARCHING,   /* hypotheses kept; none is the ball yet */
    L3_BALL_TRACK_WHY_COUNT
};

typedef struct {
    l3_ball_track_cfg_t cfg;
    l3_club_track_t core;
    uint8_t   armed;
    uint8_t   confirmed;
    uint8_t   why;
    uint8_t   done;               /* nothing more will be added: lost after confirmation */
    uint32_t  impactTimestampUs;
    float     originBin;          /* global bin of the ball at impact */
    l3_vec3_t origin;             /* golf frame */
    uint32_t  lastTargetIndex;    /* index into the last update's targets that was
                                   * appended, L3_TRACK_NO_TARGET when none */
    uint32_t  counters[L3_BALL_TRACK_WHY_COUNT];
#if L3_BALL_HYPOTHESES
    l3_ball_hyps_t hyps;
    l3_ball_hyp_verdict_t verdict; /* the last classification; index -1 before one */
#endif
} l3_ball_track_t;

void l3_ball_track_cfg_defaults(l3_ball_track_cfg_t *cfg);
void l3_ball_track_init(l3_ball_track_t *track, const l3_ball_track_cfg_t *cfg);
/* Forget the flight and the arming; configuration and counters survive. */
void l3_ball_track_reset(l3_ball_track_t *track);
/* IMPACT: start looking for a ball leaving originBin (global) at origin. */
void l3_ball_track_arm(l3_ball_track_t *track, float originBin, const l3_vec3_t *origin,
                       uint32_t impactTimestampUs);
/* Start the flight from two points already known to be the ball (the
 * ball-leave fallback's, l3_leave.h): after its late fire the departing ball
 * is too smeared for the tracker to acquire, but once a flight exists it is
 * followed at any confidence. The pair still passes the departure checks
 * (minDepartureMps, maxSpeedMps). Returns 1 when the flight was seeded and
 * confirmed; 0 unarmed, already confirmed, done, or refused. */
int32_t l3_ball_track_seed(l3_ball_track_t *track, const l3_target_obs_t *first,
                           const l3_target_obs_t *second);
/* One post-impact frame's targets. Returns 1 when a point was appended. */
int32_t l3_ball_track_update(l3_ball_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                             uint32_t frame, uint32_t timestampUs);
/* The same, beside the club track: clubIndex is the target l3_track_follow
 * claimed this frame (L3_TRACK_NO_TARGET for none). With useHypotheses the
 * ball is searched for with the hypotheses and, once one is classified, its
 * points seed the track (angles included) and tracking continues; the plain
 * update is this with no claim. Built with L3_BALL_HYPOTHESES at 0,
 * useHypotheses is ignored and this is the plain update. */
int32_t l3_ball_track_update_joint(l3_ball_track_t *track, const l3_target_obs_t *targets,
                                   uint32_t n, uint32_t frame, uint32_t timestampUs,
                                   uint32_t clubIndex);
/* sizeof(l3_ball_track_t), for the ctypes mirror's layout check. */
uint32_t l3_ball_track_struct_bytes(void);
/* Angles for the point the last update appended; see l3_track_set_angles. */
int32_t l3_ball_track_set_angles(l3_ball_track_t *track, float azimuthRad, float elevationRad,
                                 uint8_t anglesValid, float angleConfidence);
/* The launch SPEED from the earliest cfg.launchPoints confirmed points (at
 * least 3); cheap enough for every post frame. The direction is not fitted
 * here (hlaValid, vlaValid 0): l3_ball_track_reconstruct does that once.
 * Returns the points used, 0 when too few. */
uint32_t l3_ball_track_launch(const l3_ball_track_t *track, l3_launch_t *out);
/* Once per shot: fit the ball's direction from the tee (l3_ball_fit.h), write
 * every held point's reconstruction, and set launch's HLA/VLA (valid only when
 * the fit is), angle fields, launch position (the tee) and, with a valid speed,
 * its velocity along the fitted direction. An unconfirmed track leaves every
 * point unfiltered and the angles invalid. Returns the accepted angles, 0 when
 * the direction is not valid. */
uint32_t l3_ball_track_reconstruct(l3_ball_track_t *track, l3_launch_t *launch);
const char *l3_ball_track_why_name(uint8_t why);
/* "balltrack armed=1 confirmed=1 done=0 why=tracked count=5 origin=47.0 ..." */
int32_t l3_ball_track_format_status(const l3_ball_track_t *track, char *out, uint32_t cap);

#endif /* L3_BALL_TRACK_H */
