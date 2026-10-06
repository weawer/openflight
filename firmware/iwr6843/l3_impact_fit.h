/* IWR6843 impact from the tracks either side of the tee band.
 *
 * Inside the band (l3_band.h) the MTI ridge hides impact; outside it three
 * clean tracks remain: the club approaching (club in), the club carrying on
 * (club out) and the ball leaving (ball out). Each is fitted as a straight
 * line in range against time over the K points nearest the band and solved
 * for the moment it passes the ball's range, with an uncertainty that grows
 * with the extrapolation. Physics checks drop a track that cannot be what it
 * claims; the survivors are fused by inverse variance and their agreement is
 * the confidence. See docs/superpowers/specs/
 * 2026-09-28-iwr-impact-back-interpolation-design.md. Pure C, no hardware.
 */
#ifndef L3_IMPACT_FIT_H
#define L3_IMPACT_FIT_H

#include <stdint.h>

#include "l3_club_track.h"

#define L3_FIT_MAX_POINTS 8U
#define L3_FIT_NO_TRACK   0xFFU

enum { L3_FIT_CLUB_IN = 0, L3_FIT_CLUB_OUT, L3_FIT_BALL_OUT, L3_FIT_TRACKS };

enum {
    L3_FIT_WHY_OK = 0,
    L3_FIT_WHY_MISSING,          /* no points */
    L3_FIT_WHY_FEW_POINTS,       /* under minPoints */
    L3_FIT_WHY_WRONG_DIRECTION,  /* not moving downrange */
    L3_FIT_WHY_SPEED_BOUNDS,     /* outside this track's speed bounds */
    L3_FIT_WHY_PHYSICS,          /* contradicts another track (smash, club slowing) */
    L3_FIT_WHY_NONFINITE,        /* times do not spread, or the fit overflowed */
    L3_FIT_WHY_DROPPED,          /* the outlier of three */
    L3_FIT_WHY_UNCERTAIN,        /* sigma over maxSigmaUs: too loose to place impact */
    L3_FIT_WHY_SHORT_SPAN,       /* club in spans under minSpanUs: too short to judge */
    L3_FIT_WHY_COUNT
};

enum {
    L3_FIT_VERDICT_NONE = 0,
    L3_FIT_VERDICT_SINGLE,
    L3_FIT_VERDICT_CONSISTENT,
    L3_FIT_VERDICT_INCONSISTENT,
    L3_FIT_VERDICT_COUNT
};

typedef struct {
    float    binWidthM;
    float    bandBins;          /* the tee band's total width in bins (0 = off) */
    uint32_t fitPoints;         /* K nearest the band */
    uint32_t minPoints;
    float    clubMinMps;        /* club in */
    float    clubMaxMps;        /* club in and club out */
    float    clubOutMaxRatio;   /* club out no faster than club in times this */
    float    ballMinMps;
    float    ballMaxMps;
    float    gateSigmas;        /* agreement gate ... */
    float    minSigmaUs;        /* ... on at least this sigma */
    float    maxSigmaUs;        /* a looser estimate is uncertain; 0 disables */
    float    bandSearchBins;    /* the band may slide this far off the tee bin */
    /* Before impact, drop a club target that does not beat its bin's clutter
     * (the band's noise map, learned with no club track) by this many
     * spreads (l3_band_clutter_filter); 0 turns it off. */
    float    clutterSigmas;
    /* Club in only (the self-trigger): the fit takes fitPoints, then older
     * points until first to last spans at least this, up to
     * L3_FIT_MAX_POINTS, so its speed is no noisier at 2 ms than at 3 ms. The
     * outs keep fitPoints: their tracks are short and a later point may be a
     * standing return (20260927_144341: the ball stuck at 52.6). 0 keeps
     * fitPoints alone. */
    uint32_t fitSpanUs;
    /* Club in only (the self-trigger): a fit whose points span less than this
     * first to last is short_span, judged neither for speed nor for impact.
     * At 2 ms a new track's three points span 4 ms, and range jitter alone
     * reads 20-57 m/s: on the bench (2026-10) such tracks, the club at
     * address, fired the trigger at takeaway past both clubMinMps and the
     * impact rule's endMinMps. 0 turns it off. */
    uint32_t minSpanUs;
} l3_impact_fit_cfg_t;

typedef struct {
    uint8_t  why;               /* L3_FIT_WHY_* */
    uint32_t points;            /* points fitted (or offered, when too few) */
    float    timeUs;            /* when the line passes the ball's range */
    float    sigmaUs;
    float    speedMps;          /* range rate, positive downrange */
} l3_fit_estimate_t;

typedef struct {
    l3_fit_estimate_t track[L3_FIT_TRACKS];
    uint8_t  verdict;           /* L3_FIT_VERDICT_* */
    uint8_t  droppedTrack;      /* L3_FIT_NO_TRACK or the dropped index */
    uint8_t  noLock;            /* the configured tee stood in for the ball */
    float    impactUs;          /* 0 with verdict none */
    float    spreadUs;          /* max - min of the estimates kept */
    float    refinedMinusTriggerUs;
} l3_impact_fit_t;

/* A point list read by index: an array ... */
typedef struct {
    const l3_track_point_t *points;
    uint32_t count;
} l3_fit_list_t;
/* ... or a run of a track's held points, oldest first. */
typedef struct {
    const l3_club_track_t *track;
    uint32_t first;
    uint32_t count;
} l3_fit_span_t;

void l3_impact_fit_cfg_defaults(l3_impact_fit_cfg_t *cfg);
/* Every track missing, verdict none, nothing dropped. */
void l3_impact_fit_reset(l3_impact_fit_t *fit);
/* l3_point_at_fn readers for the two kinds of list. */
int32_t l3_fit_list_point(const void *ctx, uint32_t index, l3_track_point_t *out);
int32_t l3_fit_span_point(const void *ctx, uint32_t index, l3_track_point_t *out);
/* The track's points appended after afterFrame (its follow-through), less a
 * newest point still tentative (l3_track_follow). */
void l3_fit_span_after(const l3_club_track_t *track, uint32_t afterFrame, l3_fit_span_t *out);
/* One track: the last fitPoints of count for club in, the first fitPoints for
 * club out and ball out; out is fully written. */
void l3_impact_fit_track(const l3_impact_fit_cfg_t *cfg, uint8_t which, l3_point_at_fn pointAt,
                         const void *ctx, uint32_t count, float ballRangeM,
                         l3_fit_estimate_t *out);

/* Physics across the tracks, then fusion: inverse-variance mean; each kept
 * estimate must lie within gateSigmas * max(sigma, minSigmaUs) of it. Three
 * that do not all agree: every leave-one-out pair is judged around its own
 * mean; the agreeing pair with the smallest disagreement (ties: the pair
 * leaving out club_in, then club_out, then ball_out) is fused and the third
 * dropped if it fails its gate around that pair's mean. Two that disagree, or
 * three with no such pair: inconsistent, impact from the smallest sigma. */
void l3_impact_fit_solve(const l3_impact_fit_cfg_t *cfg, l3_impact_fit_t *fit, uint32_t triggerUs);
/* All three tracks and the solve. A NULL list or span is a missing track. */
void l3_impact_fit_run(const l3_impact_fit_cfg_t *cfg, const l3_fit_list_t *clubIn,
                       const l3_fit_span_t *clubOut, const l3_fit_span_t *ballOut,
                       float ballRangeM, uint8_t noLock, uint32_t triggerUs,
                       l3_impact_fit_t *fit);
/* A float time in us rounded half up to a uint32 timestamp. 0 for a time that
 * is not finite, not positive or past a second uint32 wrap; a time in
 * [2^32, 2^33) (a fit across the wrap) folds back by 2^32. Exact for every
 * float: the fraction is taken after truncation, not by adding 0.5F. */
uint32_t l3_round_us(float us);
const char *l3_impact_fit_why_name(uint8_t why);
const char *l3_impact_fit_verdict_name(uint8_t verdict);
/* "impactfit verdict=consistent t=30000 spreadus=12 dtrigus=-2500 dropped=- nolock=0
 *  club_in=ok:30000+-120 club_out=ok:30001+-140 ball_out=ok:29999+-60" */
int32_t l3_impact_fit_format(const l3_impact_fit_t *fit, char *out, uint32_t cap);

#endif /* L3_IMPACT_FIT_H */
