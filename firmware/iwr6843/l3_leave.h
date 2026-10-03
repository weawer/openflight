/* IWR6843 ball-leave fallback: fire the self-trigger on the ball leaving.
 *
 * The club rules (l3_impact.h) fire from the club's approach. When the club
 * is not seen before launch (the early 2026-08-09 captures lose it 3-5 frames
 * out), nothing fires. Before impact the tee band hides the ball, so the first
 * return to stand beyond the band's far edge, starting within startBins of it
 * and stepping outward on the next frame at a ball's speed, is the ball: on
 * every labelled swing the ball goes first and the club follows 1-10 frames
 * later, slower (0.3-1.7 bins a frame against the ball's 1.5-3.6). Impact is
 * dated by running that two-point line back to the ball's rest bin. It fires
 * about two frames after launch, while the ball is still inside the ball
 * tracker's origin gate.
 *
 * The returns it watches are its own (l3_leave_targets). The trigger's floor
 * is learned where the club swings through, and the leaving ball stands at
 * only 0.4-1.2 times it; against the median of the bins beyond the band it
 * stands at 4-18 times (5th percentile 6.9). Standing clutter out there can
 * be as strong, which is why it takes a step outward at a ball's speed, and
 * noise over a quiet stretch's median can hop between bins as fast. So it is
 * armed only for clubHoldFrames after the club was near the ball
 * (l3_leave_club_near): the ball leaves once the club reaches it, 5-8 frames
 * after the club was last seen on the rescued swings, 4-7.4 bins short of the
 * band. Armed on the approach alone, a ridge left beyond a band still centred
 * on a wrong tee fired mid-downswing; so too a start from nothing must be a
 * return with nothing within newBins of it on the frame before, where the
 * ridge always has itself. Pure C, no hardware.
 */
#ifndef L3_LEAVE_H
#define L3_LEAVE_H

#include <stdint.h>

#include "l3_observation.h"

/* Bins beyond the edge needed to take their median. */
#define L3_LEAVE_MIN_FLOOR_BINS 6U
/* The club track counts as approaching when active with this many points:
 * one association, so it has moved. */
#define L3_LEAVE_CLUB_MIN_POINTS 2U

typedef struct {
    float startBins;      /* the first return lies within this beyond the band's edge */
    float minSpeedMps;    /* the step outward is a ball's: at least this */
    float maxSpeedMps;    /* and at most this */
    float binWidthM;      /* range bin width */
    float snr;            /* a return stands this far over the median beyond the edge */
    uint32_t clubHoldFrames; /* armed this many frames after the club was near the ball */
    float clubNearBins;   /* near: the club's newest point within this of the band's near edge */
    float newBins;        /* a start from nothing had no return within this last frame */
    /* A step under minStepBins is judged only minStepUs or more after the
     * start: at 2 ms a bin's range wobble reads 23 m/s over one frame, past
     * minSpeedMps, but 12 m/s over two (2026-10-03). A step of minStepBins
     * or more is judged at once: at 2 ms a ball may show beyond the band on
     * only two frames. At 3 ms every step is judged on the next frame, as
     * before. minStepUs 0: every step on the next frame. */
    uint32_t minStepUs;
    float minStepBins;
} l3_leave_cfg_t;

enum {
    L3_LEAVE_WHY_NONE = 0,
    L3_LEAVE_WHY_NOCLUB,    /* the club not near the ball within clubHoldFrames */
    L3_LEAVE_WHY_IDLE,      /* nothing beyond the band's edge */
    L3_LEAVE_WHY_FAR,       /* the nearest return beyond it starts too far out */
    L3_LEAVE_WHY_STOOD,     /* it stood within newBins last frame: a ridge, not a ball */
    L3_LEAVE_WHY_STARTED,   /* a return just beyond the edge: the ball, if it moves */
    L3_LEAVE_WHY_SLOW,      /* nothing stepped out at a ball's speed: restarted */
    L3_LEAVE_WHY_FIRED,
    L3_LEAVE_WHY_COUNT
};

typedef struct {
    l3_leave_cfg_t cfg;
    uint8_t   fired;
    uint8_t   why;                /* last update */
    uint8_t   started;            /* startBin/startUs hold the first return */
    uint32_t  clubHold;           /* frames the rule stays armed, counting down */
    uint32_t  prevCount;          /* the last frame's returns, for a new start */
    float     prevBins[L3_OBS_MAX_TARGETS];
    float     startBin;
    uint32_t  startUs;
    float     speedMps;           /* the step that fired */
    uint32_t  impactTimestampUs;  /* the line run back to the rest bin, when fired */
    l3_target_obs_t startTarget;  /* the ball's two points, to seed the ball tracker */
    l3_target_obs_t stepTarget;
    uint32_t  counters[L3_LEAVE_WHY_COUNT];
} l3_leave_t;

void l3_leave_cfg_defaults(l3_leave_cfg_t *cfg);
void l3_leave_init(l3_leave_t *leave, const l3_leave_cfg_t *cfg);
/* Forget a fire and a start so the rule can fire again; counters survive. */
void l3_leave_rearm(l3_leave_t *leave);
const char *l3_leave_why_name(uint8_t why);
/* 1 when the club track (active, count points, newest at newestBin) is an
 * approach within cfg->clubNearBins of the band's near edge loBin, or past it. */
uint8_t l3_leave_club_near(const l3_leave_cfg_t *cfg, uint8_t active, uint32_t count,
                           float newestBin, float loBin);
/* The returns the rule watches, from one pre-impact frame's whole-window
 * observations (global first bin firstBin, count bins): targets beyond
 * edgeBin whose statistic (params->stat) is at least cfg->snr times the
 * median statistic of the bins beyond it. Fewer than L3_LEAVE_MIN_FLOOR_BINS
 * bins beyond the edge give none. Returns how many were written to out;
 * *floor (when not NULL) receives the median, the noise of the stretch the
 * ball flies into, which the ball tracker's 16-bin post window freezes as its
 * floor (l3_scan.h); left untouched when there were too few bins. */
uint32_t l3_leave_targets(const l3_leave_cfg_t *cfg, const l3_obs_params_t *params,
                          const l3_bin_obs_t *obs, uint32_t firstBin, uint32_t count,
                          uint32_t frame, uint32_t timestampUs, float edgeBin,
                          l3_target_obs_t *out, uint32_t maxOut, float *floor);
/* One pre-impact frame's l3_leave_targets: edgeBin is the band's far edge,
 * originBin the ball's rest bin, clubNear l3_leave_club_near for this frame.
 * Returns 1 on the update that fires; later updates are ignored until
 * l3_leave_rearm. */
int32_t l3_leave_update(l3_leave_t *leave, const l3_target_obs_t *targets, uint32_t n,
                        float edgeBin, float originBin, uint8_t clubNear);
/* "leave fired=1 why=fired start=47.00 speed=43.75 t=25714 fired_n=1" */
int32_t l3_leave_format(const l3_leave_t *leave, char *out, uint32_t cap);

#endif /* L3_LEAVE_H */
