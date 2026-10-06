/* IWR6843 range-only impact: what fires the self-trigger.
 *
 * The club track's club-in line (l3_impact_fit_track, fitted to its range
 * points) crosses the ball's range at a time between frames. Impact is
 * declared when that crossing lies within a horizon of the current frame's
 * time, which also dates impact between frames rather than to a frame index.
 * The club coasts across the tee band, so the newest point stops advancing
 * and the frame clock must.
 *
 * The crossing alone often never comes: on the 34 labelled swings (2026-09-30)
 * the club's radar range when the ball leaves is 3-12 bins (median 7.4) short
 * of the ball's, because at impact its return merges with the ball's and the
 * club track stops taking points. So impact also fires on the frame an
 * approaching track (a usable club-in estimate) takes no point, having last
 * been seen within endM short of the ball's range, dated to that last point.
 * The approach must also be at least endMinMps: on the bench (2026-10) a
 * backswing's downrange crossing (the club-in fit takes 17 m/s and up)
 * armed it and fired 0.5-0.8 s before impact; every approach that armed it
 * on the labelled swings was 20.6 m/s or faster. A point past the ball's
 * range never arms it: before impact the club is short of the ball.
 *
 * A geometric detector once sat beside it, judging the club's 3D line against
 * the ball's position; it was removed on 2026-09-30 with the range gate: the
 * kiosk never armed it and it never fired on the recorded swings. Pure C, no
 * hardware.
 */
#ifndef L3_IMPACT_H
#define L3_IMPACT_H

#include <stdint.h>

#include "l3_impact_fit.h"

typedef struct {
    float horizonS;       /* fire when the crossing is within this of the frame's time */
    float endM;           /* fire when the approach ends within this of the ball; 0 = off */
    float endMinMps;      /* ... at this club-in speed or faster; 0 = any usable */
} l3_impact_cfg_t;

#define L3_IMPACT_END_MAX_M 2.0F     /* "trackCfg impact"'s limit for endM */
#define L3_IMPACT_END_MAX_MPS 70.0F  /* ... and for endMinMps (l3_impact_fit clubMaxMps) */

/* The club track this frame, as the approach-end rule sees it. */
typedef struct {
    uint8_t  appended;    /* the track took a point this frame */
    float    rangeM;      /* its newest point's range (read when appended) */
    uint32_t timeUs;      /* and that point's time */
    float    ballRangeM;  /* the destination's range */
} l3_impact_club_t;

enum {
    L3_IMPACT_CAUSE_NONE = 0,
    L3_IMPACT_CAUSE_CROSSING,    /* the club-in line crossed the ball's range */
    L3_IMPACT_CAUSE_END,         /* the approach ended near the ball */
    L3_IMPACT_CAUSE_COUNT
};

enum {
    L3_IMPACT_WHY_NONE = 0,
    L3_IMPACT_WHY_NO_DELIVERY,   /* no usable club-in estimate */
    L3_IMPACT_WHY_PENDING,       /* crossing still beyond the horizon */
    L3_IMPACT_WHY_PASSED,        /* crossing more than a horizon ago: missed */
    L3_IMPACT_WHY_FIRED,
    L3_IMPACT_WHY_COUNT
};

typedef struct {
    l3_impact_cfg_t cfg;
    uint8_t   fired;
    uint8_t   why;                /* last update */
    uint8_t   cause;              /* what fired, L3_IMPACT_CAUSE_* */
    uint8_t   endArmed;           /* the newest point was an approach that arms the end */
    uint32_t  endTimeUs;          /* that point's time */
    float     offsetS;            /* crossing time relative to the frame (+ ahead) */
    uint32_t  impactTimestampUs;  /* the crossing, or the approach's last point */
    uint32_t  counters[L3_IMPACT_WHY_COUNT];
} l3_impact_t;

void l3_impact_cfg_defaults(l3_impact_cfg_t *cfg);
void l3_impact_init(l3_impact_t *impact, const l3_impact_cfg_t *cfg);
/* Forget a fire so the detector can fire again; counters survive. */
void l3_impact_rearm(l3_impact_t *impact);
const char *l3_impact_why_name(uint8_t why);
const char *l3_impact_cause_name(uint8_t cause);
/* Fire on the club-in estimate when its crossing of the ball's range is
 * within the horizon of nowUs, the current frame's time, or (club not NULL)
 * on the first frame without a point after an approach point at least
 * endMinMps, short of the ball by at most endM. A missing or rejected estimate is nodelivery. Returns 1 on the
 * update that fires; later updates are ignored until l3_impact_rearm. */
int32_t l3_impact_update_range(l3_impact_t *impact, const l3_fit_estimate_t *clubIn,
                               const l3_impact_club_t *club, uint32_t nowUs);
/* "impact fired=1 why=fired cause=end offsetms=-3.00 t=24000 pending=1 passed=0
 *  fired_n=1 armed=1" */
int32_t l3_impact_format(const l3_impact_t *impact, char *out, uint32_t cap);

#endif /* L3_IMPACT_H */
