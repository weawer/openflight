/* IWR6843 ball-flight confirmation: the self-trigger fires only on a ball.
 *
 * The club rules (l3_impact.h) and the ball-leave fallback (l3_leave.h) say
 * when the club reached the ball, but at the kiosk's snr 1 a brief real mover
 * (the takeaway, the hands, the golfer shifting) chained through noise points
 * to the ball reads as a 40-70 m/s approach and fires just the same. The
 * 2026-10-03 backswing sessions fired on 41 swings; 15 of 36 TrackMan shots on
 * 2026-10-04 fired at the takeaway. Each false fire costs ~7.8 s of readback
 * and restart in which the real swing is lost. Neither a backswing nor an
 * empty lane launches a ball.
 *
 * So in confirm mode a club or leave fire is a candidate: the ring freezes as
 * before and the post movie fills, but the host is told only once the ball
 * tracker holds a ball flight, and told to release the ring if none shows
 * within windowUs. The OPS keeps ~100 ms before its S!, so the few frames the
 * flight takes to show cost it nothing.
 *
 * A flight is `points` consecutive ball-track points, each step outward at a
 * ball's speed, the aliased Doppler running on smoothly from point to point,
 * and the run's fitted range rate wrapping onto its Doppler: a real target's
 * Doppler is its range rate modulo the alias span, noise reads anything. On
 * the labelled swings, the TrackMan set and the backswing captures replayed at
 * the kiosk's settings no false fire showed one (firmware_replay,
 * tests/test_iwr6843_trigger_confirm_replay.py).
 *
 * Pure C, no hardware: tests/test_iwr6843_firmware_confirm.py builds it with
 * the host compiler.
 */
#ifndef L3_CONFIRM_H
#define L3_CONFIRM_H

#include <stdint.h>

#include "l3_club_track.h"

/* "trackCfg confirm"'s limits. */
#define L3_CONFIRM_MIN_POINTS     3U
#define L3_CONFIRM_MAX_POINTS     8U
#define L3_CONFIRM_MAX_WINDOW_US  100000U

typedef struct {
    uint32_t enabled;         /* 0: a candidate fires at once, as before */
    uint32_t windowUs;        /* the flight must show within this of the candidate */
    uint32_t points;          /* consecutive ball points a flight needs */
    float    minSpeedMps;     /* every step outward at least this ... */
    float    maxSpeedMps;     /* ... and at most this */
    float    dopplerStepMps;  /* wrapped Doppler change point to point at most this */
    float    rateDopplerMps;  /* fitted range rate onto the Doppler within this; 0 = off */
    float    binWidthM;
    float    velocitySpanMps; /* the Doppler alias span */
} l3_confirm_cfg_t;

enum {
    L3_CONFIRM_IDLE = 0,      /* no candidate */
    L3_CONFIRM_PENDING,       /* a candidate waits for its flight */
    L3_CONFIRM_CONFIRMED,     /* the ball flew: tell the host */
    L3_CONFIRM_REJECTED,      /* no flight: release the ring */
    L3_CONFIRM_VERDICT_COUNT
};

enum {
    L3_CONFIRM_WHY_NONE = 0,
    L3_CONFIRM_WHY_FEW,       /* fewer ball points than a flight needs */
    L3_CONFIRM_WHY_STILL,     /* a step not outward at a ball's speed */
    L3_CONFIRM_WHY_FAST,      /* a step faster than any ball */
    L3_CONFIRM_WHY_JUMP,      /* the Doppler jumped between points */
    L3_CONFIRM_WHY_RATE,      /* the range rate does not wrap onto the Doppler */
    L3_CONFIRM_WHY_FLIGHT,    /* confirmed */
    L3_CONFIRM_WHY_TIMEOUT,   /* windowUs passed without a flight */
    L3_CONFIRM_WHY_ENDED,     /* the post movie ended without a flight */
    L3_CONFIRM_WHY_COUNT
};

typedef struct {
    l3_confirm_cfg_t cfg;
    uint8_t  verdict;         /* L3_CONFIRM_* */
    uint8_t  why;             /* the last update's reason */
    uint32_t candidateUs;     /* the candidate's impact time */
    uint32_t decidedUs;       /* the frame time of the verdict */
    float    speedMps;        /* the confirmed flight's fitted range rate */
    uint32_t confirmed;       /* counters since init */
    uint32_t rejected;
} l3_confirm_t;

void l3_confirm_cfg_defaults(l3_confirm_cfg_t *cfg);
/* Reject a configuration the rule cannot run. Returns 0 when usable. */
int32_t l3_confirm_cfg_check(const l3_confirm_cfg_t *cfg);
void l3_confirm_init(l3_confirm_t *confirm, const l3_confirm_cfg_t *cfg);
/* Back to idle for the next shot; the counters survive. */
void l3_confirm_rearm(l3_confirm_t *confirm);
/* A club or leave fire dated candidateUs: wait for its flight. */
void l3_confirm_arm(l3_confirm_t *confirm, uint32_t candidateUs);
/* One post-impact frame at nowUs: the ball track's count points (oldest
 * first, read through pointAt) confirm the candidate, or the window passes
 * and rejects it. Only a pending candidate changes; returns the verdict. */
uint8_t l3_confirm_update(l3_confirm_t *confirm, l3_point_at_fn pointAt, const void *ctx,
                          uint32_t count, uint32_t nowUs);
/* The post movie ended at nowUs: a candidate still pending is rejected.
 * Returns the verdict. */
uint8_t l3_confirm_end(l3_confirm_t *confirm, uint32_t nowUs);
const char *l3_confirm_verdict_name(uint8_t verdict);
const char *l3_confirm_why_name(uint8_t why);
/* "confirm on=1 verdict=confirmed why=flight speed=48.20 dt=6000 confirmed_n=1 rejected_n=0" */
int32_t l3_confirm_format(const l3_confirm_t *confirm, char *out, uint32_t cap);

#endif /* L3_CONFIRM_H */
