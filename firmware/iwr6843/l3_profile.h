/* IWR6843 per-stage profiling counters.
 *
 * Before anything moves to the HWA or the DSP, the MSS must know what each
 * stage costs on real frames: the vertical residual, the trigger update,
 * target extraction, the club and ball track updates, angle estimation, the
 * impact test and the ball detector; the once-per-shot reconstruction (the ball
 * direction fit at RESULT) as "reconstruct"; and, inside
 * the residual, how long the MSS waited on the DSS to score (dspwait). l3_dump.c wraps each with a cycle
 * counter read (Cycleprofiler_getTimeStamp) and adds the difference here;
 * "triggerLog perf" prints count, last, mean and maximum per stage in
 * microseconds. The counters cost a few instructions per stage per frame
 * and stay on for the check suite to read. Pure C, no hardware.
 */
#ifndef L3_PROFILE_H
#define L3_PROFILE_H

#include <stdint.h>

enum {
    L3_PROF_RESIDUAL = 0,
    L3_PROF_TRIGGER,
    L3_PROF_EXTRACT,
    L3_PROF_CLUB_TRACK,
    L3_PROF_ANGLE,
    L3_PROF_IMPACT,
    L3_PROF_BALL_DETECT,
    L3_PROF_BALL_TRACK,
    L3_PROF_RECONSTRUCT,  /* once per shot: the ball's direction fit at RESULT */
    L3_PROF_DSP_WAIT,     /* the MSS blocked on the DSS's answer (l3_detect_core.h) */
    L3_PROF_STAGE_COUNT
};

typedef struct {
    uint32_t count;
    uint32_t lastTicks;
    uint32_t maxTicks;
    uint32_t sumTicks;     /* saturates rather than wrapping */
    uint32_t sumOverflow;  /* set once sumTicks saturated; mean is then a floor */
} l3_profile_stage_t;

typedef struct {
    uint32_t ticksPerUs;
    uint32_t frames;       /* frames profiled since reset */
    l3_profile_stage_t stage[L3_PROF_STAGE_COUNT];
} l3_profile_t;

void l3_profile_init(l3_profile_t *profile, uint32_t ticksPerUs);
/* Zero the counters, keep the clock. */
void l3_profile_reset(l3_profile_t *profile);
void l3_profile_add(l3_profile_t *profile, uint32_t stage, uint32_t ticks);
/* One frame's worth of stages has been added. */
void l3_profile_frame(l3_profile_t *profile);
uint32_t l3_profile_mean_us(const l3_profile_t *profile, uint32_t stage);
uint32_t l3_profile_max_us(const l3_profile_t *profile, uint32_t stage);
/* Sum of every stage's mean except dspwait (inside residual) and reconstruct
 * (once per shot): the per-frame budget the MSS is spending. */
uint32_t l3_profile_frame_us(const l3_profile_t *profile);
const char *l3_profile_stage_name(uint32_t stage);
/* "perf residual n=1200 last=310 mean=298 max=512" (microseconds) */
int32_t l3_profile_format(const l3_profile_t *profile, uint32_t stage, char *out, uint32_t cap);
/* "perf frames=1200 total=812us clock=200" */
int32_t l3_profile_format_summary(const l3_profile_t *profile, char *out, uint32_t cap);

#endif /* L3_PROFILE_H */
