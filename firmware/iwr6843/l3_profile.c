/* See l3_profile.h. */
#include <stdio.h>
#include <string.h>

#include "l3_profile.h"

static const char *const kStageNames[L3_PROF_STAGE_COUNT] = {
    "residual", "trigger", "extract", "clubtrack", "angle", "impact", "balldetect", "balltrack",
    "reconstruct", "dspwait"
};

void l3_profile_init(l3_profile_t *profile, uint32_t ticksPerUs)
{
    memset(profile, 0, sizeof(*profile));
    profile->ticksPerUs = (ticksPerUs == 0U) ? 1U : ticksPerUs;
}

void l3_profile_reset(l3_profile_t *profile)
{
    uint32_t ticksPerUs = profile->ticksPerUs;

    l3_profile_init(profile, ticksPerUs);
}

void l3_profile_add(l3_profile_t *profile, uint32_t stage, uint32_t ticks)
{
    l3_profile_stage_t *s;

    if (stage >= L3_PROF_STAGE_COUNT) {
        return;
    }
    s = &profile->stage[stage];
    s->count++;
    s->lastTicks = ticks;
    if (ticks > s->maxTicks) {
        s->maxTicks = ticks;
    }
    if (0xFFFFFFFFU - s->sumTicks < ticks) {
        s->sumTicks = 0xFFFFFFFFU;
        s->sumOverflow = 1U;
    } else {
        s->sumTicks += ticks;
    }
}

void l3_profile_frame(l3_profile_t *profile)
{
    profile->frames++;
}

uint32_t l3_profile_mean_us(const l3_profile_t *profile, uint32_t stage)
{
    const l3_profile_stage_t *s;

    if (stage >= L3_PROF_STAGE_COUNT) {
        return 0U;
    }
    s = &profile->stage[stage];
    if (s->count == 0U) {
        return 0U;
    }
    return (s->sumTicks / s->count) / profile->ticksPerUs;
}

uint32_t l3_profile_max_us(const l3_profile_t *profile, uint32_t stage)
{
    if (stage >= L3_PROF_STAGE_COUNT) {
        return 0U;
    }
    return profile->stage[stage].maxTicks / profile->ticksPerUs;
}

uint32_t l3_profile_frame_us(const l3_profile_t *profile)
{
    uint32_t total = 0U;
    uint32_t stage;

    for (stage = 0U; stage < L3_PROF_STAGE_COUNT; stage++) {
        if (stage == L3_PROF_DSP_WAIT || stage == L3_PROF_RECONSTRUCT) {
            continue; /* dspwait is inside residual; reconstruct is once per shot */
        }
        total += l3_profile_mean_us(profile, stage);
    }
    return total;
}

const char *l3_profile_stage_name(uint32_t stage)
{
    return (stage < L3_PROF_STAGE_COUNT) ? kStageNames[stage] : "?";
}

int32_t l3_profile_format(const l3_profile_t *profile, uint32_t stage, char *out, uint32_t cap)
{
    const l3_profile_stage_t *s;

    if (stage >= L3_PROF_STAGE_COUNT) {
        return snprintf(out, cap, "perf ?");
    }
    s = &profile->stage[stage];
    return snprintf(out, cap, "perf %s n=%u last=%u mean=%u%s max=%u", kStageNames[stage],
                    (unsigned)s->count, (unsigned)(s->lastTicks / profile->ticksPerUs),
                    (unsigned)l3_profile_mean_us(profile, stage), s->sumOverflow ? "+" : "",
                    (unsigned)l3_profile_max_us(profile, stage));
}

int32_t l3_profile_format_summary(const l3_profile_t *profile, char *out, uint32_t cap)
{
    return snprintf(out, cap, "perf frames=%u total=%uus clock=%u", (unsigned)profile->frames,
                    (unsigned)l3_profile_frame_us(profile), (unsigned)profile->ticksPerUs);
}
