/* See l3_leave.h. */
#include <stdio.h>
#include <string.h>

#include "l3_impact_fit.h"
#include "l3_leave.h"
#include "l3_text.h"

static const char *const kWhyNames[L3_LEAVE_WHY_COUNT] = {
    "none", "noclub", "idle", "far", "stood", "started", "slow", "fired"
};

void l3_leave_cfg_defaults(l3_leave_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->startBins = 4.0F;       /* the ball clears the band's edge by 1-3 bins a frame */
    cfg->minSpeedMps = 22.0F;    /* a return jittering beyond the band walked out at 20.3 */
    cfg->maxSpeedMps = 90.0F;    /* well past any ball: a jump, not a flight */
    cfg->binWidthM = 6.0F / 128.0F;
    cfg->snr = 6.0F;             /* 4-6 fire the same labelled swings: the most margin */
    cfg->clubHoldFrames = 10U;   /* the ball left 5-8 frames after the club was last seen */
    cfg->clubNearBins = 10.0F;   /* last seen 4-7.4 bins short of the band */
    cfg->newBins = 1.0F;         /* a standing ridge jitters about a bin a frame */
}

uint8_t l3_leave_club_near(const l3_leave_cfg_t *cfg, uint8_t active, uint32_t count,
                           float newestBin, float loBin)
{
    return (active && count >= L3_LEAVE_CLUB_MIN_POINTS &&
            loBin - newestBin <= cfg->clubNearBins) ? 1U : 0U;
}

void l3_leave_init(l3_leave_t *leave, const l3_leave_cfg_t *cfg)
{
    memset(leave, 0, sizeof(*leave));
    leave->cfg = *cfg;
}

void l3_leave_rearm(l3_leave_t *leave)
{
    leave->fired = 0U;
    leave->why = L3_LEAVE_WHY_NONE;
    leave->started = 0U;
    leave->clubHold = 0U;
    leave->prevCount = 0U;
    leave->startBin = 0.0F;
    leave->startUs = 0U;
    leave->speedMps = 0.0F;
    leave->impactTimestampUs = 0U;
}

static int32_t l3_leave_note(l3_leave_t *leave, uint8_t why)
{
    leave->why = why;
    leave->counters[why]++;
    return (why == L3_LEAVE_WHY_FIRED) ? 1 : 0;
}

uint32_t l3_leave_targets(const l3_leave_cfg_t *cfg, const l3_obs_params_t *params,
                          const l3_bin_obs_t *obs, uint32_t firstBin, uint32_t count,
                          uint32_t frame, uint32_t timestampUs, float edgeBin,
                          l3_target_obs_t *out, uint32_t maxOut, float *floor)
{
    l3_obs_params_t own = *params;
    uint32_t skip;
    uint32_t beyond;
    float median;

    /* The first local bin strictly beyond the edge. */
    skip = (edgeBin < (float)firstBin) ? 0U : (uint32_t)edgeBin + 1U - firstBin;
    if (skip >= count || count - skip < L3_LEAVE_MIN_FLOOR_BINS) {
        return 0U;
    }
    beyond = count - skip;
    if (beyond > L3_OBS_MAX_BINS) {
        beyond = L3_OBS_MAX_BINS;
    }
    median = l3_obs_median(own.stat, &obs[skip], beyond);
    if (floor != NULL) {
        *floor = median;
    }
    own.snr = cfg->snr;
    return l3_obs_extract(&own, frame, timestampUs, firstBin + skip, &obs[skip], beyond, median,
                          out, maxOut);
}

/* The nearest return strictly beyond the edge, or -1. */
static int32_t l3_leave_nearest(const l3_target_obs_t *targets, uint32_t n, float edgeBin)
{
    int32_t best = -1;
    uint32_t i;

    for (i = 0U; i < n; i++) {
        if (targets[i].rangeBin > edgeBin &&
            (best < 0 || targets[i].rangeBin < targets[best].rangeBin)) {
            best = (int32_t)i;
        }
    }
    return best;
}

/* 1 when a return stood within newBins of bin on the frame before. */
static uint8_t l3_leave_stood(const l3_leave_t *leave, float bin)
{
    uint32_t i;

    for (i = 0U; i < leave->prevCount; i++) {
        float gap = bin - leave->prevBins[i];

        if (gap <= leave->cfg.newBins && gap >= -leave->cfg.newBins) {
            return 1U;
        }
    }
    return 0U;
}

/* Start from the nearest return beyond the edge when it is close to it.
 * Returns the verdict: started or far. */
static uint8_t l3_leave_start(l3_leave_t *leave, const l3_target_obs_t *nearest, float edgeBin)
{
    leave->started = 0U;
    if (nearest->rangeBin - edgeBin > leave->cfg.startBins) {
        return L3_LEAVE_WHY_FAR;
    }
    leave->started = 1U;
    leave->startBin = nearest->rangeBin;
    leave->startUs = nearest->timestampUs;
    leave->startTarget = *nearest;
    return L3_LEAVE_WHY_STARTED;
}

/* This frame's verdict; fires by setting leave->fired. */
static uint8_t l3_leave_step(l3_leave_t *leave, const l3_target_obs_t *targets, uint32_t n,
                             float edgeBin, float originBin, uint8_t clubNear)
{
    int32_t nearest;
    uint32_t i;

    /* Armed only once the club has reached the ball. */
    if (clubNear) {
        leave->clubHold = leave->cfg.clubHoldFrames;
    } else if (leave->clubHold > 0U) {
        leave->clubHold--;
    }
    if (leave->clubHold == 0U) {
        leave->started = 0U;
        return L3_LEAVE_WHY_NOCLUB;
    }
    nearest = l3_leave_nearest(targets, n, edgeBin);
    if (nearest < 0) {
        leave->started = 0U;
        return L3_LEAVE_WHY_IDLE;
    }
    if (leave->started) {
        for (i = 0U; i < n; i++) {
            /* Wrap-safe: the difference of two timestamps. */
            float dtS = (float)(int32_t)(targets[i].timestampUs - leave->startUs) * 1.0e-6F;
            float stepBins = targets[i].rangeBin - leave->startBin;
            float speed;

            if (dtS <= 0.0F || stepBins <= 0.0F) {
                continue;
            }
            speed = stepBins * leave->cfg.binWidthM / dtS;
            if (speed >= leave->cfg.minSpeedMps && speed <= leave->cfg.maxSpeedMps) {
                /* The ball: run its line back to the rest bin. */
                float backS = (leave->startBin - originBin) * leave->cfg.binWidthM / speed;

                leave->fired = 1U;
                leave->speedMps = speed;
                leave->stepTarget = targets[i];
                leave->impactTimestampUs = leave->startUs - l3_round_us(backS * 1.0e6F);
                return L3_LEAVE_WHY_FIRED;
            }
        }
        /* Nothing moved out at a ball's speed: the newest return continues it. */
        (void)l3_leave_start(leave, &targets[nearest], edgeBin);
        return L3_LEAVE_WHY_SLOW;
    }
    /* From nothing, only something new: not a return that stood there. */
    if (l3_leave_stood(leave, targets[nearest].rangeBin)) {
        return L3_LEAVE_WHY_STOOD;
    }
    return l3_leave_start(leave, &targets[nearest], edgeBin);
}

int32_t l3_leave_update(l3_leave_t *leave, const l3_target_obs_t *targets, uint32_t n,
                        float edgeBin, float originBin, uint8_t clubNear)
{
    uint8_t why;
    uint32_t i;

    if (leave->fired) {
        return 0;
    }
    why = l3_leave_step(leave, targets, n, edgeBin, originBin, clubNear);
    /* Remembered armed or not: the next start is judged against it. */
    leave->prevCount = (n < L3_OBS_MAX_TARGETS) ? n : L3_OBS_MAX_TARGETS;
    for (i = 0U; i < leave->prevCount; i++) {
        leave->prevBins[i] = targets[i].rangeBin;
    }
    return l3_leave_note(leave, why);
}

const char *l3_leave_why_name(uint8_t why)
{
    return (why < L3_LEAVE_WHY_COUNT) ? kWhyNames[why] : "?";
}

int32_t l3_leave_format(const l3_leave_t *leave, char *out, uint32_t cap)
{
    char startText[16];
    char speedText[16];

    l3_text_fixed2(leave->startBin, startText, sizeof(startText));
    l3_text_fixed2(leave->speedMps, speedText, sizeof(speedText));
    return snprintf(out, cap, "leave fired=%u why=%s start=%s speed=%s t=%u fired_n=%u",
                    (unsigned)leave->fired, l3_leave_why_name(leave->why), startText,
                    speedText, (unsigned)leave->impactTimestampUs,
                    (unsigned)leave->counters[L3_LEAVE_WHY_FIRED]);
}
