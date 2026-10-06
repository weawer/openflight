/* See l3_confirm.h. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "l3_confirm.h"
#include "l3_text.h"

static const char *const kVerdictNames[L3_CONFIRM_VERDICT_COUNT] = {
    "idle", "pending", "confirmed", "rejected"
};
static const char *const kWhyNames[L3_CONFIRM_WHY_COUNT] = {
    "none", "few", "still", "fast", "jump", "rate", "flight", "timeout", "ended"
};

void l3_confirm_cfg_defaults(l3_confirm_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->enabled = 0U;
    /* The labelled swings confirm within 24 ms of the candidate when the club
     * rule fired 3-4 frames before launch; longer gains none. */
    cfg->windowUs = 24000U;
    cfg->points = 3U;            /* two steps: a line and a check on it */
    cfg->minSpeedMps = 15.0F;    /* the slowest labelled ball steps ~25 m/s */
    cfg->maxSpeedMps = 90.0F;    /* l3_leave's ceiling: a jump, not a flight */
    cfg->dopplerStepMps = 2.0F;  /* a ball's aliased Doppler moves <= ~0.6 m/s a frame */
    cfg->rateDopplerMps = 5.0F;  /* a 3-point range rate is good to ~2 m/s */
    cfg->binWidthM = 6.0F / 128.0F;
    cfg->velocitySpanMps = 2.0F * 0.00484F / (4.0F * 135.0e-6F);
}

int32_t l3_confirm_cfg_check(const l3_confirm_cfg_t *cfg)
{
    if (cfg->points < L3_CONFIRM_MIN_POINTS || cfg->points > L3_CONFIRM_MAX_POINTS) {
        return -1;
    }
    if (cfg->windowUs == 0U || cfg->windowUs > L3_CONFIRM_MAX_WINDOW_US) {
        return -1;
    }
    if (!(cfg->minSpeedMps > 0.0F) || !(cfg->maxSpeedMps > cfg->minSpeedMps) ||
        !(cfg->dopplerStepMps > 0.0F) || !(cfg->rateDopplerMps >= 0.0F) ||
        !(cfg->binWidthM > 0.0F) || !(cfg->velocitySpanMps > 0.0F)) {
        return -1;
    }
    return 0;
}

void l3_confirm_init(l3_confirm_t *confirm, const l3_confirm_cfg_t *cfg)
{
    memset(confirm, 0, sizeof(*confirm));
    confirm->cfg = *cfg;
}

void l3_confirm_rearm(l3_confirm_t *confirm)
{
    confirm->verdict = L3_CONFIRM_IDLE;
    confirm->why = L3_CONFIRM_WHY_NONE;
    confirm->candidateUs = 0U;
    confirm->decidedUs = 0U;
    confirm->speedMps = 0.0F;
}

void l3_confirm_arm(l3_confirm_t *confirm, uint32_t candidateUs)
{
    l3_confirm_rearm(confirm);
    confirm->verdict = L3_CONFIRM_PENDING;
    confirm->candidateUs = candidateUs;
}

/* Wrap-safe microseconds from a to b. */
static float l3_confirm_dt_s(uint32_t a, uint32_t b)
{
    return (float)(int32_t)(b - a) * 1.0e-6F;
}

/* The run of cfg->points points from index first: L3_CONFIRM_WHY_FLIGHT, or
 * the first rule it breaks. *speed receives the fitted range rate. */
static uint8_t l3_confirm_run(const l3_confirm_cfg_t *cfg, const l3_track_point_t *run,
                              float *speed)
{
    float t[L3_CONFIRM_MAX_POINTS];
    float tMean = 0.0F;
    float rMean = 0.0F;
    float stt = 0.0F;
    float str = 0.0F;
    float rate;
    uint32_t i;

    for (i = 1U; i < cfg->points; i++) {
        float dtS = l3_confirm_dt_s(run[i - 1U].timestampUs, run[i].timestampUs);
        float step;

        if (!(dtS > 0.0F)) {
            return L3_CONFIRM_WHY_STILL;
        }
        step = (run[i].rangeBin - run[i - 1U].rangeBin) * cfg->binWidthM / dtS;
        if (!isfinite(step) || step < cfg->minSpeedMps) {
            return L3_CONFIRM_WHY_STILL;
        }
        if (step > cfg->maxSpeedMps) {
            return L3_CONFIRM_WHY_FAST;
        }
        if (l3_track_wrapped_diff(run[i].dopplerAliasMps, run[i - 1U].dopplerAliasMps,
                                  cfg->velocitySpanMps) > cfg->dopplerStepMps) {
            return L3_CONFIRM_WHY_JUMP;
        }
    }
    /* Least squares range against time over the run. */
    for (i = 0U; i < cfg->points; i++) {
        t[i] = l3_confirm_dt_s(run[0].timestampUs, run[i].timestampUs);
        tMean += t[i];
        rMean += run[i].rangeBin * cfg->binWidthM;
    }
    tMean /= (float)cfg->points;
    rMean /= (float)cfg->points;
    for (i = 0U; i < cfg->points; i++) {
        float dt = t[i] - tMean;

        stt += dt * dt;
        str += dt * (run[i].rangeBin * cfg->binWidthM - rMean);
    }
    rate = (stt > 0.0F) ? str / stt : 0.0F;
    if (!isfinite(rate)) {
        return L3_CONFIRM_WHY_STILL;
    }
    *speed = rate;
    if (cfg->rateDopplerMps > 0.0F &&
        l3_track_wrapped_diff(rate, run[cfg->points - 1U].dopplerAliasMps,
                              cfg->velocitySpanMps) > cfg->rateDopplerMps) {
        return L3_CONFIRM_WHY_RATE;
    }
    return L3_CONFIRM_WHY_FLIGHT;
}

static uint8_t l3_confirm_decide(l3_confirm_t *confirm, uint8_t verdict, uint8_t why,
                                 uint32_t nowUs)
{
    confirm->verdict = verdict;
    confirm->why = why;
    confirm->decidedUs = nowUs;
    if (verdict == L3_CONFIRM_CONFIRMED) {
        confirm->confirmed++;
    } else {
        confirm->rejected++;
    }
    return verdict;
}

uint8_t l3_confirm_update(l3_confirm_t *confirm, l3_point_at_fn pointAt, const void *ctx,
                          uint32_t count, uint32_t nowUs)
{
    const l3_confirm_cfg_t *cfg = &confirm->cfg;
    l3_track_point_t run[L3_CONFIRM_MAX_POINTS];
    uint8_t why = L3_CONFIRM_WHY_FEW;
    uint32_t first;

    if (confirm->verdict != L3_CONFIRM_PENDING) {
        return confirm->verdict;
    }
    /* Newest runs first: the latest points are the flight if any are. */
    for (first = (count >= cfg->points) ? count - cfg->points + 1U : 0U; first > 0U; first--) {
        uint32_t i;
        float speed = 0.0F;
        uint8_t verdict;

        for (i = 0U; i < cfg->points; i++) {
            if (pointAt == NULL || pointAt(ctx, first - 1U + i, &run[i]) == 0) {
                break;
            }
        }
        if (i < cfg->points) {
            continue;
        }
        verdict = l3_confirm_run(cfg, run, &speed);
        if (verdict == L3_CONFIRM_WHY_FLIGHT) {
            confirm->speedMps = speed;
            return l3_confirm_decide(confirm, L3_CONFIRM_CONFIRMED, L3_CONFIRM_WHY_FLIGHT,
                                     nowUs);
        }
        if (first == count - cfg->points + 1U) {
            why = verdict; /* the newest run's reason is the one to report */
        }
    }
    if (l3_confirm_dt_s(confirm->candidateUs, nowUs) * 1.0e6F > (float)cfg->windowUs) {
        return l3_confirm_decide(confirm, L3_CONFIRM_REJECTED, L3_CONFIRM_WHY_TIMEOUT, nowUs);
    }
    confirm->why = why;
    return confirm->verdict;
}

uint8_t l3_confirm_end(l3_confirm_t *confirm, uint32_t nowUs)
{
    if (confirm->verdict != L3_CONFIRM_PENDING) {
        return confirm->verdict;
    }
    return l3_confirm_decide(confirm, L3_CONFIRM_REJECTED, L3_CONFIRM_WHY_ENDED, nowUs);
}

const char *l3_confirm_verdict_name(uint8_t verdict)
{
    return (verdict < L3_CONFIRM_VERDICT_COUNT) ? kVerdictNames[verdict] : "?";
}

const char *l3_confirm_why_name(uint8_t why)
{
    return (why < L3_CONFIRM_WHY_COUNT) ? kWhyNames[why] : "?";
}

int32_t l3_confirm_format(const l3_confirm_t *confirm, char *out, uint32_t cap)
{
    char speedText[16];
    uint32_t dt = (confirm->verdict == L3_CONFIRM_CONFIRMED ||
                   confirm->verdict == L3_CONFIRM_REJECTED)
                      ? (uint32_t)(confirm->decidedUs - confirm->candidateUs)
                      : 0U;

    l3_text_fixed2(confirm->speedMps, speedText, sizeof(speedText));
    return snprintf(out, cap,
                    "confirm on=%u verdict=%s why=%s speed=%s dt=%u confirmed_n=%u rejected_n=%u",
                    (unsigned)confirm->cfg.enabled, l3_confirm_verdict_name(confirm->verdict),
                    l3_confirm_why_name(confirm->why), speedText, (unsigned)dt,
                    (unsigned)confirm->confirmed, (unsigned)confirm->rejected);
}
