/* See l3_impact.h. */
#include <stdio.h>
#include <string.h>

#include "l3_impact.h"
#include "l3_text.h"

static const char *const kWhyNames[L3_IMPACT_WHY_COUNT] = {
    "none", "nodelivery", "pending", "passed", "fired"
};
static const char *const kCauseNames[L3_IMPACT_CAUSE_COUNT] = {
    "none", "crossing", "end"
};

void l3_impact_cfg_defaults(l3_impact_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->horizonS = 0.004F;      /* one 3 ms frame plus scheduling slack */
    cfg->endM = 0.40F;           /* ~8.5 bins: the labelled club-to-ball gaps, median 7.4 */
    cfg->endMinMps = 20.0F;      /* over a backswing's ~17 m/s, under the labelled 20.6-62 */
}

void l3_impact_init(l3_impact_t *impact, const l3_impact_cfg_t *cfg)
{
    memset(impact, 0, sizeof(*impact));
    impact->cfg = *cfg;
}

void l3_impact_rearm(l3_impact_t *impact)
{
    impact->fired = 0U;
    impact->why = L3_IMPACT_WHY_NONE;
    impact->cause = L3_IMPACT_CAUSE_NONE;
    impact->endArmed = 0U;
    impact->endTimeUs = 0U;
    impact->offsetS = 0.0F;
    impact->impactTimestampUs = 0U;
}

static int32_t l3_impact_note(l3_impact_t *impact, uint8_t why)
{
    impact->why = why;
    impact->counters[why]++;
    return (why == L3_IMPACT_WHY_FIRED) ? 1 : 0;
}

/* The newest point ends an approach that may fire the end: a downswing's
 * speed, short of the ball by at most endM. */
static uint8_t l3_impact_end_arms(const l3_impact_cfg_t *cfg, const l3_fit_estimate_t *clubIn,
                                  const l3_impact_club_t *club)
{
    float gapM = club->ballRangeM - club->rangeM;

    return (cfg->endM > 0.0F && clubIn->speedMps >= cfg->endMinMps && gapM >= 0.0F &&
            gapM <= cfg->endM) ? 1U : 0U;
}

static int32_t l3_impact_fire(l3_impact_t *impact, uint8_t cause, uint32_t stamp, float offset)
{
    impact->fired = 1U;
    impact->cause = cause;
    impact->impactTimestampUs = stamp;
    impact->offsetS = offset;
    return l3_impact_note(impact, L3_IMPACT_WHY_FIRED);
}

int32_t l3_impact_update_range(l3_impact_t *impact, const l3_fit_estimate_t *clubIn,
                               const l3_impact_club_t *club, uint32_t nowUs)
{
    uint32_t stamp;
    float offset;
    uint8_t usable;

    if (impact->fired) {
        return 0;
    }
    usable = (clubIn != NULL && clubIn->why == L3_FIT_WHY_OK) ? 1U : 0U;
    if (club != NULL) {
        if (!club->appended) {
            if (impact->endArmed) {
                /* The approach ended near the ball: impact, at its last point. */
                return l3_impact_fire(impact, L3_IMPACT_CAUSE_END, impact->endTimeUs,
                                      (float)(int32_t)(impact->endTimeUs - nowUs) * 1.0e-6F);
            }
        } else {
            impact->endArmed = (usable && l3_impact_end_arms(&impact->cfg, clubIn, club)) ? 1U : 0U;
            impact->endTimeUs = club->timeUs;
        }
    }
    if (!usable) {
        return l3_impact_note(impact, L3_IMPACT_WHY_NO_DELIVERY);
    }
    /* Rounded to a timestamp first so the difference is wrap-safe. */
    stamp = l3_round_us(clubIn->timeUs);
    offset = (float)(int32_t)(stamp - nowUs) * 1.0e-6F;
    impact->offsetS = offset;
    if (offset > impact->cfg.horizonS) {
        return l3_impact_note(impact, L3_IMPACT_WHY_PENDING);
    }
    if (offset < -impact->cfg.horizonS) {
        return l3_impact_note(impact, L3_IMPACT_WHY_PASSED);
    }
    return l3_impact_fire(impact, L3_IMPACT_CAUSE_CROSSING, stamp, offset);
}

const char *l3_impact_why_name(uint8_t why)
{
    return (why < L3_IMPACT_WHY_COUNT) ? kWhyNames[why] : "?";
}

const char *l3_impact_cause_name(uint8_t cause)
{
    return (cause < L3_IMPACT_CAUSE_COUNT) ? kCauseNames[cause] : "?";
}

int32_t l3_impact_format(const l3_impact_t *impact, char *out, uint32_t cap)
{
    char offsetText[16];

    l3_text_fixed2(impact->offsetS * 1000.0F, offsetText, sizeof(offsetText));
    return snprintf(out, cap,
                    "impact fired=%u why=%s cause=%s offsetms=%s t=%u pending=%u passed=%u "
                    "fired_n=%u armed=%u",
                    (unsigned)impact->fired, l3_impact_why_name(impact->why),
                    l3_impact_cause_name(impact->cause), offsetText,
                    (unsigned)impact->impactTimestampUs,
                    (unsigned)impact->counters[L3_IMPACT_WHY_PENDING],
                    (unsigned)impact->counters[L3_IMPACT_WHY_PASSED],
                    (unsigned)impact->counters[L3_IMPACT_WHY_FIRED],
                    (unsigned)impact->endArmed);
}
