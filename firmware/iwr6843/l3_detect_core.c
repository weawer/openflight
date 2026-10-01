/* IWR6843 detect core. See l3_detect_core.h. */
#include "l3_detect_core.h"

#include <stddef.h>
#include <stdio.h>
#include <string.h>

void l3_detect_core_init(l3_detect_core_t *core)
{
    memset(core, 0, sizeof(*core));
    core->requested = (uint8_t)L3_DETECT_CORE_DSS;
    core->active = (uint8_t)L3_DETECT_CORE_DSS;
    core->failLimit = (uint8_t)L3_DETECT_CORE_FAIL_LIMIT_DEFAULT;
}

void l3_detect_core_reset_counts(l3_detect_core_t *core)
{
    uint8_t requested = core->requested;
    uint8_t active = core->active;
    uint8_t latched = core->latched;
    uint8_t failLimit = core->failLimit;

    memset(core, 0, sizeof(*core));
    core->requested = requested;
    core->active = active;
    core->latched = latched;
    core->failLimit = failLimit;
}

int32_t l3_detect_core_set(l3_detect_core_t *core, uint32_t which, uint8_t captureEligible)
{
    if (which != L3_DETECT_CORE_DSS && which != L3_DETECT_CORE_VERIFY) {
        return -1;
    }
    if (which == L3_DETECT_CORE_VERIFY && captureEligible == 0U) {
        return -1;
    }
    core->requested = (uint8_t)which;
    core->active = (uint8_t)which;
    core->latched = 0U;
    core->failStreak = 0U;
    return 0;
}

uint32_t l3_detect_core_route(l3_detect_core_t *core, uint8_t frameEligible)
{
    if (core->active != L3_DETECT_CORE_MSS && frameEligible == 0U) {
        core->ineligible++;
        core->mssFrames++;
        return L3_DETECT_CORE_MSS;
    }
    switch (core->active) {
    case L3_DETECT_CORE_DSS:
        core->dssFrames++;
        return L3_DETECT_CORE_DSS;
    case L3_DETECT_CORE_VERIFY:
        core->verifyFrames++;
        return L3_DETECT_CORE_VERIFY;
    default:
        core->mssFrames++;
        return L3_DETECT_CORE_MSS;
    }
}

static void l3_detect_core_cycles(l3_detect_core_t *core, uint32_t invCycles,
                                  uint32_t scoreCycles)
{
    core->dssInvCyclesLast = invCycles;
    core->dssScoreCyclesLast = scoreCycles;
    if (invCycles > core->dssInvCyclesMax) {
        core->dssInvCyclesMax = invCycles;
    }
    if (scoreCycles > core->dssScoreCyclesMax) {
        core->dssScoreCyclesMax = scoreCycles;
    }
}

int32_t l3_detect_core_report(l3_detect_core_t *core, uint32_t route, uint32_t outcome,
                              uint32_t invCycles, uint32_t scoreCycles)
{
    if (route != L3_DETECT_CORE_DSS && route != L3_DETECT_CORE_VERIFY) {
        return 0;
    }
    if (outcome != L3_DETECT_OUTCOME_FAILED) {
        l3_detect_core_cycles(core, invCycles, scoreCycles);
        if (outcome == L3_DETECT_OUTCOME_MISMATCH) {
            core->mismatches++;
        }
        if (route == L3_DETECT_CORE_DSS) {
            core->failStreak = 0U;
        }
        return 0;
    }
    core->failures++;
    if (route != L3_DETECT_CORE_DSS) {
        return 0; /* verify: the MSS scored it anyway */
    }
    core->fallbacks++;
    core->failStreak++;
    if (core->failStreak < core->failLimit || core->latched) {
        return 0;
    }
    core->active = (uint8_t)L3_DETECT_CORE_MSS;
    core->latched = 1U;
    core->latches++;
    return 1;
}

void l3_detect_core_note_mismatch(l3_detect_core_t *core, uint32_t slot, uint32_t bin,
                                  uint32_t field)
{
    if (core->haveMismatch) {
        return;
    }
    core->haveMismatch = 1U;
    core->mismatchSlot = slot;
    core->mismatchBin = bin;
    core->mismatchField = field;
}

static const char *const kCoreNames[L3_DETECT_CORE_COUNT] = { "mss", "dss", "verify" };

const char *l3_detect_core_name(uint32_t which)
{
    return (which < L3_DETECT_CORE_COUNT) ? kCoreNames[which] : NULL;
}

int32_t l3_detect_core_parse(const char *name, uint32_t *which)
{
    uint32_t index;

    if (name == NULL || which == NULL) {
        return -1;
    }
    /* From DSS on: mss names the active core once latched, not a choice. */
    for (index = L3_DETECT_CORE_DSS; index < L3_DETECT_CORE_COUNT; index++) {
        if (strcmp(name, kCoreNames[index]) == 0) {
            *which = index;
            return 0;
        }
    }
    return -1;
}

static uint32_t l3_detect_core_us(uint32_t cycles, uint32_t clockMhz)
{
    return (clockMhz == 0U) ? 0U : cycles / clockMhz;
}

int32_t l3_detect_core_format(const l3_detect_core_t *core, uint32_t dssClockMhz, char *out,
                              uint32_t cap)
{
    int32_t written;

    written = (int32_t)snprintf(
        out, cap,
        "detect core=%s active=%s latched=%u mss=%u dss=%u verify=%u ineligible=%u "
        "failures=%u fallbacks=%u streak=%u latches=%u mismatches=%u "
        "dss_inv_us=%u/%u dss_score_us=%u/%u",
        l3_detect_core_name(core->requested), l3_detect_core_name(core->active),
        (unsigned)core->latched, (unsigned)core->mssFrames, (unsigned)core->dssFrames,
        (unsigned)core->verifyFrames, (unsigned)core->ineligible, (unsigned)core->failures,
        (unsigned)core->fallbacks, (unsigned)core->failStreak, (unsigned)core->latches,
        (unsigned)core->mismatches,
        (unsigned)l3_detect_core_us(core->dssInvCyclesLast, dssClockMhz),
        (unsigned)l3_detect_core_us(core->dssInvCyclesMax, dssClockMhz),
        (unsigned)l3_detect_core_us(core->dssScoreCyclesLast, dssClockMhz),
        (unsigned)l3_detect_core_us(core->dssScoreCyclesMax, dssClockMhz));
    if (core->haveMismatch && written >= 0 && (uint32_t)written < cap) {
        written += (int32_t)snprintf(&out[written], cap - (uint32_t)written,
                                     " first_mismatch=%u:%u:%u", (unsigned)core->mismatchSlot,
                                     (unsigned)core->mismatchBin,
                                     (unsigned)core->mismatchField);
    }
    return written;
}
