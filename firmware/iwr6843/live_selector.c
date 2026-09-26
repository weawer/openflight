#include "live_selector.h"

#include <stddef.h>
#include <string.h>

static uint32_t l3_abs_diff(int32_t left, int32_t right)
{
    return (uint32_t)(left >= right ? left - right : right - left);
}

int32_t l3_live_select(const uint32_t *powers, uint16_t nBins,
                       const L3LiveSelectorParams *params,
                       L3LiveSelectorState *state,
                       L3LiveSelectorResult *result)
{
    uint64_t sum = 0U;
    uint16_t bin;
    uint16_t candidates[2] = {0U, 0U};
    uint8_t count = 0U;
    int32_t chosen = -1;
    int32_t selected;
    int32_t predictedQ8;

    if (powers == NULL || params == NULL || state == NULL || result == NULL ||
        nBins < 3U || params->windowBins == 0U || params->windowBins > nBins) {
        return -1;
    }
    memset(result, 0, sizeof(*result));
    for (bin = 0U; bin < nBins; bin++) {
        sum += powers[bin];
    }
    result->noise = (uint32_t)(sum / nBins);
    if (result->noise == 0U) {
        result->noise = 1U;
    }
    for (bin = 1U; bin + 1U < nBins; bin++) {
        uint8_t position;
        if (powers[bin] < powers[bin - 1U] || powers[bin] < powers[bin + 1U] ||
            (uint64_t)powers[bin] * 256U <
                (uint64_t)result->noise * params->snrQ8) {
            continue;
        }
        position = 0U;
        while (position < count &&
               (powers[candidates[position]] > powers[bin] ||
                (powers[candidates[position]] == powers[bin] &&
                 candidates[position] < bin))) {
            position++;
        }
        if (position < 2U) {
            if (count < 2U) {
                count++;
            }
            if (count == 2U && position == 0U) {
                candidates[1] = candidates[0];
            }
            candidates[position] = bin;
        }
    }
    result->candidateCount = count;
    for (bin = 0U; bin < count; bin++) {
        result->candidateBins[bin] = candidates[bin];
        result->candidatePower[bin] = powers[candidates[bin]];
    }
    if (count > 0U && !state->active) {
        chosen = candidates[0];
    } else if (count > 0U) {
        uint32_t bestDistance = 0xFFFFFFFFU;
        predictedQ8 = (int32_t)state->selectedBin * 256 + state->velocityQ8;
        for (bin = 0U; bin < count; bin++) {
            int32_t candidateQ8 = (int32_t)candidates[bin] * 256;
            uint32_t distance = l3_abs_diff(candidateQ8, predictedQ8);
            if ((int32_t)candidates[bin] < (int32_t)state->selectedBin - 1 ||
                l3_abs_diff(candidates[bin], state->selectedBin) >
                    params->maxJumpBins ||
                distance > (uint32_t)params->maxJumpBins * 256U) {
                continue;
            }
            if (chosen < 0 || distance < bestDistance ||
                (distance == bestDistance &&
                 (powers[candidates[bin]] > powers[chosen] ||
                  (powers[candidates[bin]] == powers[chosen] &&
                   candidates[bin] < (uint16_t)chosen)))) {
                chosen = candidates[bin];
                bestDistance = distance;
            }
        }
    }
    if (chosen < 0) {
        state->misses++;
        selected = state->selectedBin;
        state->velocityQ8 = 0;
        if (state->misses > params->maxMisses) {
            state->active = 0U;
            state->velocityQ8 = 0;
        }
    } else {
        if (state->active) {
            state->velocityQ8 = (int16_t)((state->velocityQ8 +
                (chosen - state->selectedBin) * 256) / 2);
        } else {
            state->velocityQ8 = 0;
        }
        state->selectedBin = (int16_t)chosen;
        state->misses = 0U;
        state->active = 1U;
        selected = chosen;
        result->accepted = 1U;
        result->confidenceQ8 = (uint16_t)(
            ((uint64_t)powers[chosen] * 256U / result->noise) > 65535U
                ? 65535U
                : ((uint64_t)powers[chosen] * 256U / result->noise));
    }
    result->selectedBin = (uint16_t)selected;
    result->windowBins = params->windowBins;
    selected -= params->windowBins / 2U;
    if (selected < 0) {
        selected = 0;
    } else if (selected > (int32_t)(nBins - params->windowBins)) {
        selected = nBins - params->windowBins;
    }
    result->windowStart = (uint16_t)selected;
    result->ambiguous = count == 2U &&
        (uint64_t)powers[candidates[1]] * 256U >=
        (uint64_t)powers[candidates[0]] * 230U;
    return 0;
}
