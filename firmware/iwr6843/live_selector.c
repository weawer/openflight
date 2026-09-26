#include "live_selector.h"

#include <stddef.h>
#include <string.h>

#define L3_RETENTION_MARGIN_BINS 2U
#define L3_MAX_TRACK_HITS 255U

uint16_t l3_retention_window(L3LiveSelectorResult *result, uint16_t nBins)
{
    uint16_t low = result->selectedBin;
    uint16_t high = low;
    uint16_t i;
    if (!result->accepted && !result->coasting) {
        return L3_RETENTION_TRACK_LOST;
    }
    if (result->coasting) {
        /* Keep both the last measurement and the prediction. */
        if (result->heldBin < low) low = result->heldBin;
        if (result->heldBin > high) high = result->heldBin;
    } else if (result->ambiguous) {
        for (i = 0U; i < result->candidateCount; i++) {
            if (result->candidateBins[i] < low) low = result->candidateBins[i];
            if (result->candidateBins[i] > high) high = result->candidateBins[i];
        }
    }
    if (low < L3_RETENTION_MARGIN_BINS ||
        high + L3_RETENTION_MARGIN_BINS >= nBins) {
        return L3_RETENTION_RANGE_EDGE;
    }
    if (high - low + 2U * L3_RETENTION_MARGIN_BINS + 1U > result->windowBins) {
        return result->coasting ? L3_RETENTION_TRACK_LOST
                                : L3_RETENTION_AMBIGUOUS;
    }
    if (result->windowStart > low - L3_RETENTION_MARGIN_BINS) {
        result->windowStart = low - L3_RETENTION_MARGIN_BINS;
    }
    if (result->windowStart + result->windowBins <=
        high + L3_RETENTION_MARGIN_BINS) {
        result->windowStart =
            high + L3_RETENTION_MARGIN_BINS + 1U - result->windowBins;
    }
    return L3_RETENTION_COMPLETE;
}

static uint32_t l3_abs_diff(int32_t left, int32_t right)
{
    return (uint32_t)(left >= right ? left - right : right - left);
}

static void l3_start_track(L3LiveSelectorState *state, int32_t chosen)
{
    state->selectedBin = (int16_t)chosen;
    state->velocityQ8 = 0;
    state->misses = 0U;
    state->active = 1U;
    state->hits = 1U;
}

static void l3_drop_track(L3LiveSelectorState *state)
{
    state->active = 0U;
    state->hits = 0U;
    state->velocityQ8 = 0;
}

/*
 * A new track is tentative until confirmFrames consecutive associations; only
 * confirmed frames are accepted. A confirmed track coasts on its velocity
 * through up to maxMisses missed frames, reporting the predicted bin.
 */
int32_t l3_live_select(const uint32_t *powers, uint16_t nBins,
                       const L3LiveSelectorParams *params,
                       L3LiveSelectorState *state,
                       L3LiveSelectorResult *result)
{
    uint64_t sum = 0U;
    uint16_t bin;
    uint16_t candidates[2] = {0U, 0U};
    uint8_t count = 0U;
    uint8_t confirmed;
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
    if (state->active) {
        int32_t steps = (int32_t)state->misses + 1;
        uint32_t bestDistance = 0xFFFFFFFFU;
        predictedQ8 = (int32_t)state->selectedBin * 256 +
                      (int32_t)state->velocityQ8 * steps;
        for (bin = 0U; bin < count; bin++) {
            int32_t candidateQ8 = (int32_t)candidates[bin] * 256;
            uint32_t distance = l3_abs_diff(candidateQ8, predictedQ8);
            if ((int32_t)candidates[bin] < (int32_t)state->selectedBin - 1 ||
                l3_abs_diff(candidates[bin], state->selectedBin) >
                    (uint32_t)params->maxJumpBins * (uint32_t)steps ||
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
        if (chosen >= 0) {
            int32_t stepQ8 = ((chosen - state->selectedBin) * 256) / steps;
            state->velocityQ8 =
                (int16_t)(((int32_t)state->velocityQ8 + stepQ8) / 2);
            state->selectedBin = (int16_t)chosen;
            state->misses = 0U;
            if (state->hits < L3_MAX_TRACK_HITS) {
                state->hits++;
            }
        }
    }
    confirmed = state->active && state->hits >= params->confirmFrames;
    selected = state->selectedBin;
    if (chosen < 0 && count > 0U && !confirmed) {
        /* No track, or a tentative one that failed to associate: restart on
         * the strongest candidate rather than spending a frame on it. */
        chosen = candidates[0];
        l3_start_track(state, chosen);
    } else if (chosen < 0 && !state->active) {
        state->misses++;
    } else if (chosen < 0 && !confirmed) {
        l3_drop_track(state);
        state->misses = 0U;
    } else if (chosen < 0) {
        state->misses++;
        if (l3_abs_diff(state->velocityQ8, 0) >
            (uint32_t)params->maxJumpBins * 256U) {
            state->velocityQ8 = 0; /* not produced by association: stale */
        }
        if (state->misses > params->maxMisses) {
            l3_drop_track(state);
        } else {
            result->coasting = 1U;
            predictedQ8 = (int32_t)state->selectedBin * 256 +
                          (int32_t)state->velocityQ8 * (int32_t)state->misses;
            if (predictedQ8 < 0) {
                predictedQ8 = 0;
            } else if (predictedQ8 > ((int32_t)nBins - 1) * 256) {
                predictedQ8 = ((int32_t)nBins - 1) * 256;
            }
            selected = (predictedQ8 + 128) / 256;
        }
    }
    if (chosen >= 0) {
        selected = chosen;
    }
    if (chosen >= 0 && state->hits >= params->confirmFrames) {
        result->accepted = 1U;
        result->confidenceQ8 = (uint16_t)(
            ((uint64_t)powers[chosen] * 256U / result->noise) > 65535U
                ? 65535U
                : ((uint64_t)powers[chosen] * 256U / result->noise));
    }
    result->selectedBin = (uint16_t)selected;
    result->heldBin = (uint16_t)state->selectedBin;
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
