#ifndef OPENFLIGHT_LIVE_SELECTOR_H
#define OPENFLIGHT_LIVE_SELECTOR_H

#include <stdint.h>

typedef struct {
    uint16_t windowBins;
    uint16_t maxJumpBins;
    uint16_t maxMisses;
    uint16_t snrQ8;
} L3LiveSelectorParams;

typedef struct {
    int16_t selectedBin;
    int16_t velocityQ8;
    uint16_t misses;
    uint8_t active;
} L3LiveSelectorState;

typedef struct {
    uint16_t candidateBins[2];
    uint32_t candidatePower[2];
    uint32_t noise;
    uint16_t selectedBin;
    uint16_t windowStart;
    uint16_t windowBins;
    uint16_t confidenceQ8;
    uint8_t candidateCount;
    uint8_t accepted;
    uint8_t ambiguous;
} L3LiveSelectorResult;

int32_t l3_live_select(const uint32_t *powers, uint16_t nBins,
                       const L3LiveSelectorParams *params,
                       L3LiveSelectorState *state,
                       L3LiveSelectorResult *result);

#endif
