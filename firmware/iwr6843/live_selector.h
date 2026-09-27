#ifndef OPENFLIGHT_LIVE_SELECTOR_H
#define OPENFLIGHT_LIVE_SELECTOR_H

#include <stdint.h>

enum {
    L3_RETENTION_COMPLETE = 0,
    L3_RETENTION_TRACK_LOST = 1,
    L3_RETENTION_AMBIGUOUS = 2,
    L3_RETENTION_RANGE_EDGE = 3,
    L3_RETENTION_SHORT_HISTORY = 4
};

typedef struct {
    uint16_t windowBins;
    uint16_t maxJumpBins;
    uint16_t maxMisses;
    uint16_t snrQ8;
    /* Consecutive associated frames before a track is accepted. */
    uint16_t confirmFrames;
} L3LiveSelectorParams;

typedef struct {
    int16_t selectedBin;
    int16_t velocityQ8;
    uint16_t misses;
    uint8_t active;
    uint8_t hits;
} L3LiveSelectorState;

typedef struct {
    uint16_t candidateBins[2];
    uint32_t candidatePower[2];
    uint32_t noise;
    uint16_t selectedBin;
    uint16_t windowStart;
    uint16_t windowBins;
    uint16_t confidenceQ8;
    uint16_t heldBin;
    uint8_t candidateCount;
    uint8_t accepted;
    uint8_t ambiguous;
    uint8_t coasting;
} L3LiveSelectorResult;

int32_t l3_live_select(const uint32_t *powers, uint16_t nBins,
                       const L3LiveSelectorParams *params,
                       L3LiveSelectorState *state,
                       L3LiveSelectorResult *result);

uint16_t l3_retention_window(L3LiveSelectorResult *result, uint16_t nBins);

/*
 * One "RST ..." CLI line describing the selector result that stopped adaptive
 * retention. frame is the dump-order index of the rejected, unretained frame.
 */
int32_t l3_format_retention_stop(uint16_t reason, uint16_t frame,
                                 const L3LiveSelectorResult *result,
                                 char *out, uint32_t cap);

/*
 * Suppress rise[] bins whose coherent[] difference is not well above the
 * in-window mean. See coherent_gate() in live_selector.py and
 * plans/iwr-coherent-gate.md. gateQ8 = 512 requires 2x the mean.
 */
void l3_coherent_gate(const uint32_t *rise, const uint32_t *coherent,
                     uint16_t nBins, uint16_t gateQ8, uint32_t *gated);

#endif
