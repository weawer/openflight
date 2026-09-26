#ifndef OPENFLIGHT_COMPACT_IQ16_H
#define OPENFLIGHT_COMPACT_IQ16_H

#include <stdint.h>

int32_t l3_compact_iq16(const int16_t *source, int16_t *destination,
                        uint16_t chirps, uint16_t receivers,
                        uint16_t source_bins, uint16_t bin_start,
                        uint16_t bin_count);

#endif
