#include "compact_iq16.h"

#include <stddef.h>
#include <string.h>

int32_t l3_compact_iq16(const int16_t *source, int16_t *destination,
                        uint16_t chirps, uint16_t receivers,
                        uint16_t source_bins, uint16_t bin_start,
                        uint16_t bin_count)
{
    uint16_t chirp;
    uint16_t receiver;
    size_t destination_word = 0U;
    size_t words_per_window;

    if (source == NULL || destination == NULL || chirps == 0U ||
        receivers == 0U || source_bins == 0U || bin_count == 0U ||
        bin_start >= source_bins || bin_count > source_bins - bin_start) {
        return -1;
    }

    words_per_window = (size_t)bin_count * 2U;
    for (chirp = 0U; chirp < chirps; chirp++) {
        for (receiver = 0U; receiver < receivers; receiver++) {
            size_t source_word =
                (((size_t)chirp * receivers + receiver) * source_bins +
                 bin_start) * 2U;
            memcpy(&destination[destination_word], &source[source_word],
                   words_per_window * sizeof(int16_t));
            destination_word += words_per_window;
        }
    }
    return 0;
}
