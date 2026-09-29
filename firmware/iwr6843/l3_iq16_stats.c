/* See l3_iq16_stats.h. */
#include <string.h>

#include "l3_iq16_stats.h"

/* One loop's (Im, Re) at the bin, windowed; see the header for the kernel. */
static void l3_iq16_sample(const int16_t *sample, uint32_t window, int32_t *im, int32_t *re)
{
    if (window == L3_RANGE_WINDOW_HANN) {
        *im = 2 * (int32_t)sample[0] - (int32_t)sample[-2] - (int32_t)sample[2];
        *re = 2 * (int32_t)sample[1] - (int32_t)sample[-1] - (int32_t)sample[3];
    } else {
        *im = (int32_t)sample[0];
        *re = (int32_t)sample[1];
    }
}

int32_t l3_iq16_channel_stats(const int16_t *samples, uint32_t loops, uint32_t strideWords,
                              l3_iq16_channel_stats_t *out)
{
    return l3_iq16_channel_stats_windowed(samples, loops, strideWords, L3_RANGE_WINDOW_NONE, out);
}

int32_t l3_iq16_channel_stats_windowed(const int16_t *samples, uint32_t loops,
                                       uint32_t strideWords, uint32_t window,
                                       l3_iq16_channel_stats_t *out)
{
    int32_t sumIm = 0;
    int32_t sumRe = 0;
    int32_t prevIm = 0;
    int32_t prevRe = 0;
    /* Each loop's windowed sample is read here once and reused below. The
     * two-pass mean/residual algorithm would otherwise call l3_iq16_sample
     * twice per loop, so L3_RANGE_WINDOW_HANN's 3-neighbour read and combine
     * ran twice for every sample -- the cost that missed the 3 ms detect
     * deadline on the rig (2026-09-29 armed soak, scratch_stale=13/106). */
    int32_t valueIm[L3_IQ16_MAX_LOOPS];
    int32_t valueRe[L3_IQ16_MAX_LOOPS];
    const int16_t *sample;
    uint32_t loop;

    memset(out, 0, sizeof(*out));
    if (loops == 0U || loops > L3_IQ16_MAX_LOOPS ||
        (window != L3_RANGE_WINDOW_NONE && window != L3_RANGE_WINDOW_HANN)) {
        return -1;
    }
    out->loops = loops;
    out->window = window;
    sample = samples;
    for (loop = 0U; loop < loops; loop++) {
        l3_iq16_sample(sample, window, &valueIm[loop], &valueRe[loop]);
        sumIm += valueIm[loop];
        sumRe += valueRe[loop];
        sample += strideWords;
    }
    out->sumIm = sumIm;
    out->sumRe = sumRe;
    for (loop = 0U; loop < loops; loop++) {
        /* Residual scaled by loops: exact in int32 (|x| < 2^17 windowed,
         * loops <= 16). */
        int32_t im = (int32_t)loops * valueIm[loop] - sumIm;
        int32_t re = (int32_t)loops * valueRe[loop] - sumRe;
        int64_t power = (int64_t)im * im + (int64_t)re * re;

        out->loopPower[loop] = power;
        out->energy += power;
        if (loop > 0U) {
            /* r'[loop] * conj(r'[loop - 1]) */
            out->r1Re += (int64_t)re * prevRe + (int64_t)im * prevIm;
            out->r1Im += (int64_t)im * prevRe - (int64_t)re * prevIm;
        }
        prevIm = im;
        prevRe = re;
    }
    return 0;
}

void l3_iq16_bin_stats_init(l3_iq16_bin_stats_t *bin, uint32_t loops)
{
    memset(bin, 0, sizeof(*bin));
    bin->loops = loops;
}

void l3_iq16_bin_stats_add(l3_iq16_bin_stats_t *bin, const l3_iq16_channel_stats_t *channel)
{
    uint32_t loop;

    if (channel->loops != bin->loops) {
        return;
    }
    if (bin->channels == 0U) {
        bin->window = channel->window;
    } else if (channel->window != bin->window) {
        return;
    }
    bin->channels++;
    bin->energy += channel->energy;
    bin->r1Re += channel->r1Re;
    bin->r1Im += channel->r1Im;
    for (loop = 0U; loop < bin->loops; loop++) {
        bin->loopPower[loop] += channel->loopPower[loop];
    }
}

void l3_iq16_bin_stats_finish(const l3_iq16_bin_stats_t *bin, float *energy, float *peak,
                              float *loop0, float *r1Re, float *r1Im, float *perLoop)
{
    /* One division by loops^2 (and 4 for the windowed 2X) undoes the scaling;
     * double keeps the totals (under 2^52) exact before the float conversion. */
    double gain = (bin->window == L3_RANGE_WINDOW_HANN) ? 4.0 : 1.0;
    double scale = (bin->loops > 0U)
                       ? 1.0 / ((double)bin->loops * (double)bin->loops * gain)
                       : 0.0;
    double best = 0.0;
    uint32_t loop;

    for (loop = 0U; loop < bin->loops; loop++) {
        double value = (double)bin->loopPower[loop] * scale;

        if (perLoop != NULL) {
            perLoop[loop] = (float)value;
        }
        if (value > best) {
            best = value;
        }
    }
    if (energy != NULL) {
        *energy = (float)((double)bin->energy * scale);
    }
    if (peak != NULL) {
        *peak = (float)best;
    }
    if (loop0 != NULL) {
        *loop0 = (bin->loops > 0U) ? (float)((double)bin->loopPower[0] * scale) : 0.0F;
    }
    if (r1Re != NULL) {
        *r1Re = (float)((double)bin->r1Re * scale);
    }
    if (r1Im != NULL) {
        *r1Im = (float)((double)bin->r1Im * scale);
    }
}
