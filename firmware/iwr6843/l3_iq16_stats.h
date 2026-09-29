/* IWR6843 exact IQ16 channel statistics.
 *
 * The observation layer's per-bin numbers (burst-MTI residual energy,
 * per-loop residual power, lag-1 loop autocorrelation) come from int16 I/Q
 * samples. Summing their squares in float loses low bits at every add; this
 * module keeps the arithmetic in integers until one conversion at the end.
 * The loop mean is not an integer, so the residual is taken SCALED by the
 * loop count: r' = loops * x - sum(x) is exact in int32, its square in int64
 * (|r'| <= 2^21 for 16 loops), and the sums over loops and channels stay
 * far below 2^63. Dividing the totals by loops^2 once gives the same
 * quantities l3_verticalResidual computed in float, bit-exact up to that
 * final conversion. Pure C, no hardware.
 *
 * Range window. The range FFT is 128 points over 128 samples, so a periodic
 * Hann window in time is exactly the 3-tap kernel X[k] - (X[k-1] + X[k+1]) / 2
 * on its output (gain-compensated: an on-bin tone keeps its amplitude). The
 * sidelobes of a strong return (club, body, clutter) drop from about -13 dB to
 * -31 dB, so a weak return a few bins away (the ball) is no longer buried; the
 * main lobe doubles in width. It is taken as 2X[k] - X[k-1] - X[k+1] to stay
 * in integers (|x| < 2^17, |r'| < 2^22 for 16 loops) and finish removes the 2.
 */
#ifndef L3_IQ16_STATS_H
#define L3_IQ16_STATS_H

#include <stdint.h>

#define L3_IQ16_MAX_LOOPS 16U

enum {
    L3_RANGE_WINDOW_NONE = 0,
    L3_RANGE_WINDOW_HANN = 1
};

/* One (tx, rx) channel of one bin over the loops, scaled by loops (sums) and
 * loops^2 (products), and by 2 (sums) and 4 (products) under the Hann window. */
typedef struct {
    uint32_t loops;
    int32_t  sumIm;                 /* over the loops */
    int32_t  sumRe;
    uint32_t window;                /* L3_RANGE_WINDOW_* */
    int64_t  energy;                /* sum over loops of |r'|^2 */
    int64_t  loopPower[L3_IQ16_MAX_LOOPS];
    int64_t  r1Re;                  /* sum over loops >= 1 of r'[l] conj(r'[l-1]) */
    int64_t  r1Im;
} l3_iq16_channel_stats_t;

/* Accumulate channels into one bin's statistics, still scaled. */
typedef struct {
    uint32_t loops;
    uint32_t channels;
    uint32_t window;                /* taken from the first channel added */
    int64_t  energy;
    int64_t  loopPower[L3_IQ16_MAX_LOOPS];
    int64_t  r1Re;
    int64_t  r1Im;
} l3_iq16_bin_stats_t;

/* samples: the channel's first (Im, Re) pair; strideWords: int16 words from
 * one loop's pair to the next. Returns 0, or -1 for loops of 0 or > 16. */
int32_t l3_iq16_channel_stats(const int16_t *samples, uint32_t loops, uint32_t strideWords,
                              l3_iq16_channel_stats_t *out);
/* The same over the windowed bin. With L3_RANGE_WINDOW_HANN the neighbouring
 * bins must be readable at samples - 2 and samples + 2 words (the adjacent
 * complex samples of the same chirp and channel); a bin on the edge of its
 * window has only one and must be passed L3_RANGE_WINDOW_NONE. Returns -1 for
 * an unknown window as for a bad loop count. */
int32_t l3_iq16_channel_stats_windowed(const int16_t *samples, uint32_t loops,
                                       uint32_t strideWords, uint32_t window,
                                       l3_iq16_channel_stats_t *out);
void l3_iq16_bin_stats_init(l3_iq16_bin_stats_t *bin, uint32_t loops);
/* A channel with another loop count, or another window than the channels
 * already added, is refused rather than mixed in. */
void l3_iq16_bin_stats_add(l3_iq16_bin_stats_t *bin, const l3_iq16_channel_stats_t *channel);
/* The observation-layer quantities in physical units (divided by loops^2),
 * as floats: energy, the strongest loop, loop 0, r1. perLoop may be NULL. */
void l3_iq16_bin_stats_finish(const l3_iq16_bin_stats_t *bin, float *energy, float *peak,
                              float *loop0, float *r1Re, float *r1Im, float *perLoop);

#endif /* L3_IQ16_STATS_H */
