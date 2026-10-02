#include "l3_ball_recover.h"

#include <math.h>
#include <string.h>

#include "l3_club_track.h"

void l3_ball_recover_cfg_defaults(l3_ball_recover_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->binWidthM = 6.0F / 128.0F;
    cfg->velocitySpanMps = 2.0F * L3_OBS_WAVELENGTH_M / (4.0F * 135.0e-6F);
    cfg->gateM = 0.6F * cfg->binWidthM;
    cfg->tieBins = 0.1F;
    cfg->maxResidualBins = 1.0F;
    cfg->dopplerToleranceMps = 2.5F;
}

void l3_ball_history_reset(l3_ball_history_t *h)
{
    memset(h, 0, sizeof(*h));
}

void l3_ball_history_push(l3_ball_history_t *h, const l3_target_obs_t *targets, uint32_t n,
                          uint32_t frame, uint32_t timestampUs, uint32_t clubIndex)
{
    l3_ball_history_frame_t *f = &h->frames[h->next];
    uint32_t k;

    memset(f, 0, sizeof(*f));
    f->frame = frame;
    f->timestampUs = timestampUs;
    for (k = 0U; k < n && k < L3_BALL_HISTORY_TARGETS; k++) {
        f->targets[k].rangeBin = targets[k].rangeBin;
        f->targets[k].dopplerAliasMps = targets[k].dopplerAliasMps;
        f->targets[k].stat = targets[k].stat;
        f->targets[k].coherence = targets[k].coherence;
        if (k == clubIndex) {
            f->clubMask |= (uint8_t)(1U << k);
        }
    }
    f->count = (uint8_t)k;
    h->next = (h->next + 1U) % L3_BALL_HISTORY_FRAMES;
    if (h->count < L3_BALL_HISTORY_FRAMES) {
        h->count++;
    }
}

void l3_ball_history_mark_club(l3_ball_history_t *h, uint32_t frame, uint32_t index)
{
    l3_ball_history_frame_t *f;

    if (h->count == 0U) {
        return;
    }
    f = &h->frames[(h->next + L3_BALL_HISTORY_FRAMES - 1U) % L3_BALL_HISTORY_FRAMES];
    if (f->frame != frame || index >= f->count) {
        return;
    }
    f->clubMask |= (uint8_t)(1U << index);
}

const l3_ball_history_frame_t *l3_ball_history_at(const l3_ball_history_t *h, uint32_t index)
{
    if (index >= h->count) {
        return NULL;
    }
    return &h->frames[(h->next + L3_BALL_HISTORY_FRAMES - h->count + index) %
                      L3_BALL_HISTORY_FRAMES];
}

static int32_t l3_ball_recover_hasFrame(const l3_ball_hyp_t *hyp, uint32_t frame)
{
    uint32_t k;

    for (k = 0U; k < hyp->count; k++) {
        if (hyp->points[k].frame == frame) {
            return 1;
        }
    }
    return 0;
}

/* The candidate in f for a ball at predictedBin moving at rateMps; -1 when none. */
static int32_t l3_ball_recover_pick(const l3_ball_recover_cfg_t *cfg,
                                    const l3_ball_history_frame_t *f, float predictedBin,
                                    float acceptFromBin, float rateMps)
{
    float gateBins = cfg->gateM / cfg->binWidthM;
    int32_t best = -1;
    float bestErr = 0.0F;
    uint32_t k;

    for (k = 0U; k < f->count; k++) {
        const l3_ball_history_target_t *t = &f->targets[k];
        float err = fabsf(t->rangeBin - predictedBin);

        if ((f->clubMask & (1U << k)) != 0U || t->rangeBin < acceptFromBin || err > gateBins) {
            continue;
        }
        if (best >= 0 && fabsf(err - bestErr) <= cfg->tieBins) {
            int32_t agrees = l3_track_wrapped_diff(rateMps, t->dopplerAliasMps,
                                                   cfg->velocitySpanMps) <=
                             cfg->dopplerToleranceMps;
            int32_t bestAgrees =
                l3_track_wrapped_diff(rateMps, f->targets[best].dopplerAliasMps,
                                      cfg->velocitySpanMps) <= cfg->dopplerToleranceMps;

            if (agrees && !bestAgrees) {
                best = (int32_t)k;
                bestErr = err;
            } else if (agrees == bestAgrees && err < bestErr) {
                best = (int32_t)k;
                bestErr = err;
            }
            continue;
        }
        if (best < 0 || err < bestErr) {
            best = (int32_t)k;
            bestErr = err;
        }
    }
    return best;
}

/* Whether the capped merge kept the point of this frame. */
static int32_t l3_ball_recover_kept(const l3_ball_hyp_point_t *out, uint32_t n, uint32_t frame)
{
    uint32_t k;

    for (k = 0U; k < n; k++) {
        if (out[k].frame == frame) {
            return 1;
        }
    }
    return 0;
}

/* Merge two time-ordered runs into out (at most cap, newest kept). */
static uint32_t l3_ball_recover_merge(const l3_ball_hyp_point_t *a, uint32_t na,
                                      const l3_ball_hyp_point_t *b, uint32_t nb,
                                      l3_ball_hyp_point_t *out, uint32_t cap)
{
    uint32_t total = na + nb;
    uint32_t skip = (total > cap) ? total - cap : 0U;
    uint32_t i = 0U;
    uint32_t j = 0U;
    uint32_t n = 0U;

    while (i < na || j < nb) {
        const l3_ball_hyp_point_t *p;

        if (j >= nb || (i < na && (int32_t)(a[i].timestampUs - b[j].timestampUs) <= 0)) {
            p = &a[i++];
        } else {
            p = &b[j++];
        }
        if (skip > 0U) {
            skip--;
            continue;
        }
        out[n++] = *p;
    }
    return n;
}

uint32_t l3_ball_recover(const l3_ball_recover_cfg_t *cfg, const l3_ball_history_t *h,
                         const l3_ball_hyp_t *hyp, float acceptFromBin,
                         l3_ball_hyp_point_t *out, uint32_t cap, l3_ball_recover_result_t *result)
{
    l3_ball_hyp_point_t found[L3_BALL_HISTORY_FRAMES];
    uint32_t newestUs;
    uint32_t refUs;
    uint32_t nFound = 0U;
    uint32_t n;
    float rate;
    float atRef;
    float residual;
    uint32_t i;

    memset(result, 0, sizeof(*result));
    if (hyp->count == 0U || cap == 0U) {
        return 0U;
    }
    refUs = hyp->points[0].timestampUs;
    newestUs = hyp->points[hyp->count - 1U].timestampUs;
    if (l3_ball_points_fit(hyp->points, hyp->count, refUs, &rate, &atRef, &residual)) {
        float rateMps = rate * cfg->binWidthM;

        for (i = 0U; i < h->count; i++) {
            const l3_ball_history_frame_t *f = l3_ball_history_at(h, i);
            float dtS;
            int32_t k;

            if ((int32_t)(f->timestampUs - newestUs) >= 0 ||
                l3_ball_recover_hasFrame(hyp, f->frame)) {
                continue;
            }
            dtS = (float)(int32_t)(f->timestampUs - refUs) * 1.0e-6F;
            k = l3_ball_recover_pick(cfg, f, atRef + rate * dtS, acceptFromBin, rateMps);
            if (k < 0) {
                continue;
            }
            memset(&found[nFound], 0, sizeof(found[nFound]));
            found[nFound].frame = f->frame;
            found[nFound].timestampUs = f->timestampUs;
            found[nFound].rangeBin = f->targets[k].rangeBin;
            found[nFound].dopplerAliasMps = f->targets[k].dopplerAliasMps;
            found[nFound].stat = f->targets[k].stat;
            found[nFound].coherence = f->targets[k].coherence;
            nFound++;
        }
    }
    if (nFound > 0U) {
        n = l3_ball_recover_merge(hyp->points, hyp->count, found, nFound, out, cap);
        if (l3_ball_points_fit(out, n, out[0].timestampUs, &rate, &atRef, &residual) &&
            residual <= cfg->maxResidualBins) {
            result->count = n;
            result->residualBins = residual;
            for (i = 0U; i < nFound; i++) {
                if (l3_ball_recover_kept(out, n, found[i].frame)) {
                    if (result->recovered == 0U) {
                        result->firstFrame = found[i].frame;
                    }
                    result->recovered++;
                }
            }
            for (i = 0U; i < nFound; i++) {
                uint32_t bit = found[i].frame - result->firstFrame;

                if (l3_ball_recover_kept(out, n, found[i].frame) && bit < 32U) {
                    result->mask |= 1U << bit;
                }
            }
            return n;
        }
    }
    n = l3_ball_recover_merge(hyp->points, hyp->count, NULL, 0U, out, cap);
    result->count = n;
    return n;
}

uint32_t l3_ball_history_struct_bytes(void)
{
    return (uint32_t)sizeof(l3_ball_history_t);
}
