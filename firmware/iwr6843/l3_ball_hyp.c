/* IWR6843 ball hypotheses. See l3_ball_hyp.h. */
#include <math.h>
#include <string.h>

#include "l3_ball_hyp.h"
#include "l3_club_track.h"

void l3_ball_hyps_cfg_defaults(l3_ball_hyps_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->binWidthM = 6.0F / 128.0F;
    cfg->velocitySpanMps = 2.0F * L3_OBS_WAVELENGTH_M / (4.0F * 135.0e-6F);
    cfg->spawnBehindBins = 1.0F;      /* the ball starts at the origin ... */
    cfg->spawnBeyondBins = 10.0F;     /* ... and is first seen within ~3 frames of it */
    cfg->gateBins = 1.5F;
    cfg->gateMps = 8.0F;              /* drag and fit error, per second of prediction */
    cfg->maxMisses = 2U;
    cfg->classifyPoints = 4U;
    cfg->minDepartureMps = 20.0F;     /* kept equal to the ball tracker's */
    cfg->maxSpeedMps = 100.0F;
    cfg->impactToleranceUs = 15000U;  /* the gate is not the exact impact */
    cfg->maxResidualBins = 1.0F;
    cfg->dopplerToleranceMps = 2.5F;
    cfg->fastBallMps = 26.5F;         /* TrackMan Aug: without it hypotheses regress at 2 ms */
    cfg->fastSupportFraction = 0.55F; /* the Pi detector's FAST_SUPPORT_FRAC */
    cfg->farWindowBins = 0.0F;        /* off until the recorded captures say otherwise */
}

static void l3_ball_hyps_clear(l3_ball_hyps_t *hyps)
{
    uint32_t i;

    memset(hyps->hyp, 0, sizeof(hyps->hyp));
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        hyps->hyp[i].lastTargetIndex = L3_BALL_HYP_NONE;
    }
    hyps->nextId = 0U;
    hyps->spawned = 0U;
    hyps->dropped = 0U;
}

void l3_ball_hyps_init(l3_ball_hyps_t *hyps, const l3_ball_hyps_cfg_t *cfg)
{
    memset(hyps, 0, sizeof(*hyps));
    hyps->cfg = *cfg;
    l3_ball_hyps_clear(hyps);
}

void l3_ball_hyps_arm(l3_ball_hyps_t *hyps, float originBin, uint32_t impactTimestampUs)
{
    l3_ball_hyps_clear(hyps);
    hyps->armed = 1U;
    hyps->originBin = originBin;
    hyps->impactTimestampUs = impactTimestampUs;
}

/* Signed seconds from earlier to later on the wrapping microsecond clock. */
static float l3_ball_hyps_seconds(uint32_t later, uint32_t earlier)
{
    return (float)(int32_t)(later - earlier) * 1.0e-6F;
}

int32_t l3_ball_hyp_fit(const l3_ball_hyp_t *hyp, uint32_t referenceUs, float *rateBinsPerS,
                        float *binAtReference, float *residualBins)
{
    float meanT = 0.0F;
    float meanR = 0.0F;
    float sumTT = 0.0F;
    float sumTR = 0.0F;
    float sse = 0.0F;
    float slope;
    float intercept;
    uint32_t i;

    if (hyp->count < 2U) {
        return 0;
    }
    for (i = 0U; i < hyp->count; i++) {
        meanT += l3_ball_hyps_seconds(hyp->points[i].timestampUs, referenceUs);
        meanR += hyp->points[i].rangeBin;
    }
    meanT /= (float)hyp->count;
    meanR /= (float)hyp->count;
    for (i = 0U; i < hyp->count; i++) {
        float dt = l3_ball_hyps_seconds(hyp->points[i].timestampUs, referenceUs) - meanT;

        sumTT += dt * dt;
        sumTR += dt * (hyp->points[i].rangeBin - meanR);
    }
    if (!(sumTT > 0.0F)) {
        return 0;
    }
    slope = sumTR / sumTT;
    intercept = meanR - slope * meanT;
    for (i = 0U; i < hyp->count; i++) {
        float t = l3_ball_hyps_seconds(hyp->points[i].timestampUs, referenceUs);
        float error = hyp->points[i].rangeBin - (intercept + slope * t);

        sse += error * error;
    }
    *rateBinsPerS = slope;
    *binAtReference = intercept;
    *residualBins = sqrtf(sse / (float)hyp->count);
    return 1;
}

/* Where a hypothesis can be at timestampUs: [lo, hi] around a centre. One
 * point: leaving at anything from minDepartureMps to maxSpeedMps. More: its
 * fitted line, with a gate that widens with the time predicted over. */
static void l3_ball_hyps_window(const l3_ball_hyps_cfg_t *cfg, const l3_ball_hyp_t *hyp,
                                uint32_t timestampUs, float *lo, float *hi, float *centre)
{
    const l3_ball_hyp_point_t *last = &hyp->points[hyp->count - 1U];
    float dtS = l3_ball_hyps_seconds(timestampUs, last->timestampUs);
    float binsPerM = (cfg->binWidthM > 0.0F) ? 1.0F / cfg->binWidthM : 0.0F;
    float rate;
    float atLast;
    float residual;

    if (dtS < 0.0F) {
        dtS = 0.0F;
    }
    if (l3_ball_hyp_fit(hyp, last->timestampUs, &rate, &atLast, &residual)) {
        float spread = cfg->gateBins + cfg->gateMps * binsPerM * dtS;

        *centre = atLast + rate * dtS;
        *lo = *centre - spread;
        *hi = *centre + spread;
        return;
    }
    *lo = last->rangeBin + cfg->minDepartureMps * binsPerM * dtS - cfg->gateBins;
    *hi = last->rangeBin + cfg->maxSpeedMps * binsPerM * dtS + cfg->gateBins;
    *centre = 0.5F * (*lo + *hi);
}

static void l3_ball_hyps_append(l3_ball_hyp_t *hyp, const l3_target_obs_t *target,
                                uint32_t index, uint32_t frame, uint32_t timestampUs,
                                float clubStat)
{
    l3_ball_hyp_point_t *point;

    if (hyp->count == L3_BALL_HYP_POINTS) {
        memmove(&hyp->points[0], &hyp->points[1],
                sizeof(hyp->points[0]) * (L3_BALL_HYP_POINTS - 1U));
        hyp->count--;
    }
    point = &hyp->points[hyp->count++];
    memset(point, 0, sizeof(*point));
    point->frame = frame;
    point->timestampUs = timestampUs;
    point->rangeBin = target->rangeBin;
    point->dopplerAliasMps = target->dopplerAliasMps;
    point->stat = target->stat;
    point->clubStat = clubStat;
    hyp->misses = 0U;
    hyp->lastTargetIndex = index;
}

/* A slot for a new hypothesis: a free one, else a single-point hypothesis not
 * fed this frame (the one that missed most, then the oldest); -1 when every
 * hypothesis is established. */
static int32_t l3_ball_hyps_slot(const l3_ball_hyps_t *hyps)
{
    int32_t best = -1;
    uint32_t i;

    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        if (!hyps->hyp[i].active) {
            return (int32_t)i;
        }
    }
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        const l3_ball_hyp_t *hyp = &hyps->hyp[i];

        if (hyp->count != 1U || hyp->lastTargetIndex != L3_BALL_HYP_NONE) {
            continue;
        }
        if (best < 0 || hyp->misses > hyps->hyp[best].misses ||
            (hyp->misses == hyps->hyp[best].misses && hyp->id < hyps->hyp[best].id)) {
            best = (int32_t)i;
        }
    }
    return best;
}

uint32_t l3_ball_hyps_update(l3_ball_hyps_t *hyps, const l3_target_obs_t *targets, uint32_t n,
                             uint32_t frame, uint32_t timestampUs, uint32_t clubIndex)
{
    const l3_ball_hyps_cfg_t *cfg = &hyps->cfg;
    uint8_t taken[L3_OBS_MAX_TARGETS];
    uint8_t fed[L3_BALL_HYP_MAX];
    float lo[L3_BALL_HYP_MAX];
    float hi[L3_BALL_HYP_MAX];
    float centre[L3_BALL_HYP_MAX];
    float clubStat;
    float sinceGateS;
    float spawnHi;
    uint32_t i;
    uint32_t j;
    uint32_t active = 0U;

    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        hyps->hyp[i].lastTargetIndex = L3_BALL_HYP_NONE;
    }
    if (!hyps->armed) {
        return 0U;
    }
    if (n > L3_OBS_MAX_TARGETS) {
        n = L3_OBS_MAX_TARGETS;
    }
    memset(taken, 0, sizeof(taken));
    memset(fed, 0, sizeof(fed));
    clubStat = (clubIndex < n) ? targets[clubIndex].stat : 0.0F;
    if (clubIndex < n) {
        taken[clubIndex] = 1U;  /* the club's return is never a ball point */
    }
    if (cfg->farWindowBins > 0.0F) {
        for (j = 0U; j < n; j++) {
            if (targets[j].rangeBin < hyps->originBin + cfg->farWindowBins) {
                taken[j] = 1U;  /* short of the far window: never a ball point */
            }
        }
    }
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        if (hyps->hyp[i].active) {
            l3_ball_hyps_window(cfg, &hyps->hyp[i], timestampUs, &lo[i], &hi[i], &centre[i]);
        }
    }
    /* Joint assignment: the cheapest (hypothesis, target) pair first, each
     * hypothesis and each target used once. Ties go to the earlier pair. */
    for (;;) {
        int32_t bestHyp = -1;
        int32_t bestTarget = -1;
        float bestCost = 0.0F;

        for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
            float half;

            if (!hyps->hyp[i].active || fed[i]) {
                continue;
            }
            half = 0.5F * (hi[i] - lo[i]);
            for (j = 0U; j < n; j++) {
                float cost;

                if (taken[j] || targets[j].rangeBin < lo[i] || targets[j].rangeBin > hi[i]) {
                    continue;
                }
                cost = (half > 0.0F) ? fabsf(targets[j].rangeBin - centre[i]) / half : 0.0F;
                if (bestHyp < 0 || cost < bestCost) {
                    bestHyp = (int32_t)i;
                    bestTarget = (int32_t)j;
                    bestCost = cost;
                }
            }
        }
        if (bestHyp < 0) {
            break;
        }
        l3_ball_hyps_append(&hyps->hyp[bestHyp], &targets[bestTarget], (uint32_t)bestTarget,
                            frame, timestampUs, clubStat);
        fed[bestHyp] = 1U;
        taken[bestTarget] = 1U;
    }
    /* Coast the hypotheses that found nothing; drop them after maxMisses. */
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        l3_ball_hyp_t *hyp = &hyps->hyp[i];

        if (!hyp->active || fed[i]) {
            continue;
        }
        hyp->misses++;
        if (hyp->misses > cfg->maxMisses) {
            hyp->active = 0U;
            hyps->dropped++;
        }
    }
    /* Start hypotheses from what is left in the start band. Its far edge
     * moves out with the time since the gate at the fastest ball's speed: a
     * gate that fired late finds the ball already out, and it must still be
     * able to start. */
    sinceGateS = l3_ball_hyps_seconds(timestampUs, hyps->impactTimestampUs);
    spawnHi = hyps->originBin + cfg->spawnBeyondBins +
              ((sinceGateS > 0.0F && cfg->binWidthM > 0.0F)
                   ? cfg->maxSpeedMps / cfg->binWidthM * sinceGateS
                   : 0.0F);
    for (j = 0U; j < n; j++) {
        float range = targets[j].rangeBin;
        l3_ball_hyp_t *hyp;
        int32_t slot;

        if (taken[j] || range < hyps->originBin - cfg->spawnBehindBins || range > spawnHi) {
            continue;
        }
        slot = l3_ball_hyps_slot(hyps);
        if (slot < 0) {
            break;
        }
        hyp = &hyps->hyp[slot];
        if (hyp->active) {
            hyps->dropped++;  /* evicted */
        }
        memset(hyp, 0, sizeof(*hyp));
        hyp->active = 1U;
        hyp->id = hyps->nextId++;
        l3_ball_hyps_append(hyp, &targets[j], j, frame, timestampUs, clubStat);
        hyps->spawned++;
        taken[j] = 1U;
    }
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        active += hyps->hyp[i].active;
    }
    return active;
}

int32_t l3_ball_hyps_set_angles(l3_ball_hyps_t *hyps, uint32_t index, float azimuthRad,
                                float elevationRad, uint8_t anglesValid)
{
    l3_ball_hyp_t *hyp;
    l3_ball_hyp_point_t *point;

    if (index >= L3_BALL_HYP_MAX) {
        return 0;
    }
    hyp = &hyps->hyp[index];
    if (!hyp->active || hyp->count == 0U || hyp->lastTargetIndex == L3_BALL_HYP_NONE) {
        return 0;
    }
    point = &hyp->points[hyp->count - 1U];
    point->azimuthRad = azimuthRad;
    point->elevationRad = elevationRad;
    point->anglesValid = anglesValid;
    return 1;
}

/* Judge one hypothesis as the ball (see l3_ball_hyps_classify). Returns 1
 * and fills out (index included) when it qualifies. */
static int32_t l3_ball_hyps_judge(const l3_ball_hyps_t *hyps, uint32_t i,
                                  l3_ball_hyp_verdict_t *out)
{
    const l3_ball_hyps_cfg_t *cfg = &hyps->cfg;
    const l3_ball_hyp_t *hyp = &hyps->hyp[i];
    float rate;
    float atGate;
    float residual;
    float rateMps;
    float originOffsetS;
    float agree = 0.0F;
    float weaker = 0.0F;
    float withClub = 0.0F;
    float weakerFraction;
    uint32_t k;

    if (!hyp->active || hyp->count < cfg->classifyPoints) {
        return 0;
    }
    if (!l3_ball_hyp_fit(hyp, hyps->impactTimestampUs, &rate, &atGate, &residual) ||
        !(rate > 0.0F)) {
        return 0;
    }
    rateMps = rate * cfg->binWidthM;
    if (rateMps < cfg->minDepartureMps || rateMps > cfg->maxSpeedMps) {
        return 0;
    }
    originOffsetS = (hyps->originBin - atGate) / rate;
    if (fabsf(originOffsetS) * 1.0e6F > (float)cfg->impactToleranceUs) {
        return 0;
    }
    if (residual > cfg->maxResidualBins) {
        return 0;
    }
    for (k = 0U; k < hyp->count; k++) {
        const l3_ball_hyp_point_t *p = &hyp->points[k];

        if (l3_track_wrapped_diff(rateMps, p->dopplerAliasMps, cfg->velocitySpanMps) <=
            cfg->dopplerToleranceMps) {
            agree += 1.0F;
        }
        if (p->clubStat > 0.0F) {
            withClub += 1.0F;
            if (p->stat < p->clubStat) {
                weaker += 1.0F;
            }
        }
    }
    weakerFraction = (withClub > 0.0F) ? weaker / withClub : 0.5F;
    memset(out, 0, sizeof(*out));
    out->index = (int32_t)i;
    out->points = hyp->count;
    out->rateMps = rateMps;
    out->originOffsetUs = originOffsetS * 1.0e6F;
    out->residualBins = residual;
    out->dopplerAgreement = agree / (float)hyp->count;
    out->weakerFraction = weakerFraction;
    out->score = (1.0F - residual / cfg->maxResidualBins) + out->dopplerAgreement +
                 0.5F * weakerFraction;
    return 1;
}

/* An active hypothesis not yet classifiable that is already leaving at
 * fastBallMps or more (2+ points, fitted from its last point). */
static int32_t l3_ball_hyps_fastPending(const l3_ball_hyps_t *hyps)
{
    const l3_ball_hyps_cfg_t *cfg = &hyps->cfg;
    uint32_t i;

    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        const l3_ball_hyp_t *hyp = &hyps->hyp[i];
        float rate;
        float atLast;
        float residual;
        float rateMps;

        if (!hyp->active || hyp->count < 2U || hyp->count >= cfg->classifyPoints) {
            continue;
        }
        if (!l3_ball_hyp_fit(hyp, hyp->points[hyp->count - 1U].timestampUs, &rate, &atLast,
                             &residual)) {
            continue;
        }
        rateMps = rate * cfg->binWidthM;
        if (rateMps >= cfg->fastBallMps && rateMps <= cfg->maxSpeedMps) {
            return 1;
        }
    }
    return 0;
}

void l3_ball_hyps_classify(const l3_ball_hyps_t *hyps, l3_ball_hyp_verdict_t *out)
{
    const l3_ball_hyps_cfg_t *cfg = &hyps->cfg;
    l3_ball_hyp_verdict_t best;
    l3_ball_hyp_verdict_t fast;
    l3_ball_hyp_verdict_t cand;
    uint32_t mostPoints = 0U;
    uint32_t i;

    memset(out, 0, sizeof(*out));
    out->index = -1;
    if (!hyps->armed || !(cfg->maxResidualBins > 0.0F)) {
        return;
    }
    memset(&best, 0, sizeof(best));
    memset(&fast, 0, sizeof(fast));
    best.index = -1;
    fast.index = -1;
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        if (!l3_ball_hyps_judge(hyps, i, &cand)) {
            continue;
        }
        if (cand.points > mostPoints) {
            mostPoints = cand.points;
        }
        if (best.index < 0 || cand.score > best.score) {
            best = cand;
        }
        if (cfg->fastBallMps > 0.0F && cand.rateMps >= cfg->fastBallMps &&
            (fast.index < 0 || cand.points > fast.points ||
             (cand.points == fast.points && cand.score > fast.score))) {
            fast = cand;
        }
    }
    if (cfg->fastBallMps > 0.0F && (best.index < 0 || best.rateMps < cfg->fastBallMps)) {
        /* Fastest credible: a fast hypothesis with enough support of its own
         * beats a slower, better-scoring one. */
        if (fast.index >= 0 &&
            (float)fast.points >= cfg->fastSupportFraction * (float)mostPoints) {
            *out = fast;
            return;
        }
        /* The slow winner is likely the club or the tee while a fast one is
         * still gathering points: wait for it. It classifies or coasts out
         * within classifyPoints + maxMisses frames, so this is bounded. */
        if (best.index >= 0 && l3_ball_hyps_fastPending(hyps)) {
            out->waitingForFast = 1U;
            return;
        }
    }
    if (best.index >= 0) {
        *out = best;
    }
}

uint32_t l3_ball_hyps_struct_bytes(void)
{
    return (uint32_t)sizeof(l3_ball_hyps_t);
}
