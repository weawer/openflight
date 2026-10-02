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
    cfg->spawnBehindM = cfg->binWidthM;          /* the ball starts at the origin ... */
    cfg->spawnBeyondM = 10.0F * cfg->binWidthM;  /* ... and is first seen within ~3 frames of it */
    cfg->gateM = 1.5F * cfg->binWidthM;
    cfg->gateMps = 8.0F;              /* drag and fit error, per second of prediction */
    cfg->coastUs = 6000U;
    cfg->impactCoastUs = 18000U;
    cfg->impactRegionM = 0.5F;
    cfg->classifyPoints = 4U;
    cfg->minDepartureMps = 10.0F;     /* the slowest chip leaves faster than this */
    cfg->maxSpeedMps = 100.0F;
    cfg->maxResidualBins = 1.0F;
    cfg->dopplerToleranceMps = 2.5F;
    cfg->fastBallMps = 0.0F;          /* off until the recorded captures say otherwise */
    cfg->fastSupportFraction = 0.55F; /* the Pi detector's FAST_SUPPORT_FRAC */
    cfg->farWindowM = 0.0F;           /* off until the recorded captures say otherwise */
    cfg->corridorGate = 1U;
    cfg->anchorRangeTolM = 0.1F;
    cfg->maxDecelMps2 = 200.0F;
    cfg->rangeNoiseM = 0.012F;
    cfg->wBack = 3.0F;
    cfg->wVel = 2.0F;
    cfg->wResid = 1.0F;
    cfg->wDoppler = 1.0F;
    cfg->wCoherence = 0.5F;
    cfg->wWeaker = 0.5F;
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
    float binsPerM = (cfg->binWidthM > 0.0F) ? 1.0F / cfg->binWidthM : 0.0F;

    memset(hyps, 0, sizeof(*hyps));
    hyps->cfg = *cfg;
    hyps->spawnBehindBins = cfg->spawnBehindM * binsPerM;
    hyps->spawnBeyondBins = cfg->spawnBeyondM * binsPerM;
    hyps->gateBins = cfg->gateM * binsPerM;
    hyps->farWindowBins = cfg->farWindowM * binsPerM;
    hyps->impactRegionBins = cfg->impactRegionM * binsPerM;
    l3_ball_hyps_clear(hyps);
}

void l3_ball_hyps_arm(l3_ball_hyps_t *hyps, const l3_ball_anchor_t *anchor)
{
    l3_ball_hyps_clear(hyps);
    hyps->armed = 1U;
    hyps->anchor = *anchor;
}

/* Signed seconds from earlier to later on the wrapping microsecond clock. */
static float l3_ball_hyps_seconds(uint32_t later, uint32_t earlier)
{
    return (float)(int32_t)(later - earlier) * 1.0e-6F;
}

int32_t l3_ball_points_fit(const l3_ball_hyp_point_t *points, uint32_t count, uint32_t referenceUs,
                           float *rateBinsPerS, float *binAtReference, float *residualBins)
{
    float meanT = 0.0F;
    float meanR = 0.0F;
    float sumTT = 0.0F;
    float sumTR = 0.0F;
    float sse = 0.0F;
    float slope;
    float intercept;
    uint32_t i;

    if (count < 2U) {
        return 0;
    }
    for (i = 0U; i < count; i++) {
        meanT += l3_ball_hyps_seconds(points[i].timestampUs, referenceUs);
        meanR += points[i].rangeBin;
    }
    meanT /= (float)count;
    meanR /= (float)count;
    for (i = 0U; i < count; i++) {
        float dt = l3_ball_hyps_seconds(points[i].timestampUs, referenceUs) - meanT;

        sumTT += dt * dt;
        sumTR += dt * (points[i].rangeBin - meanR);
    }
    if (!(sumTT > 0.0F)) {
        return 0;
    }
    slope = sumTR / sumTT;
    intercept = meanR - slope * meanT;
    for (i = 0U; i < count; i++) {
        float t = l3_ball_hyps_seconds(points[i].timestampUs, referenceUs);
        float error = points[i].rangeBin - (intercept + slope * t);

        sse += error * error;
    }
    *rateBinsPerS = slope;
    *binAtReference = intercept;
    *residualBins = sqrtf(sse / (float)count);
    return 1;
}

int32_t l3_ball_hyp_fit(const l3_ball_hyp_t *hyp, uint32_t referenceUs, float *rateBinsPerS,
                        float *binAtReference, float *residualBins)
{
    return l3_ball_points_fit(hyp->points, hyp->count, referenceUs, rateBinsPerS, binAtReference,
                              residualBins);
}

/* G1: some impact within the anchor's tolerance and some speed in
 * [minDepartureMps, maxSpeedMps] put a ball at rangeBin at timestampUs. */
static int32_t l3_ball_hyps_inCorridor(const l3_ball_hyps_t *hyps, float rangeBin,
                                       uint32_t timestampUs)
{
    const l3_ball_hyps_cfg_t *cfg = &hyps->cfg;
    float dtS = l3_ball_hyps_seconds(timestampUs, hyps->anchor.anchorUs);
    float tolS = (float)hyps->anchor.anchorTolUs * 1.0e-6F;
    float travelledM = (rangeBin - hyps->anchor.anchorBin) * cfg->binWidthM;
    float loM = cfg->minDepartureMps * (dtS - tolS) - cfg->anchorRangeTolM;
    float hiM = cfg->maxSpeedMps * (dtS + tolS) + cfg->anchorRangeTolM;

    return (travelledM >= loM && travelledM <= hiM) ? 1 : 0;
}

/* G2: a hypothesis whose fitted rate after three points is under the slowest
 * ball's is a still return, not a departure. */
static int32_t l3_ball_hyps_stalled(const l3_ball_hyps_cfg_t *cfg, const l3_ball_hyp_t *hyp)
{
    float rate;
    float at;
    float residual;

    if (hyp->count < 3U ||
        !l3_ball_hyp_fit(hyp, hyp->points[hyp->count - 1U].timestampUs, &rate, &at, &residual)) {
        return 0;
    }
    return (rate * cfg->binWidthM < cfg->minDepartureMps) ? 1 : 0;
}

/* Where a hypothesis can be at timestampUs: [lo, hi] around a centre. One
 * point: leaving at anything from minDepartureMps to maxSpeedMps. More: its
 * fitted line, with a gate that widens with the time predicted over. */
static void l3_ball_hyps_window(const l3_ball_hyps_t *hyps, const l3_ball_hyp_t *hyp,
                                uint32_t timestampUs, float *lo, float *hi, float *centre)
{
    const l3_ball_hyps_cfg_t *cfg = &hyps->cfg;
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
        float spread = hyps->gateBins + cfg->gateMps * binsPerM * dtS;

        *centre = atLast + rate * dtS;
        *lo = *centre - spread;
        *hi = *centre + spread;
        return;
    }
    *lo = last->rangeBin + cfg->minDepartureMps * binsPerM * dtS - hyps->gateBins;
    *hi = last->rangeBin + cfg->maxSpeedMps * binsPerM * dtS + hyps->gateBins;
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
    point->anglesValid = 0U;
    point->angleConfidence = 0.0F;
    point->rangeBin = target->rangeBin;
    point->dopplerAliasMps = target->dopplerAliasMps;
    point->stat = target->stat;
    point->clubStat = clubStat;
    point->coherence = target->coherence;
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
    if (hyps->farWindowBins > 0.0F) {
        for (j = 0U; j < n; j++) {
            if (targets[j].rangeBin < hyps->anchor.acceptFromBin + hyps->farWindowBins) {
                taken[j] = 1U;  /* short of the far window: never a ball point */
            }
        }
    }
    if (cfg->corridorGate) {
        for (j = 0U; j < n; j++) {
            if (!taken[j] && !l3_ball_hyps_inCorridor(hyps, targets[j].rangeBin, timestampUs)) {
                taken[j] = 1U;  /* no impact and speed explain it: never a ball point */
            }
        }
    }
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        if (hyps->hyp[i].active) {
            l3_ball_hyps_window(hyps, &hyps->hyp[i], timestampUs, &lo[i], &hi[i], &centre[i]);
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
    /* Coast the hypotheses that found nothing; drop one whose newest point is
     * older than its coast: impactCoastUs inside the impact region, else coastUs. */
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        l3_ball_hyp_t *hyp = &hyps->hyp[i];
        const l3_ball_hyp_point_t *last;
        uint32_t limitUs;

        if (!hyp->active || fed[i]) {
            continue;
        }
        hyp->misses++;
        last = &hyp->points[hyp->count - 1U];
        limitUs = (last->rangeBin < hyps->anchor.anchorBin + hyps->impactRegionBins)
                      ? cfg->impactCoastUs
                      : cfg->coastUs;
        if ((int32_t)(timestampUs - last->timestampUs) > (int32_t)limitUs) {
            hyp->active = 0U;
            hyps->dropped++;
        }
    }
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        l3_ball_hyp_t *hyp = &hyps->hyp[i];

        if (hyp->active && l3_ball_hyps_stalled(cfg, hyp)) {
            hyp->active = 0U;
            hyps->dropped++;
        }
    }
    /* Start hypotheses from what is left in the start band. Its far edge
     * moves out with the time since the gate at the fastest ball's speed: a
     * gate that fired late finds the ball already out, and it must still be
     * able to start. */
    sinceGateS = l3_ball_hyps_seconds(timestampUs, hyps->anchor.anchorUs);
    spawnHi = hyps->anchor.acceptFromBin + hyps->spawnBeyondBins +
              ((sinceGateS > 0.0F && cfg->binWidthM > 0.0F)
                   ? cfg->maxSpeedMps / cfg->binWidthM * sinceGateS
                   : 0.0F);
    for (j = 0U; j < n; j++) {
        float range = targets[j].rangeBin;
        l3_ball_hyp_t *hyp;
        int32_t slot;

        if (taken[j] || range < hyps->anchor.acceptFromBin - hyps->spawnBehindBins ||
            (!cfg->corridorGate && range > spawnHi)) {
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
                                float elevationRad, uint8_t anglesValid,
                                float angleConfidence)
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
    point->angleConfidence = angleConfidence;
    return 1;
}

static float l3_ball_hyps_unit(float x)
{
    return (x < 0.0F) ? 0.0F : ((x > 1.0F) ? 1.0F : x);
}

/* Sum of squared time deviations (s^2) and mean time (s from ref) of a run. */
static float l3_ball_hyps_timeSpread(const l3_ball_hyp_point_t *p, uint32_t n, uint32_t refUs,
                                     float *meanS)
{
    float mean = 0.0F;
    float sum = 0.0F;
    uint32_t k;

    for (k = 0U; k < n; k++) {
        mean += l3_ball_hyps_seconds(p[k].timestampUs, refUs);
    }
    mean /= (float)n;
    for (k = 0U; k < n; k++) {
        float d = l3_ball_hyps_seconds(p[k].timestampUs, refUs) - mean;

        sum += d * d;
    }
    *meanS = mean;
    return sum;
}

/* G4: 1 when the newer half is slower than drag and range noise allow. */
static int32_t l3_ball_hyps_decelerates(const l3_ball_hyps_cfg_t *cfg, const l3_ball_hyp_t *hyp)
{
    uint32_t older = hyp->count / 2U;
    uint32_t newer = hyp->count - older;
    uint32_t ref = hyp->points[0].timestampUs;
    float rateOld;
    float rateNew;
    float at;
    float residual;
    float meanOld;
    float meanNew;
    float spreadOld;
    float spreadNew;
    float sigmaDelta;

    if (!(cfg->maxDecelMps2 > 0.0F) || older < 2U) {
        return 0;
    }
    if (!l3_ball_points_fit(&hyp->points[0], older, ref, &rateOld, &at, &residual) ||
        !l3_ball_points_fit(&hyp->points[older], newer, ref, &rateNew, &at, &residual)) {
        return 0;
    }
    spreadOld = l3_ball_hyps_timeSpread(&hyp->points[0], older, ref, &meanOld);
    spreadNew = l3_ball_hyps_timeSpread(&hyp->points[older], newer, ref, &meanNew);
    sigmaDelta = cfg->rangeNoiseM * sqrtf(1.0F / spreadOld + 1.0F / spreadNew);
    return ((rateOld - rateNew) * cfg->binWidthM >
            cfg->maxDecelMps2 * (meanNew - meanOld) + 2.0F * sigmaDelta)
               ? 1
               : 0;
}

/* 1 - spread/mean of the implied launch speeds of the points later than the
 * anchor's tolerance (earlier ones divide by a time the tolerance swamps);
 * 0.5 with fewer than two such points. */
static float l3_ball_hyps_velocityConsistency(const l3_ball_hyps_t *hyps, const l3_ball_hyp_t *hyp,
                                              float rateMps)
{
    float v[L3_BALL_HYP_POINTS];
    float mean = 0.0F;
    float var = 0.0F;
    uint32_t n = 0U;
    uint32_t k;

    for (k = 0U; k < hyp->count; k++) {
        float dtS = l3_ball_hyps_seconds(hyp->points[k].timestampUs, hyps->anchor.anchorUs);

        if (dtS * 1.0e6F > (float)hyps->anchor.anchorTolUs) {
            v[n++] = (hyp->points[k].rangeBin - hyps->anchor.anchorBin) * hyps->cfg.binWidthM / dtS;
        }
    }
    if (n < 2U || !(rateMps > 0.0F)) {
        return 0.5F;
    }
    for (k = 0U; k < n; k++) {
        mean += v[k];
    }
    mean /= (float)n;
    for (k = 0U; k < n; k++) {
        var += (v[k] - mean) * (v[k] - mean);
    }
    return 1.0F - l3_ball_hyps_unit(sqrtf(var / (float)n) / rateMps);
}

/* Judge one hypothesis as the ball (see l3_ball_hyps_classify). Returns 1
 * and fills out (index included) when it qualifies. */
static int32_t l3_ball_hyps_judge(const l3_ball_hyps_t *hyps, uint32_t i,
                                  l3_ball_hyp_verdict_t *out)
{
    const l3_ball_hyps_cfg_t *cfg = &hyps->cfg;
    const l3_ball_hyp_t *hyp = &hyps->hyp[i];
    float rate;
    float atAnchor;
    float residual;
    float rateMps;
    float originOffsetS;
    float agree = 0.0F;
    float weaker = 0.0F;
    float withClub = 0.0F;
    float weakerFraction;
    float coherence = 0.0F;
    float backTerm;
    uint32_t k;

    if (!hyp->active || hyp->count < cfg->classifyPoints) {
        return 0;
    }
    if (!l3_ball_hyp_fit(hyp, hyps->anchor.anchorUs, &rate, &atAnchor, &residual) ||
        !(rate > 0.0F)) {
        return 0;
    }
    rateMps = rate * cfg->binWidthM;
    if (rateMps < cfg->minDepartureMps || rateMps > cfg->maxSpeedMps) {
        return 0;
    }
    originOffsetS = (hyps->anchor.anchorBin - atAnchor) / rate;
    if (fabsf(originOffsetS) * 1.0e6F > (float)hyps->anchor.anchorTolUs) {
        return 0;
    }
    if (residual > cfg->maxResidualBins) {
        return 0;
    }
    if (l3_ball_hyps_decelerates(cfg, hyp)) {
        return 0;
    }
    for (k = 0U; k < hyp->count; k++) {
        const l3_ball_hyp_point_t *p = &hyp->points[k];

        coherence += p->coherence;
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
    out->velocityConsistency = l3_ball_hyps_velocityConsistency(hyps, hyp, rateMps);
    out->coherence = coherence / (float)hyp->count;
    out->anchorSource = hyps->anchor.source;
    backTerm = (hyps->anchor.anchorTolUs > 0U)
                   ? l3_ball_hyps_unit(1.0F - fabsf(out->originOffsetUs) /
                                                  (float)hyps->anchor.anchorTolUs)
                   : 0.0F;
    out->score = cfg->wBack * backTerm + cfg->wVel * out->velocityConsistency +
                 cfg->wResid * l3_ball_hyps_unit(1.0F - residual / cfg->maxResidualBins) +
                 cfg->wDoppler * out->dopplerAgreement + cfg->wCoherence * out->coherence +
                 cfg->wWeaker * weakerFraction;
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
         * within classifyPoints points or its coast, so this is bounded. */
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
