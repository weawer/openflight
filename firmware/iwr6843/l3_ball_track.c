/* See l3_ball_track.h. */
#include <stdio.h>
#include <string.h>

#include "l3_ball_track.h"
#include "l3_text.h"

static const char *const kWhyNames[L3_BALL_TRACK_WHY_COUNT] = {
    "none", "unarmed", "nocandidate", "acquired", "confirmed", "tooslow", "toofast", "tracked",
    "coasted", "lost", "searching"
};

void l3_ball_track_cfg_defaults(l3_ball_track_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    l3_track_cfg_defaults(&cfg->core);
    cfg->core.gateBins = 6.0F;        /* a 70 m/s ball moves ~4.5 bins per 3 ms frame */
    /* A departing ball smears within a frame: its first points read 0.04-0.13
     * on the labelled swings, and at the club's 0.2 the follow-through was
     * taken instead. The second point's range rate refuses slow returns. */
    cfg->core.minConfidence = 0.05F;
    cfg->core.maxMisses = 1U;
    /* The core is seeded one point at a time (a confirmed hypothesis, the
     * leave fallback's points) and must start on it: the ball's candidates
     * are its hypotheses (l3_ball_hyp), not the club's approach steps. */
    cfg->core.acquireMaxStepBins = 0.0F;
    /* The club rules (ascending bins, at most two per bin) describe the
     * approach; the ball tracker has its own departure tests. */
    cfg->core.ascendingOnly = 0U;
    cfg->core.maxSameBinPoints = 0U;
    cfg->core.approachMaxSameBinPoints = 0U;
    cfg->core.standingFrames = 0U;
    cfg->minDepartureMps = 10.0F;     /* the slowest chip leaves faster than this */
    cfg->maxSpeedMps = 100.0F;
    cfg->originGateBins = 8.0F;       /* the first post frame is at most ~5 bins out */
    cfg->minDepartureBins = 1.0F;     /* the impact echo sits at the origin itself */
    cfg->displaceConfidence = 0.2F;   /* a clean return, as the club tracker needs */
    cfg->launchPoints = 6U;
    cfg->snr = 1.0F;                  /* the floor itself: the ball is weak and moving */
    cfg->useHypotheses = 0U;          /* decided by the recorded captures */
    cfg->skipClubClaim = 1U;
    cfg->gateTolUs = 15000U;          /* the gate is not the exact impact */
    cfg->anchorMaxSigmaUs = 3000.0F;
    l3_ball_fit_cfg_defaults(&cfg->fit);
#if L3_BALL_HYPOTHESES
    l3_ball_hyps_cfg_defaults(&cfg->hyps);
#endif
#if L3_BALL_RECOVER
    cfg->recover = 1U;
    cfg->historySnr = 0.0F;           /* the same as snr */
    l3_ball_recover_cfg_defaults(&cfg->rec);
#endif
}

void l3_ball_track_init(l3_ball_track_t *track, const l3_ball_track_cfg_t *cfg)
{
    memset(track, 0, sizeof(*track));
    track->cfg = *cfg;
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    l3_track_init(&track->core, &cfg->core);
#if L3_BALL_HYPOTHESES
    /* The hypotheses share the core's geometry: one source for both. */
    track->cfg.hyps.binWidthM = cfg->core.binWidthM;
    track->cfg.hyps.velocitySpanMps = cfg->core.velocitySpanMps;
    track->verdict.index = -1;
    l3_ball_hyps_init(&track->hyps, &track->cfg.hyps);
#endif
#if L3_BALL_RECOVER
    /* Recovery shares the core's geometry and the hypotheses' tolerances. */
    track->cfg.rec.binWidthM = cfg->core.binWidthM;
    track->cfg.rec.velocitySpanMps = cfg->core.velocitySpanMps;
    track->cfg.rec.maxResidualBins = track->cfg.hyps.maxResidualBins;
    track->cfg.rec.dopplerToleranceMps = track->cfg.hyps.dopplerToleranceMps;
    l3_ball_history_reset(&track->history);
#endif
}

void l3_ball_track_reset(l3_ball_track_t *track)
{
    l3_track_reset(&track->core);
    track->armed = 0U;
    track->confirmed = 0U;
    track->done = 0U;
    track->why = L3_BALL_TRACK_WHY_NONE;
    track->impactTimestampUs = 0U;
    track->originBin = 0.0F;
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    memset(&track->origin, 0, sizeof(track->origin));
    memset(&track->anchor, 0, sizeof(track->anchor));
#if L3_BALL_HYPOTHESES
    l3_ball_hyps_init(&track->hyps, &track->cfg.hyps);
    memset(&track->verdict, 0, sizeof(track->verdict));
    track->verdict.index = -1;
#endif
#if L3_BALL_RECOVER
    l3_ball_history_reset(&track->history);
#endif
}

void l3_ball_track_arm(l3_ball_track_t *track, const l3_ball_anchor_t *anchor,
                       const l3_vec3_t *origin)
{
    l3_ball_track_reset(track);
    track->armed = 1U;
    track->originBin = anchor->acceptFromBin;
    track->origin = *origin;
    track->impactTimestampUs = anchor->gateUs;
    track->anchor = *anchor;
#if L3_BALL_HYPOTHESES
    l3_ball_hyps_arm(&track->hyps, anchor);
#endif
}

void l3_ball_track_anchor(const l3_ball_track_t *track, float teeBin, float acceptFromBin,
                          uint32_t gateUs, const l3_impact_fit_cfg_t *fitCfg,
                          const l3_club_track_t *club, l3_ball_anchor_t *out)
{
    l3_ball_anchor_make(teeBin, acceptFromBin, gateUs, track->cfg.gateTolUs, fitCfg, club,
                        track->cfg.anchorMaxSigmaUs, out);
}

static int32_t l3_ball_track_note(l3_ball_track_t *track, uint8_t why, int32_t appended)
{
    track->why = why;
    track->counters[why]++;
    return appended;
}

/* The second point's range rate must be a ball's; the core is reset when not. */
static int32_t l3_ball_track_confirm(l3_ball_track_t *track)
{
    l3_track_point_t newest;

    (void)l3_track_point(&track->core, track->core.count - 1U, &newest);
    if (newest.radialVelocityMps < track->cfg.minDepartureMps) {
        l3_track_reset(&track->core);
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_TOO_SLOW, 0);
    }
    if (newest.radialVelocityMps > track->cfg.maxSpeedMps) {
        l3_track_reset(&track->core);
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_TOO_FAST, 0);
    }
    track->confirmed = 1U;
    return l3_ball_track_note(track, L3_BALL_TRACK_WHY_CONFIRMED, 1);
}

int32_t l3_ball_track_seed(l3_ball_track_t *track, const l3_target_obs_t *first,
                           const l3_target_obs_t *second)
{
    const l3_target_obs_t *pair[2];
    float gateBins = track->core.cfg.gateBins;
    uint32_t k;

    if (!track->armed || track->done || track->confirmed) {
        return 0;
    }
    pair[0] = first;
    pair[1] = second;
    l3_track_reset(&track->core);
    /* Known to be the ball: neither the frame-counted gate nor the
     * acquisition's confidence may refuse the pair (as l3_ball_track_adopt). */
    track->core.cfg.gateBins = 1.0e9F;
    for (k = 0U; k < 2U; k++) {
        l3_target_obs_t point = *pair[k];

        point.confidence = 1.0F;
        if (!l3_track_update(&track->core, &point, 1U, point.frame, point.timestampUs)) {
            track->core.cfg.gateBins = gateBins;
            l3_track_reset(&track->core);
            return l3_ball_track_note(track, L3_BALL_TRACK_WHY_NO_CANDIDATE, 0);
        }
    }
    track->core.cfg.gateBins = gateBins;
    return l3_ball_track_confirm(track);
}

/* The most confident return in the origin gate at least displaceConfidence
 * and more confident than the unconfirmed first point, or -1: a smeared stray
 * taken first must not keep the ball, arriving behind it, from being offered
 * (20260916_184748). */
static int32_t l3_ball_track_displacer(const l3_ball_track_t *track,
                                       const l3_target_obs_t *targets, uint32_t n)
{
    l3_track_point_t first;
    int32_t best = -1;
    uint32_t i;

    if (!l3_track_point(&track->core, 0U, &first)) {
        return -1;
    }
    for (i = 0U; i < n; i++) {
        float beyond = targets[i].rangeBin - track->originBin;

        if (beyond < track->cfg.minDepartureBins || beyond > track->cfg.originGateBins ||
            targets[i].confidence < track->cfg.displaceConfidence ||
            targets[i].confidence <= first.confidence) {
            continue;
        }
        if (best < 0 || targets[i].confidence > targets[best].confidence) {
            best = (int32_t)i;
        }
    }
    return best;
}

/* The core's gate around its prediction for this frame. */
static int32_t l3_ball_track_inGate(const l3_club_track_t *core, const l3_target_obs_t *target,
                                    uint32_t frame)
{
    float predicted = core->lastBin + core->velocityBinsPerFrame * (float)(frame - core->lastFrame);
    float error = target->rangeBin - predicted;

    return ((error < 0.0F) ? -error : error) <= core->cfg.gateBins;
}

static int32_t l3_ball_track_step(l3_ball_track_t *track, const l3_target_obs_t *targets,
                                  uint32_t n, uint32_t frame, uint32_t timestampUs,
                                  uint32_t skipIndex)
{
    l3_target_obs_t candidates[L3_OBS_MAX_TARGETS];
    uint32_t indices[L3_OBS_MAX_TARGETS];  /* candidate -> targets index */
    uint32_t kept = 0U;
    uint32_t i;
    int32_t appended;

    memset(candidates, 0, sizeof(candidates));
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    if (!track->armed) {
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_UNARMED, 0);
    }
    if (track->done) {
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_LOST, 0);
    }
    /* A ball never comes back toward the radar: nothing behind the last point
     * (or, before acquisition, at or short of the origin) can be it. Before
     * acquisition every candidate in the departure band is offered and the
     * core takes the most confident; the ball does not reliably outrun the
     * club's follow-through at once (the hypothesis search handles that). */
    if (track->core.active && track->confirmed) {
        uint8_t otherInGate = 0U;

        for (i = 0U; i < n && kept < L3_OBS_MAX_TARGETS; i++) {
            if (targets[i].rangeBin < track->core.lastBin - 0.5F || i == skipIndex) {
                continue;
            }
            otherInGate |= (uint8_t)l3_ball_track_inGate(&track->core, &targets[i], frame);
            indices[kept] = i;
            candidates[kept++] = targets[i];
        }
        if (skipIndex < n && !otherInGate && kept < L3_OBS_MAX_TARGETS &&
            targets[skipIndex].rangeBin >= track->core.lastBin - 0.5F) {
            /* Only the club's return is in the gate: the two share a bin. */
            indices[kept] = skipIndex;
            candidates[kept++] = targets[skipIndex];
        }
    } else {
        /* Acquiring, or confirming from a single point (whose prediction is
         * the point itself): only candidates in the departure band beyond the
         * reference (the origin, then the first point) are offered, which
         * excludes the impact echo, the resting club and the follow-through
         * behind the ball; the core then takes the most confident. */
        float reference = track->core.active ? track->core.lastBin : track->originBin;
        float span = track->core.active ? track->cfg.core.gateBins : track->cfg.originGateBins;

        for (i = 0U; i < n && kept < L3_OBS_MAX_TARGETS; i++) {
            float beyond = targets[i].rangeBin - reference;

            if (beyond < track->cfg.minDepartureBins || beyond > span) {
                continue;
            }
            indices[kept] = i;
            candidates[kept++] = targets[i];
        }
    }
    if (kept == 0U && !track->core.active) {
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_NO_CANDIDATE, 0);
    }
    appended = l3_track_update(&track->core, candidates, kept, frame, timestampUs);
    if (!appended && !track->confirmed && track->core.count == 1U) {
        /* Nothing confirmed the first point: a confident departure replaces it. */
        int32_t displacer = l3_ball_track_displacer(track, targets, n);

        if (displacer >= 0) {
            l3_track_reset(&track->core);
            appended = l3_track_update(&track->core, &targets[displacer], 1U, frame,
                                       timestampUs);
            if (appended) {
                track->lastTargetIndex = (uint32_t)displacer;
                return l3_ball_track_note(track, L3_BALL_TRACK_WHY_ACQUIRED, 1);
            }
        }
    }
    if (appended && track->core.lastTargetIndex < kept) {
        track->lastTargetIndex = indices[track->core.lastTargetIndex];
    }
    if (!appended) {
        if (!track->core.active) {
            if (track->confirmed) {
                track->done = 1U;
                return l3_ball_track_note(track, L3_BALL_TRACK_WHY_LOST, 0);
            }
            return l3_ball_track_note(track, L3_BALL_TRACK_WHY_NO_CANDIDATE, 0);
        }
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_COASTED, 0);
    }
    if (track->core.count == 1U) {
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_ACQUIRED, 1);
    }
    if (!track->confirmed) {
        return l3_ball_track_confirm(track);
    }
    return l3_ball_track_note(track, L3_BALL_TRACK_WHY_TRACKED, 1);
}

#if L3_BALL_HYPOTHESES
/* Once the ball is chosen (or the search is over) the hypotheses claim no
 * target: nothing downstream estimates angles for them. */
static void l3_ball_track_quietHyps(l3_ball_track_t *track)
{
    uint32_t i;

    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        track->hyps.hyp[i].lastTargetIndex = L3_BALL_HYP_NONE;
    }
}

/* The classified hypothesis becomes the track: its points (angles included)
 * seed the core in order, and tracking carries on from them. With recovery
 * the frames it missed are first recovered from the history and merged in. */
static int32_t l3_ball_track_adopt(l3_ball_track_t *track, uint32_t index)
{
    const l3_ball_hyp_t *hyp = &track->hyps.hyp[index];
    const l3_ball_hyp_point_t *points = hyp->points;
    uint32_t count = hyp->count;
    float gateBins = track->core.cfg.gateBins;
    uint32_t k;
#if L3_BALL_RECOVER
    static l3_ball_hyp_point_t merged[L3_TRACK_POINTS];
    l3_ball_recover_result_t rec;

    if (track->cfg.recover) {
        count = l3_ball_recover(&track->cfg.rec, &track->history, hyp,
                                track->anchor.acceptFromBin, merged, L3_TRACK_POINTS, &rec);
        points = merged;
        track->verdict.recovered = rec.recovered;
        track->verdict.recoveredFirstFrame = rec.firstFrame;
        track->verdict.recoveredMask = rec.mask;
    }
#endif

    l3_track_reset(&track->core);
    /* The points were associated on timestamps by the hypothesis already;
     * the core's frame-counted gate must not refuse them on the way in. */
    track->core.cfg.gateBins = 1.0e9F;
    for (k = 0U; k < count; k++) {
        const l3_ball_hyp_point_t *p = &points[k];
        l3_target_obs_t seed;

        memset(&seed, 0, sizeof(seed));
        seed.frame = p->frame;
        seed.timestampUs = p->timestampUs;
        seed.peakBin = (uint8_t)(p->rangeBin + 0.5F);
        seed.rangeBin = p->rangeBin;
        seed.stat = p->stat;
        seed.peak = p->stat;
        seed.dopplerAliasMps = p->dopplerAliasMps;
        seed.coherence = p->coherence;
        seed.confidence = 1.0F;
        if (!l3_track_update(&track->core, &seed, 1U, p->frame, p->timestampUs)) {
            track->core.cfg.gateBins = gateBins;
            l3_track_reset(&track->core);
            return l3_ball_track_note(track, L3_BALL_TRACK_WHY_SEARCHING, 0);
        }
        if (p->anglesValid) {
            (void)l3_track_set_angles(&track->core, p->azimuthRad, p->elevationRad,
                                      p->anglesValid, p->angleConfidence);
        }
    }
    track->core.cfg.gateBins = gateBins;
    track->confirmed = 1U;
    track->lastTargetIndex = hyp->lastTargetIndex;
    l3_ball_track_quietHyps(track);
    return l3_ball_track_note(track, L3_BALL_TRACK_WHY_CONFIRMED,
                              (hyp->lastTargetIndex != L3_BALL_HYP_NONE) ? 1 : 0);
}
#endif /* L3_BALL_HYPOTHESES */

/* l3_ball_track_update_joint over the targets the track may use (the snr
 * filter already applied); indices it reports are into these targets. */
static int32_t l3_ball_track_joint(l3_ball_track_t *track, const l3_target_obs_t *targets,
                                   uint32_t n, uint32_t frame, uint32_t timestampUs,
                                   uint32_t clubIndex)
{
#if L3_BALL_HYPOTHESES
    uint32_t skip = track->cfg.skipClubClaim ? clubIndex : L3_TRACK_NO_TARGET;

    if (!track->cfg.useHypotheses) {
        return l3_ball_track_step(track, targets, n, frame, timestampUs, L3_TRACK_NO_TARGET);
    }
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    if (!track->armed || track->done || track->confirmed) {
        l3_ball_track_quietHyps(track);
    }
    if (!track->armed) {
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_UNARMED, 0);
    }
    if (track->done) {
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_LOST, 0);
    }
    if (track->confirmed) {
        return l3_ball_track_step(track, targets, n, frame, timestampUs, skip);
    }
    (void)l3_ball_hyps_update(&track->hyps, targets, n, frame, timestampUs, clubIndex);
    l3_ball_hyps_classify(&track->hyps, &track->verdict);
    if (track->verdict.index < 0) {
        return l3_ball_track_note(track, L3_BALL_TRACK_WHY_SEARCHING, 0);
    }
    return l3_ball_track_adopt(track, (uint32_t)track->verdict.index);
#else
    (void)clubIndex;
    return l3_ball_track_step(track, targets, n, frame, timestampUs, L3_TRACK_NO_TARGET);
#endif
}

#if L3_BALL_RECOVER
/* An index into the filtered targets back to the caller's list. */
static uint32_t l3_ball_track_original(const uint32_t *original, uint32_t kept, uint32_t index)
{
    return (index < kept) ? original[index] : index;
}
#endif

int32_t l3_ball_track_update_joint(l3_ball_track_t *track, const l3_target_obs_t *targets,
                                   uint32_t n, uint32_t frame, uint32_t timestampUs,
                                   uint32_t clubIndex)
{
#if L3_BALL_RECOVER
    static l3_target_obs_t usable[L3_OBS_MAX_TARGETS];
    static uint32_t original[L3_OBS_MAX_TARGETS];  /* usable index -> caller index */
    uint32_t kept = 0U;
    uint32_t keptClub = L3_TRACK_NO_TARGET;
    uint32_t i;
    int32_t appended;

    if (!track->cfg.recover) {
        return l3_ball_track_joint(track, targets, n, frame, timestampUs, clubIndex);
    }
    if (track->cfg.useHypotheses && track->armed && !track->done && !track->confirmed) {
        /* Searching: the history keeps the whole frame, weaker returns included. */
        l3_ball_history_push(&track->history, targets, n, frame, timestampUs, clubIndex);
    }
    if (!(track->cfg.historySnr > 0.0F && track->cfg.historySnr < track->cfg.snr)) {
        return l3_ball_track_joint(track, targets, n, frame, timestampUs, clubIndex);
    }
    /* Of this track, only the history holds the returns under snr: every
     * branch (legacy, search, confirmed) sees the caller's targets at snr, and
     * every index it reports is mapped back into the caller's list. (The
     * caller's club follow sees the lowered list: l3_ball_track_extract_snr.) */
    for (i = 0U; i < n && i < L3_OBS_MAX_TARGETS; i++) {
        if (targets[i].snr >= track->cfg.snr) {
            if (i == clubIndex) {
                keptClub = kept;
            }
            original[kept] = i;
            usable[kept++] = targets[i];
        }
    }
    appended = l3_ball_track_joint(track, usable, kept, frame, timestampUs, keptClub);
    track->lastTargetIndex = l3_ball_track_original(original, kept, track->lastTargetIndex);
    for (i = 0U; i < L3_BALL_HYP_MAX; i++) {
        track->hyps.hyp[i].lastTargetIndex =
            l3_ball_track_original(original, kept, track->hyps.hyp[i].lastTargetIndex);
    }
    return appended;
#else
    return l3_ball_track_joint(track, targets, n, frame, timestampUs, clubIndex);
#endif
}

int32_t l3_ball_track_update(l3_ball_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                             uint32_t frame, uint32_t timestampUs)
{
    return l3_ball_track_update_joint(track, targets, n, frame, timestampUs, L3_TRACK_NO_TARGET);
}

void l3_ball_track_note_club(l3_ball_track_t *track, uint32_t frame, uint32_t clubIndex)
{
#if L3_BALL_RECOVER
    l3_ball_history_mark_club(&track->history, frame, clubIndex);
#else
    (void)track;
    (void)frame;
    (void)clubIndex;
#endif
}

float l3_ball_track_extract_snr(const l3_ball_track_cfg_t *cfg, float searchSnr)
{
#if L3_BALL_RECOVER
    /* Only the search fills the history: the legacy acquisition keeps snr. */
    if (cfg->recover && cfg->useHypotheses && cfg->historySnr > 0.0F &&
        cfg->historySnr < searchSnr) {
        return cfg->historySnr;
    }
#else
    (void)cfg;
#endif
    return searchSnr;
}

uint32_t l3_ball_track_struct_bytes(void)
{
    return (uint32_t)sizeof(l3_ball_track_t);
}

int32_t l3_ball_track_set_angles(l3_ball_track_t *track, float azimuthRad, float elevationRad,
                                 uint8_t anglesValid, float angleConfidence)
{
    return l3_track_set_angles(&track->core, azimuthRad, elevationRad, anglesValid,
                               angleConfidence);
}

uint32_t l3_ball_track_launch(const l3_ball_track_t *track, l3_launch_t *out)
{
    l3_delivery_t fit;
    uint32_t used;

    memset(out, 0, sizeof(*out));
    if (!track->confirmed) {
        return 0U;
    }
    used = l3_track_delivery_range(&track->core, 0U, track->cfg.launchPoints,
                                   track->cfg.launchPoints, &fit);
    if (used == 0U) {
        return 0U;
    }
    l3_launch_from_delivery(&fit, track->impactTimestampUs, out);
    /* The direction is l3_ball_track_reconstruct's, once per shot. */
    out->hlaValid = 0U;
    out->vlaValid = 0U;
    out->hlaRad = 0.0F;
    out->vlaRad = 0.0F;
    return used;
}

uint32_t l3_ball_track_reconstruct(l3_ball_track_t *track, l3_launch_t *launch)
{
    l3_ball_fit_t fit;
    l3_vec3_t u;

    launch->hlaValid = 0U;
    launch->vlaValid = 0U;
    launch->hlaRad = 0.0F;
    launch->vlaRad = 0.0F;
    launch->anglesAccepted = 0U;
    launch->angleRmsRad = 0.0F;
    launch->angleWhy = L3_BALL_FIT_WHY_NONE;
    if (!track->confirmed) {
        l3_track_unfilter_all(&track->core);
        return 0U;
    }
    (void)l3_ball_fit_run(&track->cfg.fit, &track->origin, &track->core, &fit);
    launch->anglesAccepted = (uint8_t)((fit.accepted > 0xFFU) ? 0xFFU : fit.accepted);
    launch->angleRmsRad = fit.rmsRad;
    launch->angleWhy = fit.why;
    if (!fit.valid) {
        return 0U;
    }
    launch->hlaRad = fit.hlaRad;
    launch->vlaRad = fit.vlaRad;
    launch->hlaValid = 1U;
    launch->vlaValid = 1U;
    launch->launchPosition = fit.tee;
    if (launch->speedValid) {
        l3_ball_fit_direction(fit.hlaRad, fit.vlaRad, &u);
        launch->velocity.x = launch->speedMps * u.x;
        launch->velocity.y = launch->speedMps * u.y;
        launch->velocity.z = launch->speedMps * u.z;
    }
    return fit.accepted;
}

const char *l3_ball_track_why_name(uint8_t why)
{
    return (why < L3_BALL_TRACK_WHY_COUNT) ? kWhyNames[why] : "?";
}

int32_t l3_ball_track_format_status(const l3_ball_track_t *track, char *out, uint32_t cap)
{
    char originText[16];
    char binText[16];
#if L3_BALL_HYPOTHESES
    unsigned recovered = (unsigned)track->verdict.recovered;
#else
    unsigned recovered = 0U;
#endif

    l3_text_fixed2(track->originBin, originText, sizeof(originText));
    l3_text_fixed2(track->core.lastBin, binText, sizeof(binText));
    return snprintf(out, cap,
                    "balltrack armed=%u confirmed=%u done=%u why=%s count=%u origin=%s bin=%s "
                    "impact=%u acq=%u slow=%u fast=%u lost=%u rec=%u",
                    (unsigned)track->armed, (unsigned)track->confirmed, (unsigned)track->done,
                    l3_ball_track_why_name(track->why), (unsigned)track->core.count, originText,
                    binText, (unsigned)track->impactTimestampUs,
                    (unsigned)track->counters[L3_BALL_TRACK_WHY_ACQUIRED],
                    (unsigned)track->counters[L3_BALL_TRACK_WHY_TOO_SLOW],
                    (unsigned)track->counters[L3_BALL_TRACK_WHY_TOO_FAST],
                    (unsigned)track->counters[L3_BALL_TRACK_WHY_LOST], recovered);
}

