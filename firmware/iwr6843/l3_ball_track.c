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
    cfg->core.maxMisses = 1U;
    /* The club rules (ascending bins, at most two per bin) describe the
     * approach; the ball tracker has its own departure tests. */
    cfg->core.ascendingOnly = 0U;
    cfg->core.maxSameBinPoints = 0U;
    cfg->minDepartureMps = 20.0F;     /* TrackMan Aug: rejects club-speed decoys, keeps chips */
    cfg->maxSpeedMps = 100.0F;
    cfg->originGateBins = 8.0F;       /* the first post frame is at most ~5 bins out */
    cfg->minDepartureBins = 1.0F;     /* the impact echo sits at the origin itself */
    cfg->launchPoints = 6U;
    cfg->snr = 3.0F;                  /* half the trigger's: the ball is weak and moving */
    cfg->useHypotheses = 1U;          /* TrackMan Aug 3 ms: 121/124 follow the ball vs 68 */
    cfg->skipClubClaim = 1U;
#if L3_BALL_HYPOTHESES
    l3_ball_hyps_cfg_defaults(&cfg->hyps);
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
#if L3_BALL_HYPOTHESES
    l3_ball_hyps_init(&track->hyps, &track->cfg.hyps);
    memset(&track->verdict, 0, sizeof(track->verdict));
    track->verdict.index = -1;
#endif
}

void l3_ball_track_arm(l3_ball_track_t *track, float originBin, const l3_vec3_t *origin,
                       uint32_t impactTimestampUs)
{
    l3_ball_track_reset(track);
    track->armed = 1U;
    track->originBin = originBin;
    track->origin = *origin;
    track->impactTimestampUs = impactTimestampUs;
#if L3_BALL_HYPOTHESES
    l3_ball_hyps_arm(&track->hyps, originBin, impactTimestampUs);
#endif
}

static int32_t l3_ball_track_note(l3_ball_track_t *track, uint8_t why, int32_t appended)
{
    track->why = why;
    track->counters[why]++;
    return appended;
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
 * seed the core in order, and tracking carries on from them. */
static int32_t l3_ball_track_adopt(l3_ball_track_t *track, uint32_t index)
{
    const l3_ball_hyp_t *hyp = &track->hyps.hyp[index];
    float gateBins = track->core.cfg.gateBins;
    uint32_t k;

    l3_track_reset(&track->core);
    /* The points were associated on timestamps by the hypothesis already;
     * the core's frame-counted gate must not refuse them on the way in. */
    track->core.cfg.gateBins = 1.0e9F;
    for (k = 0U; k < hyp->count; k++) {
        const l3_ball_hyp_point_t *p = &hyp->points[k];
        l3_target_obs_t seed;

        memset(&seed, 0, sizeof(seed));
        seed.frame = p->frame;
        seed.timestampUs = p->timestampUs;
        seed.peakBin = (uint8_t)(p->rangeBin + 0.5F);
        seed.rangeBin = p->rangeBin;
        seed.stat = p->stat;
        seed.peak = p->stat;
        seed.dopplerAliasMps = p->dopplerAliasMps;
        seed.coherence = 1.0F;
        seed.confidence = 1.0F;
        if (!l3_track_update(&track->core, &seed, 1U, p->frame, p->timestampUs)) {
            track->core.cfg.gateBins = gateBins;
            l3_track_reset(&track->core);
            return l3_ball_track_note(track, L3_BALL_TRACK_WHY_SEARCHING, 0);
        }
        if (p->anglesValid) {
            (void)l3_track_set_angles(&track->core, p->azimuthRad, p->elevationRad,
                                      p->anglesValid);
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

int32_t l3_ball_track_update_joint(l3_ball_track_t *track, const l3_target_obs_t *targets,
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

int32_t l3_ball_track_update(l3_ball_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                             uint32_t frame, uint32_t timestampUs)
{
    return l3_ball_track_update_joint(track, targets, n, frame, timestampUs, L3_TRACK_NO_TARGET);
}

uint32_t l3_ball_track_struct_bytes(void)
{
    return (uint32_t)sizeof(l3_ball_track_t);
}

int32_t l3_ball_track_set_angles(l3_ball_track_t *track, float azimuthRad, float elevationRad,
                                 uint8_t anglesValid)
{
    return l3_track_set_angles(&track->core, azimuthRad, elevationRad, anglesValid);
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
    return used;
}

const char *l3_ball_track_why_name(uint8_t why)
{
    return (why < L3_BALL_TRACK_WHY_COUNT) ? kWhyNames[why] : "?";
}

int32_t l3_ball_track_format_status(const l3_ball_track_t *track, char *out, uint32_t cap)
{
    char originText[16];
    char binText[16];

    l3_text_fixed2(track->originBin, originText, sizeof(originText));
    l3_text_fixed2(track->core.lastBin, binText, sizeof(binText));
    return snprintf(out, cap,
                    "balltrack armed=%u confirmed=%u done=%u why=%s count=%u origin=%s bin=%s "
                    "impact=%u acq=%u slow=%u fast=%u lost=%u",
                    (unsigned)track->armed, (unsigned)track->confirmed, (unsigned)track->done,
                    l3_ball_track_why_name(track->why), (unsigned)track->core.count, originText,
                    binText, (unsigned)track->impactTimestampUs,
                    (unsigned)track->counters[L3_BALL_TRACK_WHY_ACQUIRED],
                    (unsigned)track->counters[L3_BALL_TRACK_WHY_TOO_SLOW],
                    (unsigned)track->counters[L3_BALL_TRACK_WHY_TOO_FAST],
                    (unsigned)track->counters[L3_BALL_TRACK_WHY_LOST]);
}

