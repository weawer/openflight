/* IWR6843 club track. See l3_club_track.h. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "l3_club_track.h"
#include "l3_text.h"
#include "l3_track_kf.h"

static const char *const kWhyNames[L3_TRACK_WHY_COUNT] = {
    "none", "acquired", "associated", "coasted", "dropped", "idle", "released"
};

void l3_track_cfg_defaults(l3_track_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->binWidthM = 6.0F / 128.0F;
    cfg->gateBins = 3.0F;        /* a driver moves ~2.5 bins per 3 ms frame */
    cfg->maxMisses = 2U;
    cfg->minConfidence = 0.2F;
    cfg->weightRange = 1.0F;     /* one bin of range error ... */
    cfg->weightVelocity = 1.0F;  /* ... equals a full wrap of Doppler ... */
    cfg->weightQuality = 1.0F;   /* ... equals a target of no confidence */
    cfg->weightStrength = 0.5F;  /* prefer targets with higher MTI-residual SNR */
    cfg->velocitySpanMps = 2.0F * L3_OBS_WAVELENGTH_M / (4.0F * 135.0e-6F);
    l3_cal_identity(&cfg->cal, L3_CAL_MAX_VIRTUAL);
    cfg->minAcquireDopplerMps = 1.0F;
    cfg->maxAngleResidualM = 2.0F * cfg->binWidthM;  /* two bins of scatter */
    cfg->ascendingOnly = 1U;
    cfg->maxSameBinPoints = 2U;
    cfg->followDopplerTolMps = 4.0F;
    cfg->followDopplerRiseMps = 1.5F;
    cfg->approachMaxSameBinPoints = 1U;
    cfg->standingFrames = 2U;
    cfg->acquireMinStepBins = 0.75F;   /* 5th percentile of the labelled club steps */
    /* Off: on the labelled swings (2026-10-01) candidates rescued swings past
     * the golfer but confirmed steps between still returns and the hands, so
     * captures with no swing fired. 4.5 (a 70 m/s club's step) turns it on;
     * revisit once the clutter model damps the still returns. */
    cfg->acquireMaxStepBins = 0.0F;
    cfg->acquireExpectedStepBins = 2.0F; /* the labelled approaches' median, 2.0-2.1 */
    /* Off (half the alias span): frame to frame the club's aliased Doppler
     * moves by a median 2.3 m/s and a tenth of steps by 7-8 (labelled swings,
     * 2026-10-01), so it only ranks pairs (l3_track_misfit). */
    cfg->acquireDopplerTolMps = 9.0F;
    cfg->acquireMinConfidence = 0.0F;  /* the club past the golfer reads 0.0-0.2 */
    l3_track_kf_cfg_defaults(&cfg->kf);
}

static float l3_track_absf(float value)
{
    return (value < 0.0F) ? -value : value;
}

/* A point's golf-frame position from its range and whatever angles it has. */
static void l3_track_locate(const l3_club_track_t *track, l3_track_point_t *point)
{
    float azimuth = (point->anglesValid & L3_OBS_ANGLE_AZIMUTH) ? point->azimuthRad : 0.0F;
    float elevation = (point->anglesValid & L3_OBS_ANGLE_ELEVATION) ? point->elevationRad : 0.0F;

    l3_frames_observe(&track->cfg.cal, point->rangeM, azimuth, elevation, &point->position);
}

void l3_track_init(l3_club_track_t *track, const l3_track_cfg_t *cfg)
{
    memset(track, 0, sizeof(*track));
    track->cfg = *cfg;
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
}

void l3_track_reset(l3_club_track_t *track)
{
    track->active = 0U;
    track->why = L3_TRACK_WHY_NONE;
    track->next = 0U;
    track->count = 0U;
    track->misses = 0U;
    track->lastFrame = 0U;
    track->lastBin = 0.0F;
    track->velocityBinsPerFrame = 0.0F;
    track->predictedBin = 0.0F;
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    track->sameBin = 0;
    track->sameBinCount = 0U;
    track->following = 0U;
    track->tentative = 0U;
    track->followBinsPerS = 0.0F;
    track->releasedValid = 0U;
    track->releasedBin = 0.0F;
    track->releasedDopplerMps = 0.0F;
    track->candidateCount = 0U;
}

static void l3_track_note(l3_club_track_t *track, uint8_t why)
{
    track->why = why;
    track->counters[why]++;
}

static void l3_track_push(l3_club_track_t *track)
{
    track->next = (track->next + 1U) % L3_TRACK_POINTS;
    if (track->count < L3_TRACK_POINTS) {
        track->count++;
    }
    track->total++;
}

static void l3_track_append(l3_club_track_t *track, const l3_target_obs_t *target,
                            float velocityBinsPerFrame, float dtS)
{
    l3_track_point_t *point = &track->points[track->next];

    point->frame = target->frame;
    point->timestampUs = target->timestampUs;
    point->rangeBin = target->rangeBin;
    point->rangeM = target->rangeBin * track->cfg.binWidthM;
    point->radialVelocityMps = (dtS > 0.0F)
                                   ? (velocityBinsPerFrame * track->cfg.binWidthM / dtS)
                                   : 0.0F;
    point->dopplerAliasMps = target->dopplerAliasMps;
    point->azimuthRad = target->azimuthRad;
    point->elevationRad = target->elevationRad;
    point->anglesValid = target->anglesValid;
    point->energy = target->energy;
    point->coherence = target->coherence;
    point->confidence = target->confidence;
    l3_track_locate(track, point);
    /* A target's own angles (a seeded or synthetic point) carry no estimate
     * of their quality: full weight when present, none when absent. */
    point->angleConfidence = target->anglesValid ? 1.0F : 0.0F;
    l3_track_point_unfilter(point);
    l3_track_push(track);
}

void l3_track_append_point(l3_club_track_t *track, const l3_track_point_t *point)
{
    l3_track_point_t *slot = &track->points[track->next];

    *slot = *point;
    l3_track_locate(track, slot);
    l3_track_point_unfilter(slot);
    track->lastBin = point->rangeBin;
    track->lastFrame = point->frame;
    l3_track_push(track);
}

float l3_track_wrapped_diff(float a, float b, float span)
{
    float diff = a - b;

    if (span <= 0.0F) {
        return 0.0F;
    }
    while (diff > 0.5F * span) {
        diff -= span;
    }
    while (diff < -0.5F * span) {
        diff += span;
    }
    return (diff < 0.0F) ? -diff : diff;
}

static int32_t l3_track_roundBin(float rangeBin)
{
    return (int32_t)floorf(rangeBin + 0.5F);
}

/* True for a target whose bin a target has stood within one bin of for
 * standingFrames frames running (counted before this frame). */
static uint8_t l3_track_standing(const l3_club_track_t *track, float rangeBin)
{
    int32_t bin = l3_track_roundBin(rangeBin);

    return (track->cfg.standingFrames > 0U && bin >= 0 && bin < (int32_t)L3_TRACK_GLOBAL_BINS &&
            track->standHold[bin] >= track->cfg.standingFrames) ? 1U : 0U;
}

/* True when, on the frame before `frame`, a return other than one at
 * `ownBin` stood within a bin of rangeBin. A club stepping on finds its new
 * bin empty (bar its own last return, when it steps under a bin and a half);
 * a hop between two still returns does not. */
static uint8_t l3_track_heldLastFrame(const l3_club_track_t *track, uint32_t frame,
                                      float rangeBin, float ownBin)
{
    uint32_t i;

    if (track->prevFrame + 1U != frame) {
        return 0U;
    }
    for (i = 0U; i < track->prevCount; i++) {
        if (l3_track_absf(track->prevBins[i] - ownBin) > 1.0e-3F &&
            l3_track_absf(track->prevBins[i] - rangeBin) <= 1.0F) {
            return 1U;
        }
    }
    return 0U;
}

/* Remember this update's target bins for the next frame's confirmation. */
static void l3_track_notePrev(l3_club_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                              uint32_t frame)
{
    uint32_t i;

    if (n > L3_OBS_MAX_TARGETS) {
        n = L3_OBS_MAX_TARGETS;
    }
    for (i = 0U; i < n; i++) {
        track->prevBins[i] = targets[i].rangeBin;
    }
    track->prevCount = n;
    track->prevFrame = frame;
}

/* After a frame's decision: extend or break each bin's run of frames with a
 * target within one bin of it. */
static void l3_track_holdFrame(l3_club_track_t *track, const l3_target_obs_t *targets, uint32_t n)
{
    uint8_t seen[L3_TRACK_GLOBAL_BINS];
    uint32_t i;
    uint32_t bin;

    if (track->cfg.standingFrames == 0U) {
        return;
    }
    memset(seen, 0, sizeof(seen));
    for (i = 0U; i < n; i++) {
        int32_t centre = l3_track_roundBin(targets[i].rangeBin);
        int32_t offset;

        for (offset = -1; offset <= 1; offset++) {
            int32_t marked = centre + offset;

            if (marked >= 0 && marked < (int32_t)L3_TRACK_GLOBAL_BINS) {
                seen[marked] = 1U;
            }
        }
    }
    for (bin = 0U; bin < L3_TRACK_GLOBAL_BINS; bin++) {
        if (seen[bin] != 0U) {
            if (track->standHold[bin] < 0xFFU) {
                track->standHold[bin]++;
            }
        } else {
            track->standHold[bin] = 0U;
        }
    }
}

/* Count the newest point into its rounded bin's run of consecutive points. */
static void l3_track_countBin(l3_club_track_t *track, float rangeBin, int32_t first)
{
    int32_t bin = l3_track_roundBin(rangeBin);

    if (!first && bin == track->sameBin) {
        track->sameBinCount++;
    } else {
        track->sameBin = bin;
        track->sameBinCount = 1U;
    }
}


/* Forget the track, remembering its last return so acquisition does not take
 * it straight back. */
static void l3_track_release(l3_club_track_t *track)
{
    float releasedBin = track->lastBin;
    float releasedDoppler =
        track->points[(track->next + L3_TRACK_POINTS - 1U) % L3_TRACK_POINTS].dopplerAliasMps;

    l3_track_reset(track);
    track->releasedValid = 1U;
    track->releasedBin = releasedBin;
    track->releasedDopplerMps = releasedDoppler;
    l3_track_note(track, L3_TRACK_WHY_RELEASED);
}

/* Before a tentative point: remember the track as it is, to put it back if
 * the point is withdrawn. */
static void l3_track_hold(l3_club_track_t *track)
{
    l3_track_held_t *held = &track->held;

    held->active = track->active;
    held->following = track->following;
    held->misses = track->misses;
    held->lastFrame = track->lastFrame;
    held->lastBin = track->lastBin;
    held->velocityBinsPerFrame = track->velocityBinsPerFrame;
    held->followBinsPerS = track->followBinsPerS;
    held->sameBin = track->sameBin;
    held->sameBinCount = track->sameBinCount;
}

/* Take back the tentative newest point: pop it from the ring and put the
 * track back as it was before it; the withdrawn point's frame and the frames
 * since count as misses. */
static void l3_track_withdraw(l3_club_track_t *track)
{
    const l3_track_held_t *held = &track->held;
    uint32_t since = track->misses + 1U;

    track->next = (track->next + L3_TRACK_POINTS - 1U) % L3_TRACK_POINTS;
    if (track->count > 0U) {
        track->count--;
    }
    if (track->total > 0U) {
        track->total--;
    }
    track->active = held->active;
    track->following = held->following;
    track->misses = held->misses + since;
    track->lastFrame = held->lastFrame;
    track->lastBin = held->lastBin;
    track->velocityBinsPerFrame = held->velocityBinsPerFrame;
    track->followBinsPerS = held->followBinsPerS;
    track->sameBin = held->sameBin;
    track->sameBinCount = held->sameBinCount;
    track->tentative = 0U;
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
}

/* The newest held point's slot (meaningful only when count > 0). */
static const l3_track_point_t *l3_track_newest(const l3_club_track_t *track)
{
    return &track->points[(track->next + L3_TRACK_POINTS - 1U) % L3_TRACK_POINTS];
}

/* How badly a target fits a track (lower is better): rangeErr bins off
 * where it should be, its aliased Doppler this far (wrapped) from
 * dopplerMps, its confidence and its strength (1/snr is 0 for a very strong
 * mover and 1 just above the floor: prefer the stronger MTI residual). */
static float l3_track_misfit(const l3_track_cfg_t *cfg, const l3_target_obs_t *target,
                             float rangeErr, float dopplerMps)
{
    float span = (cfg->velocitySpanMps > 0.0F) ? cfg->velocitySpanMps : 1.0F;
    float velocityErr = l3_track_wrapped_diff(target->dopplerAliasMps, dopplerMps,
                                              cfg->velocitySpanMps) / span;
    float strengthMisfit = (target->snr > 0.0F) ? (1.0F / target->snr) : 1.0F;

    return cfg->weightRange * rangeErr + cfg->weightVelocity * velocityErr +
           cfg->weightQuality * (1.0F - target->confidence) + cfg->weightStrength * strengthMisfit;
}

static uint8_t l3_track_moves(const l3_track_cfg_t *cfg, const l3_target_obs_t *target)
{
    return (uint8_t)(cfg->minAcquireDopplerMps <= 0.0F ||
                     l3_track_absf(target->dopplerAliasMps) >= cfg->minAcquireDopplerMps);
}

/* A target acquisition may start from: at least minConfidence, not standing
 * in its bin, and not the return the last released track ended on. */
static uint8_t l3_track_acquirable(const l3_club_track_t *track, const l3_target_obs_t *target,
                                   float minConfidence)
{
    const l3_track_cfg_t *cfg = &track->cfg;

    if (target->confidence < minConfidence || l3_track_standing(track, target->rangeBin)) {
        return 0U;
    }
    return (uint8_t)!(track->releasedValid &&
                      l3_track_absf(target->rangeBin - track->releasedBin) <= cfg->gateBins &&
                      l3_track_wrapped_diff(target->dopplerAliasMps, track->releasedDopplerMps,
                                            cfg->velocitySpanMps) <=
                          L3_TRACK_RELEASE_DOPPLER_TOL_MPS);
}

/* The acquirable target a single frame prefers: one that reads as moving (a
 * stationary body in the lane is often the strongest return), then the most
 * confident. `taken` (may be NULL) marks targets already chosen. Returns its
 * index, L3_TRACK_NO_TARGET for none. */
static uint32_t l3_track_pickBest(const l3_club_track_t *track, const l3_target_obs_t *targets,
                                  uint32_t n, float minConfidence, const uint8_t *taken)
{
    uint32_t best = L3_TRACK_NO_TARGET;
    uint8_t bestMoves = 0U;
    uint32_t i;

    for (i = 0U; i < n; i++) {
        uint8_t moves = l3_track_moves(&track->cfg, &targets[i]);

        if ((taken != NULL && taken[i]) || !l3_track_acquirable(track, &targets[i], minConfidence)) {
            continue;
        }
        if (best == L3_TRACK_NO_TARGET || (moves && !bestMoves) ||
            (moves == bestMoves && targets[i].confidence > targets[best].confidence)) {
            best = i;
            bestMoves = moves;
        }
    }
    return best;
}

/* Start a track on `first`, observed at `frame`. */
static void l3_track_start(l3_club_track_t *track, const l3_target_obs_t *first, uint32_t frame)
{
    track->active = 1U;
    track->misses = 0U;
    track->lastFrame = frame;
    track->lastBin = first->rangeBin;
    track->velocityBinsPerFrame = 0.0F;
    track->predictedBin = first->rangeBin;
    track->releasedValid = 0U;
    track->candidateCount = 0U;
    l3_track_append(track, first, 0.0F, 0.0F);
    l3_track_countBin(track, first->rangeBin, 1);
}

/* The held candidate this frame's targets confirm: a target stepping on from
 * it by acquireMinStepBins..acquireMaxStepBins a frame (ascending: the club
 * closes on the ball) with its aliased Doppler within acquireDopplerTolMps.
 * Of several, the pair that fits best as association would judge it
 * (l3_track_misfit): the step's distance from acquireExpectedStepBins, the
 * Doppler agreement, and both points' confidence and strength. Returns 1
 * with *candidate and *index. */
static int32_t l3_track_confirm(const l3_club_track_t *track, const l3_target_obs_t *targets,
                                uint32_t n, uint32_t frame, uint32_t *candidate,
                                uint32_t *index)
{
    const l3_track_cfg_t *cfg = &track->cfg;
    float bestScore = 0.0F;
    int32_t found = 0;
    uint32_t c;
    uint32_t i;

    for (c = 0U; c < track->candidateCount; c++) {
        const l3_target_obs_t *held = &track->candidates[c];
        uint32_t gap = frame - held->frame;

        if (gap == 0U || gap > L3_TRACK_CANDIDATE_MAX_GAP_FRAMES) {
            continue;
        }
        for (i = 0U; i < n; i++) {
            float step = (targets[i].rangeBin - held->rangeBin) / (float)gap;
            float score;

            if (step < cfg->acquireMinStepBins || step > cfg->acquireMaxStepBins ||
                !l3_track_acquirable(track, &targets[i], cfg->acquireMinConfidence) ||
                l3_track_heldLastFrame(track, frame, targets[i].rangeBin, held->rangeBin) ||
                l3_track_wrapped_diff(targets[i].dopplerAliasMps, held->dopplerAliasMps,
                                      cfg->velocitySpanMps) > cfg->acquireDopplerTolMps) {
                continue;
            }
            /* The step's misfit is the second point's; the first point's
             * quality and strength count as well. */
            score = l3_track_misfit(cfg, &targets[i],
                                    l3_track_absf(step - cfg->acquireExpectedStepBins),
                                    held->dopplerAliasMps) +
                    cfg->weightQuality * (1.0F - held->confidence) +
                    cfg->weightStrength * ((held->snr > 0.0F) ? (1.0F / held->snr) : 1.0F);
            if (!found || score < bestScore) {
                found = 1;
                bestScore = score;
                *candidate = c;
                *index = i;
            }
        }
    }
    return found;
}

/* Hold this frame's acquirable targets as candidates, preferred first (as a
 * single frame would pick); held ones still young enough to be confirmed fill
 * what is left, newest first. */
static void l3_track_hold_candidates(l3_club_track_t *track, const l3_target_obs_t *targets,
                                     uint32_t n, uint32_t frame)
{
    l3_target_obs_t kept[L3_TRACK_CANDIDATES];
    uint8_t taken[L3_OBS_MAX_TARGETS];
    uint32_t count = 0U;
    uint32_t age;
    uint32_t c;

    memset(taken, 0, sizeof(taken));
    if (n > L3_OBS_MAX_TARGETS) {
        n = L3_OBS_MAX_TARGETS;
    }
    while (count < L3_TRACK_CANDIDATES) {
        uint32_t best = l3_track_pickBest(track, targets, n, track->cfg.acquireMinConfidence, taken);

        if (best == L3_TRACK_NO_TARGET) {
            break;
        }
        taken[best] = 1U;
        kept[count++] = targets[best];
    }
    /* A candidate from `age` frames ago can still be confirmed next frame
     * while age + 1 is within the gap. */
    for (age = 1U; age < L3_TRACK_CANDIDATE_MAX_GAP_FRAMES; age++) {
        for (c = 0U; c < track->candidateCount && count < L3_TRACK_CANDIDATES; c++) {
            if (frame - track->candidates[c].frame == age) {
                kept[count++] = track->candidates[c];
            }
        }
    }
    memcpy(track->candidates, kept, count * sizeof(kept[0]));
    track->candidateCount = count;
}

/* Start a track, or note idle unless quietWhenIdle (which leaves the last
 * "why", a release, standing). With acquireMaxStepBins > 0 a track starts on
 * a confirmed candidate approach, its two points at once; otherwise on the
 * best target of this frame (l3_track_pickBest at minConfidence). */
static int32_t l3_track_acquire(l3_club_track_t *track, const l3_target_obs_t *targets,
                                uint32_t n, uint32_t frame, int32_t quietWhenIdle)
{
    const l3_track_cfg_t *cfg = &track->cfg;
    uint32_t index = L3_TRACK_NO_TARGET;
    uint32_t candidate = 0U;

    if (cfg->acquireMaxStepBins <= 0.0F) {
        index = l3_track_pickBest(track, targets, n, cfg->minConfidence, NULL);
        if (index != L3_TRACK_NO_TARGET) {
            l3_track_start(track, &targets[index], frame);
        }
    } else if (l3_track_confirm(track, targets, n, frame, &candidate, &index)) {
        const l3_target_obs_t first = track->candidates[candidate];
        uint32_t gap = frame - first.frame;
        float step = (targets[index].rangeBin - first.rangeBin) / (float)gap;
        float dtS = (float)(targets[index].timestampUs - first.timestampUs) * 1.0e-6F;

        l3_track_start(track, &first, first.frame);
        track->velocityBinsPerFrame = step;
        l3_track_append(track, &targets[index], step, dtS / (float)gap);
        track->lastFrame = frame;
        track->lastBin = targets[index].rangeBin;
        track->predictedBin = targets[index].rangeBin;
        l3_track_countBin(track, targets[index].rangeBin, 0);
    } else {
        l3_track_hold_candidates(track, targets, n, frame);
    }
    if (index == L3_TRACK_NO_TARGET) {
        if (!quietWhenIdle) {
            l3_track_note(track, L3_TRACK_WHY_IDLE);
        }
        return 0;
    }
    track->lastTargetIndex = index;
    l3_track_note(track, L3_TRACK_WHY_ACQUIRED);
    return 1;
}

/* After impact: the club is a departing return beyond the band (or the
 * ball, without a band) whose rate from the ball at the impact time is
 * positive, no faster than it arrived (x L3_TRACK_FOLLOW_MAX_RATIO) and
 * slower than the ball, not the ball's target and at least minConfidence;
 * the strongest such return. With a last point (from != NULL, an active track
 * the band hid), a return that left that point at the ball's rate or faster
 * is refused as well, as association refuses it. Nothing when the approach is
 * unknown (only the ceiling bounds the club) and the ball has no rate yet.
 * Returns its index, L3_TRACK_NO_TARGET for none. Shared by re-acquisition
 * and the hidden club's re-emergence. */
static uint32_t l3_track_pickBeyondBand(const l3_club_track_t *track, const l3_target_obs_t *targets,
                                        uint32_t n,
                                        uint32_t timestampUs, const l3_follow_ctx_t *ctx,
                                        const l3_track_point_t *from, float minConfidence)
{
    float dtS = (float)(int32_t)(timestampUs - ctx->impactTimestampUs) * 1.0e-6F;
    float fromDtS = (from != NULL)
                        ? (float)(int32_t)(timestampUs - from->timestampUs) * 1.0e-6F
                        : 0.0F;
    float edge = ctx->bandValid ? ctx->bandHiBin : ctx->originBin;
    float maxRate = ctx->approachBinsPerS * L3_TRACK_FOLLOW_MAX_RATIO;
    uint32_t bestIndex = L3_TRACK_NO_TARGET;
    uint32_t i;

    if (dtS <= 0.0F || ctx->approachBinsPerS <= 0.0F ||
        (!ctx->approachKnown && ctx->ballBinsPerS <= 0.0F)) {
        return L3_TRACK_NO_TARGET;
    }
    for (i = 0U; i < n; i++) {
        float rate = (targets[i].rangeBin - ctx->originBin) / dtS;

        if (targets[i].confidence < minConfidence || l3_track_standing(track, targets[i].rangeBin)) {
            continue;
        }
        if (i == ctx->ballClaimIndex || targets[i].rangeBin <= edge || rate <= 0.0F ||
            rate > maxRate || (ctx->ballBinsPerS > 0.0F && rate >= ctx->ballBinsPerS)) {
            continue;
        }
        if (from != NULL && ctx->ballBinsPerS > 0.0F && fromDtS > 0.0F &&
            (targets[i].rangeBin - from->rangeBin) / fromDtS >= ctx->ballBinsPerS) {
            continue;
        }
        if (bestIndex == L3_TRACK_NO_TARGET || targets[i].stat > targets[bestIndex].stat) {
            bestIndex = i;
        }
    }
    return bestIndex;
}

/* After impact the tee band hides the club for as long as it takes to cross
 * it at its arriving speed; it is coasted, not dropped, until then plus a
 * frame. */
static int32_t l3_track_coasting_across_band(const l3_club_track_t *track,
                                             const l3_follow_ctx_t *ctx, uint32_t timestampUs,
                                             const l3_track_point_t *last)
{
    float crossUs;

    if (ctx == NULL || !ctx->bandValid || track->followBinsPerS <= 0.0F ||
        track->lastBin >= ctx->bandHiBin) {
        return 0;
    }
    crossUs = (ctx->bandHiBin - track->lastBin) / track->followBinsPerS * 1.0e6F +
              (float)ctx->frameUs;
    return ((float)(int32_t)(timestampUs - last->timestampUs) <= crossUs) ? 1 : 0;
}

/* One frame of association for an active track; coast, then drop, when
 * nothing qualifies. Approaching: the best-scoring target in the gate around
 * the prediction, never below the last point's rounded bin when
 * ascendingOnly. Following after impact: the strongest target from
 * L3_TRACK_FOLLOW_RETREAT_BINS behind the last point to where the club would
 * be at its impact speed (fitted in bins per second on the first follow),
 * plus L3_TRACK_FOLLOW_LEAD_BINS -- the club is the
 * stronger of the two returns after impact and only slows from its impact
 * speed, so it is anywhere between; beyond, it would be moving faster than
 * it arrived, which only the ball does -- and never a third consecutive point
 * in one bin, which a stall beside the ball is and the club is not. With a
 * follow context (following only): never the ball's claimed target nor a
 * return that left the last point at the ball's rate or faster, and a frame
 * with nothing coasts while the club is still crossing the tee band. */
static int32_t l3_track_associate(l3_club_track_t *track, const l3_target_obs_t *targets,
                                  uint32_t n, uint32_t frame, uint32_t timestampUs,
                                  int32_t following, const l3_follow_ctx_t *ctx)
{
    const l3_track_cfg_t *cfg = &track->cfg;
    uint32_t elapsed = frame - track->lastFrame;
    float predicted = track->lastBin + track->velocityBinsPerFrame * (float)elapsed;
    int32_t floorBin = l3_track_roundBin(track->lastBin);
    const l3_target_obs_t *best = NULL;
    float bestScore = 0.0F;
    const l3_track_point_t *last = l3_track_newest(track);
    int32_t withdrawn = 0;
    uint32_t i;

    track->predictedBin = predicted;
    for (i = 0U; i < n; i++) {
        float rangeErr = l3_track_absf(targets[i].rangeBin - predicted);
        float score;

        if (following) {
            float dtS = (float)(int32_t)(timestampUs - last->timestampUs) * 1.0e-6F;
            float reach = track->lastBin + track->followBinsPerS * ((dtS > 0.0F) ? dtS : 0.0F) +
                          L3_TRACK_FOLLOW_LEAD_BINS;

            if (targets[i].rangeBin < track->lastBin - L3_TRACK_FOLLOW_RETREAT_BINS ||
                targets[i].rangeBin > reach) {
                continue;
            }
            /* A return that has stood in its bin is not the club, however
             * strong: the stall wobbles between two rounded bins, which the
             * same-bin rule below never sees. */
            if (l3_track_standing(track, targets[i].rangeBin)) {
                continue;
            }
            /* After impact the club stalls; the same bin for too many consecutive
             * follow-through frames is a stationary return, not the club. */
            if (cfg->maxSameBinPoints > 0U &&
                l3_track_roundBin(targets[i].rangeBin) == track->sameBin &&
                track->sameBinCount >= cfg->maxSameBinPoints) {
                continue;
            }
            if (ctx != NULL) {
                if (i == ctx->ballClaimIndex) {
                    continue;
                }
                if (ctx->ballBinsPerS > 0.0F && dtS > 0.0F &&
                    (targets[i].rangeBin - track->lastBin) / dtS >= ctx->ballBinsPerS) {
                    continue;
                }
            }
            /* The club's Doppler runs on smoothly through impact and decays; a
             * return that reads a different velocity (the shaft and hands
             * lag the head) is not the club however strong it is. */
            if (cfg->followDopplerTolMps > 0.0F && cfg->velocitySpanMps > 0.0F) {
                float span = cfg->velocitySpanMps;
                float step = targets[i].dopplerAliasMps - last->dopplerAliasMps;

                step -= span * floorf(step / span + 0.5F); /* wrapped, signed */
                if (step < -cfg->followDopplerTolMps || step > cfg->followDopplerRiseMps) {
                    continue;
                }
            }
            if (best == NULL || targets[i].stat > best->stat) {
                best = &targets[i];
                track->lastTargetIndex = i;
            }
            continue;
        }
        if (rangeErr > cfg->gateBins) {
            continue;
        }
        /* A standing return is not the club, except the one the track is on
         * (the same-bin release handles that) and one a track already moving
         * at club speed predicts the club into: the club merges with a
         * standing return as it sweeps through its bin. */
        if (l3_track_standing(track, targets[i].rangeBin) &&
            l3_track_roundBin(targets[i].rangeBin) != floorBin &&
            !(l3_track_absf(track->velocityBinsPerFrame) >= L3_TRACK_STANDING_PASS_BINS_PER_FRAME &&
              rangeErr <= L3_TRACK_STANDING_PASS_ERR_BINS)) {
            continue;
        }
        if (cfg->ascendingOnly && l3_track_roundBin(targets[i].rangeBin) < floorBin) {
            continue;
        }
        score = l3_track_misfit(cfg, &targets[i], rangeErr, last->dopplerAliasMps);
        if (best == NULL || score < bestScore) {
            best = &targets[i];
            bestScore = score;
            track->lastTargetIndex = i;
        }
    }
    if (best != NULL && track->tentative &&
        best->rangeBin - track->lastBin < L3_TRACK_TENTATIVE_ADVANCE_BINS) {
        /* The point after a tentative one did not move on downrange: the
         * tentative point was not a moving club. Take it back; this frame
         * takes nothing. */
        l3_track_withdraw(track);
        if (!track->active) {
            l3_track_note(track, L3_TRACK_WHY_DROPPED);
            return 0;
        }
        best = NULL;
        last = l3_track_newest(track);
        withdrawn = 1;
    }
    if (best != NULL) {
        track->tentative = 0U; /* confirmed, or never tentative */
    }
    if (best == NULL && !withdrawn && following && ctx != NULL && ctx->bandValid &&
        track->lastBin < ctx->bandHiBin) {
        /* The band hid the club and nothing is in the follow window: a return
         * beyond the band that re-acquisition would take is the club
         * re-emerging, however slowly it was fitted going in. */
        uint32_t index =
            l3_track_pickBeyondBand(track, targets, n, timestampUs, ctx, last, cfg->minConfidence);

        if (index != L3_TRACK_NO_TARGET) {
            /* Tentative, as a re-acquisition is, and like one with no
             * frame-to-frame velocity across the gap. */
            l3_track_hold(track);
            track->tentative = 1U;
            l3_track_append(track, &targets[index], 0.0F, 0.0F);
            /* The fit going in was the dwell, not the club's speed: follow
             * at the approach from here, as a re-acquired club does. */
            track->followBinsPerS = ctx->approachBinsPerS;
            track->velocityBinsPerFrame = 0.0F;
            track->misses = 0U;
            track->lastFrame = frame;
            track->lastBin = targets[index].rangeBin;
            track->lastTargetIndex = index;
            l3_track_countBin(track, targets[index].rangeBin, 1);
            l3_track_note(track, L3_TRACK_WHY_ASSOCIATED);
            return 1;
        }
    }
    if (best == NULL) {
        track->lastTargetIndex = L3_TRACK_NO_TARGET;
        track->misses++;
        if (following && l3_track_coasting_across_band(track, ctx, timestampUs, last)) {
            l3_track_note(track, L3_TRACK_WHY_COASTED);
            return 0;
        }
        if (track->misses > cfg->maxMisses && track->tentative) {
            /* Nothing confirmed the tentative point before the miss rule
             * would drop it: take it back and judge the track as it was. */
            l3_track_withdraw(track);
            if (track->active && following &&
                l3_track_coasting_across_band(track, ctx, timestampUs, l3_track_newest(track))) {
                l3_track_note(track, L3_TRACK_WHY_COASTED);
                return 0;
            }
        }
        if (track->misses > cfg->maxMisses) {
            track->active = 0U;
            l3_track_note(track, L3_TRACK_WHY_DROPPED);
        } else {
            l3_track_note(track, L3_TRACK_WHY_COASTED);
        }
        return 0;
    }
    if (elapsed > 0U) {
        float measured = (best->rangeBin - track->lastBin) / (float)elapsed;
        float dtS = (float)(timestampUs - last->timestampUs) * 1.0e-6F;
        /* Half new, half old: smooth enough to predict with, quick enough
         * for a club that accelerates through the approach. */
        track->velocityBinsPerFrame = (track->count > 1U)
                                          ? 0.5F * (track->velocityBinsPerFrame + measured)
                                          : measured;
        l3_track_append(track, best, track->velocityBinsPerFrame, dtS / (float)elapsed);
    } else {
        l3_track_append(track, best, track->velocityBinsPerFrame, 0.0F);
    }
    track->misses = 0U;
    track->lastFrame = frame;
    track->lastBin = best->rangeBin;
    l3_track_countBin(track, best->rangeBin, 0);
    l3_track_note(track, L3_TRACK_WHY_ASSOCIATED);
    return 1;
}

static int32_t l3_track_updateFrame(l3_club_track_t *track, const l3_target_obs_t *targets,
                                    uint32_t n, uint32_t frame, uint32_t timestampUs)
{
    const l3_track_cfg_t *cfg = &track->cfg;

    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    if (track->active) {
        if (!l3_track_associate(track, targets, n, frame, timestampUs, 0, NULL)) {
            return 0;
        }
        if (cfg->approachMaxSameBinPoints == 0U ||
            track->sameBinCount <= cfg->approachMaxSameBinPoints) {
            return 1;
        }
        /* One point too many in one bin: not the club. Release it, and let
         * this frame's other targets seed the next track at once. */
        l3_track_release(track);
        track->lastTargetIndex = L3_TRACK_NO_TARGET;
        return l3_track_acquire(track, targets, n, frame, 1);
    }
    return l3_track_acquire(track, targets, n, frame, 0);
}

/* After impact with no track: the strongest departing return beyond the
 * band (l3_track_pickBeyondBand) starts a following track, its first point
 * tentative. */
static int32_t l3_track_reacquire(l3_club_track_t *track, const l3_target_obs_t *targets,
                                  uint32_t n, uint32_t frame, uint32_t timestampUs,
                                  const l3_follow_ctx_t *ctx)
{
    uint32_t bestIndex =
        l3_track_pickBeyondBand(track, targets, n, timestampUs, ctx, NULL, track->cfg.minConfidence);
    const l3_target_obs_t *best;

    if (bestIndex == L3_TRACK_NO_TARGET) {
        return 0;
    }
    best = &targets[bestIndex];
    /* Tentative until the next point moves on (l3_track_follow). */
    l3_track_hold(track);
    track->tentative = 1U;
    track->active = 1U;
    track->misses = 0U;
    track->following = 1U;
    track->followBinsPerS = ctx->approachBinsPerS;
    track->velocityBinsPerFrame = 0.0F;
    track->lastFrame = frame;
    track->lastBin = best->rangeBin;
    track->lastTargetIndex = bestIndex;
    l3_track_append(track, best, 0.0F, 0.0F);
    l3_track_countBin(track, best->rangeBin, 1);
    l3_track_note(track, L3_TRACK_WHY_ACQUIRED);
    return 1;
}

static int32_t l3_track_followFrame(l3_club_track_t *track, const l3_target_obs_t *targets,
                                    uint32_t n, uint32_t frame, uint32_t timestampUs,
                                    const l3_follow_ctx_t *ctx)
{
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    if (!track->active) {
        return (ctx != NULL) ? l3_track_reacquire(track, targets, n, frame, timestampUs, ctx) : 0;
    }
    if (!track->following) {
        /* The first frame after impact: the club can only slow from here.
         * Too few points for a fit (a track reacquired just before impact)
         * gives the rate between its last two points; none at all, the
         * context's approach speed. */
        float slope = l3_track_recent_rate(track);

        track->following = 1U;
        track->followBinsPerS = (slope > 0.0F) ? slope : 0.0F;
        if (track->followBinsPerS <= 0.0F && ctx != NULL && ctx->approachBinsPerS > 0.0F) {
            track->followBinsPerS = ctx->approachBinsPerS;
        }
    }
    return l3_track_associate(track, targets, n, frame, timestampUs, 1, ctx);
}

int32_t l3_track_update(l3_club_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                        uint32_t frame, uint32_t timestampUs)
{
    int32_t appended = l3_track_updateFrame(track, targets, n, frame, timestampUs);

    l3_track_holdFrame(track, targets, n);
    l3_track_notePrev(track, targets, n, frame);
    return appended;
}

int32_t l3_track_follow(l3_club_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                        uint32_t frame, uint32_t timestampUs, const l3_follow_ctx_t *ctx)
{
    int32_t appended = l3_track_followFrame(track, targets, n, frame, timestampUs, ctx);

    l3_track_holdFrame(track, targets, n);
    return appended;
}

float l3_track_recent_rate(const l3_club_track_t *track)
{
    float slope = 0.0F;
    float residual = 0.0F;
    l3_track_point_t newest;
    l3_track_point_t previous;
    float dtS;

    if (l3_track_fit(track, L3_TRACK_FOLLOW_FIT_POINTS, &slope, &residual) != 0U) {
        return slope;
    }
    if (track->count < 2U || !l3_track_point(track, track->count - 1U, &newest) ||
        !l3_track_point(track, track->count - 2U, &previous)) {
        return 0.0F;
    }
    dtS = (float)(int32_t)(newest.timestampUs - previous.timestampUs) * 1.0e-6F;
    return (dtS > 0.0F) ? (newest.rangeBin - previous.rangeBin) / dtS : 0.0F;
}

int32_t l3_track_point(const l3_club_track_t *track, uint32_t index, l3_track_point_t *out)
{
    uint32_t oldest;

    if (index >= track->count) {
        return 0;
    }
    oldest = (track->next + L3_TRACK_POINTS - track->count) % L3_TRACK_POINTS;
    *out = track->points[(oldest + index) % L3_TRACK_POINTS];
    return 1;
}

int32_t l3_track_find_point(const l3_club_track_t *track, uint32_t timestampUs,
                            uint32_t *index)
{
    uint32_t oldest = (track->next + L3_TRACK_POINTS - track->count) % L3_TRACK_POINTS;
    uint32_t i;

    for (i = 0U; i < track->count; i++) {
        if (track->points[(oldest + i) % L3_TRACK_POINTS].timestampUs == timestampUs) {
            *index = i;
            return 1;
        }
    }
    return 0;
}

void l3_track_point_unfilter(l3_track_point_t *point)
{
    point->filteredPosition = point->position;
    point->filterAccepted = 0U;
    point->filterHypothesis = L3_FILTER_HYP_UNFILTERED;
}

l3_track_point_t *l3_track_point_mut(l3_club_track_t *track, uint32_t index)
{
    uint32_t oldest;

    if (index >= track->count) {
        return NULL;
    }
    oldest = (track->next + L3_TRACK_POINTS - track->count) % L3_TRACK_POINTS;
    return &track->points[(oldest + index) % L3_TRACK_POINTS];
}

void l3_track_unfilter_all(l3_club_track_t *track)
{
    uint32_t i;

    for (i = 0U; i < track->count; i++) {
        l3_track_point_unfilter(l3_track_point_mut(track, i));
    }
}

uint32_t l3_track_newest_first(const l3_club_track_t *track, uint32_t maxPoints)
{
    uint32_t used;

    if (maxPoints > L3_TRACK_POINTS) {
        maxPoints = L3_TRACK_POINTS;
    }
    used = (track->count < maxPoints) ? track->count : maxPoints;
    return track->count - used;
}

int32_t l3_track_set_point_angles(l3_club_track_t *track, uint32_t index, float azimuthRad,
                                  float elevationRad, uint8_t anglesValid, float angleConfidence)
{
    l3_track_point_t *point = l3_track_point_mut(track, index);

    if (point == NULL) {
        return 0;
    }
    point->azimuthRad = azimuthRad;
    point->elevationRad = elevationRad;
    point->anglesValid = anglesValid;
    point->angleConfidence = angleConfidence;
    l3_track_locate(track, point);
    l3_track_point_unfilter(point);
    return 1;
}

int32_t l3_track_set_angles(l3_club_track_t *track, float azimuthRad, float elevationRad,
                            uint8_t anglesValid, float angleConfidence)
{
    if (track->lastTargetIndex == L3_TRACK_NO_TARGET || track->count == 0U) {
        return 0;
    }
    return l3_track_set_point_angles(track, track->count - 1U, azimuthRad, elevationRad,
                                     anglesValid, angleConfidence);
}

/* Least squares of one coordinate against time over the selected points:
 * slope, intercept (at t = 0, the newest point's time) and the sum of
 * squared errors. Times are relative to the newest point so the intercept
 * is the fitted position there. Returns 0 when the times do not spread. */
static int32_t l3_track_fitAxis(const float *t, const float *value, uint32_t n, float *slope,
                                float *intercept, float *sumSquares)
{
    float sumT = 0.0F;
    float sumV = 0.0F;
    float sumTT = 0.0F;
    float sumTV = 0.0F;
    float denominator;
    uint32_t i;

    for (i = 0U; i < n; i++) {
        sumT += t[i];
        sumV += value[i];
        sumTT += t[i] * t[i];
        sumTV += t[i] * value[i];
    }
    denominator = (float)n * sumTT - sumT * sumT;
    if (denominator <= 0.0F) {
        return 0;
    }
    *slope = ((float)n * sumTV - sumT * sumV) / denominator;
    *intercept = (sumV - *slope * sumT) / (float)n;
    *sumSquares = 0.0F;
    for (i = 0U; i < n; i++) {
        float error = value[i] - (*intercept + *slope * t[i]);

        *sumSquares += error * error;
    }
    return 1;
}

uint32_t l3_track_delivery(const l3_club_track_t *track, uint32_t maxPoints, l3_delivery_t *out)
{
    uint32_t first = l3_track_newest_first(track, maxPoints);

    return l3_track_delivery_range(track, first, track->count - first, L3_TRACK_FULL_POINTS, out);
}

uint32_t l3_delivery_fit(l3_point_at_fn pointAt, const void *ctx, uint32_t first, uint32_t last,
                         uint32_t fullPoints, float binWidthM, float maxAngleResidualM,
                         l3_delivery_t *out)
{
    float t[L3_TRACK_POINTS];
    float x[L3_TRACK_POINTS];
    float y[L3_TRACK_POINTS];
    float z[L3_TRACK_POINTS];
    float r[L3_TRACK_POINTS];
    uint32_t n = 0U;
    uint32_t withAzimuth = 0U;
    uint32_t withElevation = 0U;
    uint32_t i;
    uint8_t required = 0U;
    float quality = 0.0F;
    float newestUs;
    float squares;
    float total = 0.0F;
    float radialSlope;
    float radialIntercept;
    l3_track_point_t point;

    memset(out, 0, sizeof(*out));
    if (last < first || last - first < 3U) {
        return 0U;
    }
    for (i = first; i < last; i++) {
        (void)pointAt(ctx, i, &point);
        if (point.anglesValid & L3_OBS_ANGLE_AZIMUTH) {
            withAzimuth++;
        }
        if (point.anglesValid & L3_OBS_ANGLE_ELEVATION) {
            withElevation++;
        }
    }
    /* Fit only points that carry the angles the fit will read, so an
     * assumed boresight never mixes with a measured direction. */
    if (withElevation >= 3U && withAzimuth >= 3U) {
        required = L3_OBS_ANGLE_AZIMUTH | L3_OBS_ANGLE_ELEVATION;
    } else if (withElevation >= 3U) {
        required = L3_OBS_ANGLE_ELEVATION;
    }
    (void)pointAt(ctx, last - 1U, &point);
    newestUs = (float)point.timestampUs;
    out->timestampUs = point.timestampUs;
    for (i = first; i < last; i++) {
        (void)pointAt(ctx, i, &point);
        if ((point.anglesValid & required) != required) {
            continue;
        }
        t[n] = ((float)point.timestampUs - newestUs) * 1.0e-6F;
        x[n] = point.position.x;
        y[n] = point.position.y;
        z[n] = point.position.z;
        r[n] = point.rangeM;
        quality += point.confidence;
        n++;
    }
    if (n < 3U) {
        return 0U;
    }
    if (!l3_track_fitAxis(t, x, n, &out->velocity.x, &out->position.x, &squares)) {
        return 0U;
    }
    total += squares;
    (void)l3_track_fitAxis(t, y, n, &out->velocity.y, &out->position.y, &squares);
    total += squares;
    (void)l3_track_fitAxis(t, z, n, &out->velocity.z, &out->position.z, &squares);
    total += squares;
    (void)l3_track_fitAxis(t, r, n, &radialSlope, &radialIntercept, &squares);
    out->points = n;
    out->azimuthPoints = (required & L3_OBS_ANGLE_AZIMUTH) ? n : 0U;
    out->elevationPoints = (required & L3_OBS_ANGLE_ELEVATION) ? n : 0U;
    out->residualM = sqrtf(total / (float)n);
    out->radialSpeedMps = (radialSlope < 0.0F) ? -radialSlope : radialSlope;
    out->speedMps = l3_frames_speed(&out->velocity);
    out->speedValid = 1U;
    if (required == 0U ||
        (maxAngleResidualM > 0.0F && out->residualM > maxAngleResidualM)) {
        /* Under three angled points no direction can be read, and the
         * positions mix assumed boresight with whatever angles there are;
         * or the angled positions do not lie on a line: the angles are noise
         * (a fast ball crossing bins within a burst does this). Either way
         * keep the range walk, which is a measurement, and drop the direction. */
        required = 0U;
        out->azimuthPoints = 0U;
        out->elevationPoints = 0U;
        out->velocity.x = out->radialSpeedMps;
        out->velocity.y = 0.0F;
        out->velocity.z = 0.0F;
        out->speedMps = out->radialSpeedMps;
    }
    if (required & L3_OBS_ANGLE_AZIMUTH) {
        out->pathRad = l3_frames_horizontal_rad(&out->velocity);
        out->pathValid = 1U;
    }
    if (required & L3_OBS_ANGLE_ELEVATION) {
        out->attackRad = l3_frames_vertical_rad(&out->velocity);
        out->attackValid = 1U;
    }
    {
        /* Residual against two range bins of scatter, points against the
         * count a clean approach yields, and the points' own confidence. */
        float residualScore = 1.0F - out->residualM / (2.0F * binWidthM);
        float countScore = (float)n / (float)((fullPoints > 0U) ? fullPoints : 1U);

        if (residualScore < 0.0F) {
            residualScore = 0.0F;
        }
        if (countScore > 1.0F) {
            countScore = 1.0F;
        }
        out->confidence = residualScore * countScore * (quality / (float)n);
    }
    return n;
}

static int32_t l3_track_point_at(const void *ctx, uint32_t index, l3_track_point_t *out)
{
    return l3_track_point((const l3_club_track_t *)ctx, index, out);
}

uint32_t l3_track_delivery_range(const l3_club_track_t *track, uint32_t first, uint32_t count,
                                 uint32_t fullPoints, l3_delivery_t *out)
{
    uint32_t last = first + count;

    memset(out, 0, sizeof(*out));
    if (first >= track->count) {
        return 0U;
    }
    if (last > track->count) {
        last = track->count;
    }
    return l3_delivery_fit(l3_track_point_at, track, first, last, fullPoints,
                           track->cfg.binWidthM, track->cfg.maxAngleResidualM, out);
}

uint32_t l3_track_fit(const l3_club_track_t *track, uint32_t maxPoints, float *slopeBinsPerS,
                      float *residualBins)
{
    uint32_t used = (maxPoints < track->count) ? maxPoints : track->count;
    uint32_t first = track->count - used;
    uint32_t i;
    float t0 = 0.0F;
    float sumT = 0.0F;
    float sumB = 0.0F;
    float sumTT = 0.0F;
    float sumTB = 0.0F;
    float denominator;
    float slope;
    float intercept;
    float residual = 0.0F;
    l3_track_point_t point;

    *slopeBinsPerS = 0.0F;
    *residualBins = 0.0F;
    if (used < 3U) {
        return 0U;
    }
    (void)l3_track_point(track, first, &point);
    t0 = (float)point.timestampUs;
    for (i = first; i < track->count; i++) {
        float t;
        (void)l3_track_point(track, i, &point);
        t = ((float)point.timestampUs - t0) * 1.0e-6F;
        sumT += t;
        sumB += point.rangeBin;
        sumTT += t * t;
        sumTB += t * point.rangeBin;
    }
    denominator = (float)used * sumTT - sumT * sumT;
    if (denominator <= 0.0F) {
        return 0U;
    }
    slope = ((float)used * sumTB - sumT * sumB) / denominator;
    intercept = (sumB - slope * sumT) / (float)used;
    for (i = first; i < track->count; i++) {
        float t;
        float error;
        (void)l3_track_point(track, i, &point);
        t = ((float)point.timestampUs - t0) * 1.0e-6F;
        error = point.rangeBin - (intercept + slope * t);
        residual += error * error;
    }
    *slopeBinsPerS = slope;
    *residualBins = sqrtf(residual / (float)used);
    return used;
}

float l3_track_speed_mps(const l3_club_track_t *track, uint32_t maxPoints)
{
    float slope;
    float residual;

    if (l3_track_fit(track, maxPoints, &slope, &residual) == 0U) {
        return 0.0F;
    }
    return ((slope < 0.0F) ? -slope : slope) * track->cfg.binWidthM;
}

const char *l3_track_why_name(uint8_t why)
{
    return (why < L3_TRACK_WHY_COUNT) ? kWhyNames[why] : "?";
}

int32_t l3_track_format_status(const l3_club_track_t *track, uint32_t destBin, char *out,
                               uint32_t cap)
{
    char binText[16];
    char distText[16];
    char velocityText[16];
    char speedText[16];
    char residualText[16];
    float slope = 0.0F;
    float residual = 0.0F;
    uint32_t fitted = l3_track_fit(track, 8U, &slope, &residual);

    l3_text_fixed2(track->lastBin, binText, sizeof(binText));
    l3_text_fixed2((float)destBin - track->lastBin, distText, sizeof(distText));
    l3_text_fixed2(track->velocityBinsPerFrame, velocityText, sizeof(velocityText));
    l3_text_fixed2(((slope < 0.0F) ? -slope : slope) * track->cfg.binWidthM, speedText,
                  sizeof(speedText));
    l3_text_fixed2(residual, residualText, sizeof(residualText));
    return snprintf(out, cap,
                    "clubtrack active=%u why=%s count=%u total=%u misses=%u bin=%s dest=%u "
                    "dist=%s vel=%s speed=%s fit=%u residual=%s acq=%u assoc=%u coast=%u "
                    "drop=%u",
                    (unsigned)track->active, l3_track_why_name(track->why),
                    (unsigned)track->count, (unsigned)track->total, (unsigned)track->misses,
                    binText, (unsigned)destBin, distText, velocityText, speedText,
                    (unsigned)fitted, residualText,
                    (unsigned)track->counters[L3_TRACK_WHY_ACQUIRED],
                    (unsigned)track->counters[L3_TRACK_WHY_ASSOCIATED],
                    (unsigned)track->counters[L3_TRACK_WHY_COASTED],
                    (unsigned)track->counters[L3_TRACK_WHY_DROPPED]);
}

int32_t l3_track_format_point(const l3_track_point_t *point, uint32_t destBin, char *out,
                              uint32_t cap)
{
    char binText[16];
    char distText[16];
    char rangeText[16];
    char velocityText[16];
    char dopplerText[16];
    char confidenceText[16];
    char azText[16];
    char elText[16];
    char angles[40];

    l3_text_fixed2(point->rangeBin, binText, sizeof(binText));
    l3_text_fixed2((float)destBin - point->rangeBin, distText, sizeof(distText));
    l3_text_fixed2(point->rangeM, rangeText, sizeof(rangeText));
    l3_text_fixed2(point->radialVelocityMps, velocityText, sizeof(velocityText));
    l3_text_fixed2(point->dopplerAliasMps, dopplerText, sizeof(dopplerText));
    l3_text_fixed2(point->confidence, confidenceText, sizeof(confidenceText));
    if (point->anglesValid == 0U) {
        (void)snprintf(angles, sizeof(angles), "angles=none");
    } else {
        l3_text_degrees2(point->azimuthRad, azText, sizeof(azText));
        l3_text_degrees2(point->elevationRad, elText, sizeof(elText));
        (void)snprintf(angles, sizeof(angles), "az=%s el=%s",
                       (point->anglesValid & L3_OBS_ANGLE_AZIMUTH) ? azText : "-",
                       (point->anglesValid & L3_OBS_ANGLE_ELEVATION) ? elText : "-");
    }
    return snprintf(out, cap,
                    "p frame=%u t=%u bin=%s dist=%s range=%s vr=%s vd=%s coh=%u conf=%s %s",
                    (unsigned)point->frame, (unsigned)point->timestampUs, binText, distText,
                    rangeText, velocityText, dopplerText,
                    (unsigned)(point->coherence * 100.0F + 0.5F), confidenceText, angles);
}

int32_t l3_track_format_delivery(const l3_delivery_t *delivery, char *out, uint32_t cap)
{
    char speedText[16];
    char radialText[16];
    char pathText[16];
    char attackText[16];
    char residualText[16];
    char confidenceText[16];
    char valid[4];
    uint32_t v = 0U;

    l3_text_fixed2(delivery->speedMps, speedText, sizeof(speedText));
    l3_text_fixed2(delivery->radialSpeedMps, radialText, sizeof(radialText));
    l3_text_degrees2(delivery->pathRad, pathText, sizeof(pathText));
    l3_text_degrees2(delivery->attackRad, attackText, sizeof(attackText));
    l3_text_fixed2(delivery->residualM * 1000.0F, residualText, sizeof(residualText));
    l3_text_fixed2(delivery->confidence, confidenceText, sizeof(confidenceText));
    if (delivery->speedValid) {
        valid[v++] = 's';
    }
    if (delivery->pathValid) {
        valid[v++] = 'p';
    }
    if (delivery->attackValid) {
        valid[v++] = 'a';
    }
    valid[v] = '\0';
    return snprintf(out, cap,
                    "delivery points=%u az=%u el=%u speed=%s radial=%s path=%s attack=%s "
                    "residualmm=%s conf=%s valid=%s",
                    (unsigned)delivery->points, (unsigned)delivery->azimuthPoints,
                    (unsigned)delivery->elevationPoints, speedText, radialText, pathText,
                    attackText, residualText, confidenceText, (v > 0U) ? valid : "none");
}
