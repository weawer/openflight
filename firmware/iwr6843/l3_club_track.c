/* IWR6843 club track. See l3_club_track.h. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "l3_club_track.h"
#include "l3_text.h"

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
    cfg->velocitySpanMps = 2.0F * L3_OBS_WAVELENGTH_M / (4.0F * 135.0e-6F);
    l3_cal_identity(&cfg->cal, L3_CAL_MAX_VIRTUAL);
    cfg->minAcquireDopplerMps = 1.0F;
    cfg->maxAngleResidualM = 2.0F * cfg->binWidthM;  /* two bins of scatter */
    cfg->ascendingOnly = 1U;
    cfg->maxSameBinPoints = 2U;
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
    track->followBinsPerFrame = 0.0F;
    track->releasedValid = 0U;
    track->releasedBin = 0.0F;
    track->releasedDopplerMps = 0.0F;
}

static void l3_track_note(l3_club_track_t *track, uint8_t why)
{
    track->why = why;
    track->counters[why]++;
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
    track->next = (track->next + 1U) % L3_TRACK_POINTS;
    if (track->count < L3_TRACK_POINTS) {
        track->count++;
    }
    track->total++;
}

/* Doppler continuity: the smaller way round the alias circle. */
static float l3_track_wrappedDiff(float a, float b, float span)
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

/* Acquire the most confident target that clears the bar, preferring one that
 * reads as moving (a stationary body in the lane is often the strongest
 * return) and skipping one that looks like the last released return. With
 * quietWhenIdle, finding nothing leaves the last "why" (a release) standing. */
static int32_t l3_track_acquire(l3_club_track_t *track, const l3_target_obs_t *targets,
                                uint32_t n, uint32_t frame, int32_t quietWhenIdle)
{
    const l3_track_cfg_t *cfg = &track->cfg;
    const l3_target_obs_t *best = NULL;
    uint8_t bestMoves = 0U;
    uint32_t i;

    for (i = 0U; i < n; i++) {
        uint8_t moves = (uint8_t)(cfg->minAcquireDopplerMps <= 0.0F ||
                                  l3_track_absf(targets[i].dopplerAliasMps) >=
                                      cfg->minAcquireDopplerMps);
        if (targets[i].confidence < cfg->minConfidence) {
            continue;
        }
        if (track->releasedValid &&
            l3_track_absf(targets[i].rangeBin - track->releasedBin) <= cfg->gateBins &&
            l3_track_wrappedDiff(targets[i].dopplerAliasMps, track->releasedDopplerMps,
                                 cfg->velocitySpanMps) <= L3_TRACK_RELEASE_DOPPLER_TOL_MPS) {
            continue;
        }
        if (best == NULL || (moves && !bestMoves) ||
            (moves == bestMoves && targets[i].confidence > best->confidence)) {
            best = &targets[i];
            bestMoves = moves;
            track->lastTargetIndex = i;
        }
    }
    if (best == NULL) {
        if (!quietWhenIdle) {
            l3_track_note(track, L3_TRACK_WHY_IDLE);
        }
        return 0;
    }
    track->active = 1U;
    track->misses = 0U;
    track->lastFrame = frame;
    track->lastBin = best->rangeBin;
    track->velocityBinsPerFrame = 0.0F;
    track->predictedBin = best->rangeBin;
    track->releasedValid = 0U;
    l3_track_append(track, best, 0.0F, 0.0F);
    l3_track_countBin(track, best->rangeBin, 1);
    l3_track_note(track, L3_TRACK_WHY_ACQUIRED);
    return 1;
}

/* One frame of association for an active track; coast, then drop, when
 * nothing qualifies. Approaching: the best-scoring target in the gate around
 * the prediction, never below the last point's rounded bin when
 * ascendingOnly. Following after impact: the strongest target from
 * L3_TRACK_FOLLOW_RETREAT_BINS behind the last point to where the club would
 * be at its impact speed, plus L3_TRACK_FOLLOW_LEAD_BINS -- the club is the
 * stronger of the two returns after impact and only slows from its impact
 * speed, so it is anywhere between; beyond, it would be moving faster than
 * it arrived, which only the ball does -- and never a third consecutive point
 * in one bin, which a stall beside the ball is and the club is not. */
static int32_t l3_track_associate(l3_club_track_t *track, const l3_target_obs_t *targets,
                                  uint32_t n, uint32_t frame, uint32_t timestampUs,
                                  int32_t following)
{
    const l3_track_cfg_t *cfg = &track->cfg;
    uint32_t elapsed = frame - track->lastFrame;
    float predicted = track->lastBin + track->velocityBinsPerFrame * (float)elapsed;
    int32_t floorBin = l3_track_roundBin(track->lastBin);
    const l3_target_obs_t *best = NULL;
    float bestScore = 0.0F;
    const l3_track_point_t *last =
        &track->points[(track->next + L3_TRACK_POINTS - 1U) % L3_TRACK_POINTS];
    uint32_t i;

    track->predictedBin = predicted;
    for (i = 0U; i < n; i++) {
        float rangeErr = l3_track_absf(targets[i].rangeBin - predicted);
        float velocityErr;
        float score;

        if (following) {
            float reach = track->lastBin + track->followBinsPerFrame * (float)elapsed +
                          L3_TRACK_FOLLOW_LEAD_BINS;

            if (targets[i].rangeBin < track->lastBin - L3_TRACK_FOLLOW_RETREAT_BINS ||
                targets[i].rangeBin > reach) {
                continue;
            }
            if (cfg->maxSameBinPoints > 0U &&
                l3_track_roundBin(targets[i].rangeBin) == track->sameBin &&
                track->sameBinCount >= cfg->maxSameBinPoints) {
                continue;
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
        if (cfg->ascendingOnly && l3_track_roundBin(targets[i].rangeBin) < floorBin) {
            continue;
        }
        velocityErr = l3_track_wrappedDiff(targets[i].dopplerAliasMps, last->dopplerAliasMps,
                                           cfg->velocitySpanMps) /
                      ((cfg->velocitySpanMps > 0.0F) ? cfg->velocitySpanMps : 1.0F);
        score = cfg->weightRange * rangeErr + cfg->weightVelocity * velocityErr +
                cfg->weightQuality * (1.0F - targets[i].confidence);
        if (best == NULL || score < bestScore) {
            best = &targets[i];
            bestScore = score;
            track->lastTargetIndex = i;
        }
    }
    if (best == NULL) {
        track->lastTargetIndex = L3_TRACK_NO_TARGET;
        track->misses++;
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

int32_t l3_track_update(l3_club_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                        uint32_t frame, uint32_t timestampUs)
{
    const l3_track_cfg_t *cfg = &track->cfg;

    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    if (track->active) {
        if (!l3_track_associate(track, targets, n, frame, timestampUs, 0)) {
            return 0;
        }
        if (cfg->maxSameBinPoints == 0U || track->sameBinCount <= cfg->maxSameBinPoints) {
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

int32_t l3_track_follow(l3_club_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                        uint32_t frame, uint32_t timestampUs)
{
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    if (!track->active) {
        return 0;
    }
    if (!track->following) {
        /* The first frame after impact: the club can only slow from here. */
        track->following = 1U;
        track->followBinsPerFrame =
            (track->velocityBinsPerFrame > 0.0F) ? track->velocityBinsPerFrame : 0.0F;
    }
    return l3_track_associate(track, targets, n, frame, timestampUs, 1);
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

int32_t l3_track_set_angles(l3_club_track_t *track, float azimuthRad, float elevationRad,
                            uint8_t anglesValid)
{
    l3_track_point_t *point;

    if (track->lastTargetIndex == L3_TRACK_NO_TARGET || track->count == 0U) {
        return 0;
    }
    point = &track->points[(track->next + L3_TRACK_POINTS - 1U) % L3_TRACK_POINTS];
    point->azimuthRad = azimuthRad;
    point->elevationRad = elevationRad;
    point->anglesValid = anglesValid;
    l3_track_locate(track, point);
    return 1;
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
    uint32_t used;

    if (maxPoints > L3_TRACK_POINTS) {
        maxPoints = L3_TRACK_POINTS;
    }
    used = (track->count < maxPoints) ? track->count : maxPoints;
    return l3_track_delivery_range(track, track->count - used, used, L3_TRACK_FULL_POINTS, out);
}

uint32_t l3_track_delivery_range(const l3_club_track_t *track, uint32_t first, uint32_t count,
                                 uint32_t fullPoints, l3_delivery_t *out)
{
    float t[L3_TRACK_POINTS];
    float x[L3_TRACK_POINTS];
    float y[L3_TRACK_POINTS];
    float z[L3_TRACK_POINTS];
    float r[L3_TRACK_POINTS];
    uint32_t last;
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
    if (first >= track->count) {
        return 0U;
    }
    last = first + count;
    if (last > track->count) {
        last = track->count;
    }
    if (last - first < 3U) {
        return 0U;
    }
    for (i = first; i < last; i++) {
        (void)l3_track_point(track, i, &point);
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
    (void)l3_track_point(track, last - 1U, &point);
    newestUs = (float)point.timestampUs;
    out->timestampUs = point.timestampUs;
    for (i = first; i < last; i++) {
        (void)l3_track_point(track, i, &point);
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
    if (required != 0U && track->cfg.maxAngleResidualM > 0.0F &&
        out->residualM > track->cfg.maxAngleResidualM) {
        /* The angled positions do not lie on a line: the angles are noise
         * (a fast ball crossing bins within a burst does this). Keep the
         * range walk, which is a measurement, and drop the direction. */
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
        float residualScore = 1.0F - out->residualM / (2.0F * track->cfg.binWidthM);
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
