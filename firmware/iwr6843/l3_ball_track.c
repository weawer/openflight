/* See l3_ball_track.h. */
#include <stdio.h>
#include <string.h>

#include "l3_ball_track.h"
#include "l3_text.h"

static const char *const kWhyNames[L3_BALL_TRACK_WHY_COUNT] = {
    "none", "unarmed", "nocandidate", "acquired", "confirmed", "tooslow", "toofast", "tracked",
    "coasted", "lost"
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
    cfg->minDepartureMps = 10.0F;     /* the slowest chip leaves faster than this */
    cfg->maxSpeedMps = 100.0F;
    cfg->originGateBins = 8.0F;       /* the first post frame is at most ~5 bins out */
    cfg->minDepartureBins = 1.0F;     /* the impact echo sits at the origin itself */
    cfg->launchPoints = 6U;
    cfg->snr = 3.0F;                  /* half the trigger's: the ball is weak and moving */
}

void l3_ball_track_init(l3_ball_track_t *track, const l3_ball_track_cfg_t *cfg)
{
    memset(track, 0, sizeof(*track));
    track->cfg = *cfg;
    track->lastTargetIndex = L3_TRACK_NO_TARGET;
    l3_track_init(&track->core, &cfg->core);
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
}

void l3_ball_track_arm(l3_ball_track_t *track, float originBin, const l3_vec3_t *origin,
                       uint32_t impactTimestampUs)
{
    l3_ball_track_reset(track);
    track->armed = 1U;
    track->originBin = originBin;
    track->origin = *origin;
    track->impactTimestampUs = impactTimestampUs;
}

static int32_t l3_ball_track_note(l3_ball_track_t *track, uint8_t why, int32_t appended)
{
    track->why = why;
    track->counters[why]++;
    return appended;
}

int32_t l3_ball_track_update(l3_ball_track_t *track, const l3_target_obs_t *targets, uint32_t n,
                             uint32_t frame, uint32_t timestampUs)
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
     * acquisition only the farthest candidate inside the origin gate is
     * offered, since the ball outruns the club's follow-through at once. */
    if (track->core.active && track->confirmed) {
        for (i = 0U; i < n && kept < L3_OBS_MAX_TARGETS; i++) {
            if (targets[i].rangeBin < track->core.lastBin - 0.5F) {
                continue;
            }
            indices[kept] = i;
            candidates[kept++] = targets[i];
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

int32_t l3_ball_track_set_angles(l3_ball_track_t *track, float azimuthRad, float elevationRad,
                                 uint8_t anglesValid)
{
    return l3_track_set_angles(&track->core, azimuthRad, elevationRad, anglesValid);
}

uint32_t l3_ball_track_launch(const l3_ball_track_t *track, l3_launch_t *out)
{
    l3_delivery_t fit;
    uint32_t used;
    float dtS;

    memset(out, 0, sizeof(*out));
    if (!track->confirmed) {
        return 0U;
    }
    used = l3_track_delivery_range(&track->core, 0U, track->cfg.launchPoints,
                                   track->cfg.launchPoints, &fit);
    if (used == 0U) {
        return 0U;
    }
    out->points = used;
    out->velocity = fit.velocity;
    /* The fitted line is anchored at its last point; walk it back to impact. */
    dtS = ((float)track->impactTimestampUs - (float)fit.timestampUs) * 1.0e-6F;
    out->launchPosition.x = fit.position.x + fit.velocity.x * dtS;
    out->launchPosition.y = fit.position.y + fit.velocity.y * dtS;
    out->launchPosition.z = fit.position.z + fit.velocity.z * dtS;
    out->speedMps = fit.speedMps;
    out->radialSpeedMps = fit.radialSpeedMps;
    out->residualM = fit.residualM;
    out->confidence = fit.confidence;
    out->speedValid = fit.speedValid;
    if (fit.pathValid) {
        out->hlaRad = fit.pathRad;
        out->hlaValid = 1U;
    }
    if (fit.attackValid) {
        out->vlaRad = fit.attackRad;
        out->vlaValid = 1U;
    }
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

int32_t l3_launch_format(const l3_launch_t *launch, char *out, uint32_t cap)
{
    char speedText[16];
    char radialText[16];
    char hlaText[16];
    char vlaText[16];
    char residualText[16];
    char confidenceText[16];
    char valid[4];
    uint32_t v = 0U;

    l3_text_fixed2(launch->speedMps, speedText, sizeof(speedText));
    l3_text_fixed2(launch->radialSpeedMps, radialText, sizeof(radialText));
    l3_text_degrees2(launch->hlaRad, hlaText, sizeof(hlaText));
    l3_text_degrees2(launch->vlaRad, vlaText, sizeof(vlaText));
    l3_text_fixed2(launch->residualM * 1000.0F, residualText, sizeof(residualText));
    l3_text_fixed2(launch->confidence, confidenceText, sizeof(confidenceText));
    if (launch->speedValid) {
        valid[v++] = 's';
    }
    if (launch->hlaValid) {
        valid[v++] = 'h';
    }
    if (launch->vlaValid) {
        valid[v++] = 'v';
    }
    valid[v] = '\0';
    return snprintf(out, cap,
                    "launch points=%u speed=%s radial=%s hla=%s vla=%s residualmm=%s conf=%s "
                    "valid=%s",
                    (unsigned)launch->points, speedText, radialText, hlaText, vlaText,
                    residualText, confidenceText, (v > 0U) ? valid : "none");
}
