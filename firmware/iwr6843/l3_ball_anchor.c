#include "l3_ball_anchor.h"

#include <string.h>

void l3_ball_anchor_make(float anchorBin, float acceptFromBin, uint32_t gateUs,
                         uint32_t gateTolUs, const l3_impact_fit_cfg_t *fitCfg,
                         const l3_club_track_t *club, float maxSigmaUs, l3_ball_anchor_t *out)
{
    l3_fit_span_t span;
    l3_fit_estimate_t est;
    uint32_t anchorUs;
    float tol;

    memset(out, 0, sizeof(*out));
    out->anchorBin = anchorBin;
    out->acceptFromBin = acceptFromBin;
    out->gateUs = gateUs;
    out->anchorUs = gateUs;
    out->anchorTolUs = gateTolUs;
    out->source = L3_BALL_ANCHOR_GATE;
    if (fitCfg == NULL || club == NULL || club->count == 0U || !(maxSigmaUs > 0.0F)) {
        return;
    }
    span.track = club;
    span.first = 0U;
    span.count = club->count;
    l3_impact_fit_track(fitCfg, L3_FIT_CLUB_IN, l3_fit_span_point, &span, club->count,
                        anchorBin * fitCfg->binWidthM, &est);
    if (est.why != L3_FIT_WHY_OK || !(est.sigmaUs <= maxSigmaUs)) {
        return;
    }
    anchorUs = l3_round_us(est.timeUs);
    if (anchorUs == 0U) {
        return;  /* not finite or not positive: keep the gate */
    }
    tol = 3.0F * est.sigmaUs;
    if (tol < (float)L3_BALL_ANCHOR_MIN_TOL_US) {
        tol = (float)L3_BALL_ANCHOR_MIN_TOL_US;
    }
    out->anchorUs = anchorUs;
    out->anchorTolUs = (uint32_t)(tol + 0.5F);
    out->anchorSigmaUs = est.sigmaUs;
    out->source = L3_BALL_ANCHOR_CLUB;
}

const char *l3_ball_anchor_source_name(uint8_t source)
{
    return (source == L3_BALL_ANCHOR_CLUB) ? "club" : "gate";
}

uint32_t l3_ball_anchor_struct_bytes(void)
{
    return (uint32_t)sizeof(l3_ball_anchor_t);
}
