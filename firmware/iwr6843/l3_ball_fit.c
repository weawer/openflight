/* IWR6843 ball direction fit. See l3_ball_fit.h. */
#include <math.h>
#include <string.h>

#include "l3_ball_fit.h"

#define L3_BALL_FIT_DEG (3.14159265F / 180.0F)
#define L3_BALL_FIT_MAX_STEPS 16U
#define L3_BALL_FIT_MAX_LEVELS 4U
#define L3_BALL_FIT_MIN_ANGLES 3U
#define L3_BALL_FIT_CURV_STEP (0.5F * L3_BALL_FIT_DEG)
#define L3_BALL_FIT_CURV_EVALS 9U
#define L3_BALL_FIT_EDGE_RAD 1.0e-4F
#define L3_BALL_FIT_ANGLES (L3_OBS_ANGLE_AZIMUTH | L3_OBS_ANGLE_ELEVATION)

static const char *const kWhyNames[L3_BALL_FIT_WHY_COUNT] = {
    "none", "ok", "few_angles", "scatter", "grid_edge", "no_tee", "uncertain"
};

/* One held point, as the fit reads it. */
typedef struct {
    l3_vec3_t m;      /* measured direction (unit), golf frame */
    float     r;      /* measured range, bias removed */
    float     w;      /* the angle confidence; 0 leaves the point out */
    uint8_t   ahead;  /* beyond the tee: the line reaches its range going forward */
    uint8_t   use;    /* weighed in the current pass */
} l3_ball_fit_obs_t;

/* The line at one point's range, and the measured direction's squared angle
 * (small-angle, 2 (1 - cos)) from the direct return and from the reflection. */
typedef struct {
    l3_vec3_t p;
    l3_vec3_t q;        /* p mirrored about the floor */
    float     qNorm;
    float     directSq;
    float     imageSq;
} l3_ball_fit_pred_t;

static float l3_ball_fit_dot(const l3_vec3_t *a, const l3_vec3_t *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

static uint32_t l3_ball_fit_steps(const l3_ball_fit_cfg_t *cfg)
{
    if (cfg->gridSteps < 1U) {
        return 1U;
    }
    return (cfg->gridSteps > L3_BALL_FIT_MAX_STEPS) ? L3_BALL_FIT_MAX_STEPS : cfg->gridSteps;
}

static uint32_t l3_ball_fit_levels(const l3_ball_fit_cfg_t *cfg)
{
    if (cfg->gridLevels < 1U) {
        return 1U;
    }
    return (cfg->gridLevels > L3_BALL_FIT_MAX_LEVELS) ? L3_BALL_FIT_MAX_LEVELS : cfg->gridLevels;
}

void l3_ball_fit_cfg_defaults(l3_ball_fit_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->angleSigmaRad = 12.0F * L3_BALL_FIT_DEG;  /* elevation SD, 2026-09-29 */
    cfg->gateK = 2.5F;
    cfg->huberK = 1.5F;
    cfg->minAccepted = 4U;
    cfg->maxRmsRad = 15.0F * L3_BALL_FIT_DEG;
    cfg->imageSepMinRad = 2.0F * L3_BALL_FIT_DEG;
    cfg->radarHeightM = 0.152F;                    /* calibration.py default */
    cfg->teeBallHeightM = 0.04F;                   /* a ball on a mat */
    cfg->hlaMinRad = -45.0F * L3_BALL_FIT_DEG;
    cfg->hlaMaxRad = 45.0F * L3_BALL_FIT_DEG;
    cfg->vlaMinRad = -10.0F * L3_BALL_FIT_DEG;
    cfg->vlaMaxRad = 60.0F * L3_BALL_FIT_DEG;
    cfg->gridSteps = 10U;
    cfg->gridLevels = 3U;
    cfg->maxAngleSigmaRad = 3.0F * L3_BALL_FIT_DEG;
}

uint32_t l3_ball_fit_max_evaluations(const l3_ball_fit_cfg_t *cfg)
{
    uint32_t side = l3_ball_fit_steps(cfg) + 1U;

    return 2U * l3_ball_fit_levels(cfg) * side * side + L3_BALL_FIT_CURV_EVALS;
}

void l3_ball_fit_direction(float hlaRad, float vlaRad, l3_vec3_t *u)
{
    float cv = cosf(vlaRad);

    u->x = cv * cosf(hlaRad);
    u->y = cv * sinf(hlaRad);
    u->z = sinf(vlaRad);
}

const char *l3_ball_fit_why_name(uint8_t why)
{
    return (why < L3_BALL_FIT_WHY_COUNT) ? kWhyNames[why] : "?";
}

/* The tee: the origin's slant range and bearing, at the ball's height. */
static int32_t l3_ball_fit_tee(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *origin,
                               l3_vec3_t *tee)
{
    float range = sqrtf(l3_ball_fit_dot(origin, origin));
    float height = cfg->teeBallHeightM - cfg->radarHeightM;
    float ground;
    float bearing;

    if (!(range > ((height < 0.0F) ? -height : height))) {
        return 0;
    }
    ground = sqrtf(range * range - height * height);
    bearing = atan2f(origin->y, origin->x);
    tee->x = ground * cosf(bearing);
    tee->y = ground * sinf(bearing);
    tee->z = height;
    return 1;
}

static void l3_ball_fit_predict(float mirrorZ, const l3_vec3_t *tee, const l3_vec3_t *u,
                                float teeU, float teeSq, const l3_ball_fit_obs_t *o,
                                l3_ball_fit_pred_t *out)
{
    /* Forward root of s^2 + 2 s (tee.u) + |tee|^2 - r^2 = 0; o->r > |tee|
     * (ahead), so the root is real and non-negative. */
    float disc = teeU * teeU + o->r * o->r - teeSq;
    float s = -teeU + sqrtf((disc > 0.0F) ? disc : 0.0F);
    float qSq;

    out->p.x = tee->x + s * u->x;
    out->p.y = tee->y + s * u->y;
    out->p.z = tee->z + s * u->z;
    out->directSq = 2.0F * (1.0F - l3_ball_fit_dot(&out->p, &o->m) / o->r);
    out->q.x = out->p.x;
    out->q.y = out->p.y;
    out->q.z = mirrorZ - out->p.z;
    qSq = l3_ball_fit_dot(&out->q, &out->q);
    out->qNorm = (qSq > 0.0F) ? sqrtf(qSq) : 0.0F;
    out->imageSq = (out->qNorm > 0.0F)
                       ? 2.0F * (1.0F - l3_ball_fit_dot(&out->q, &o->m) / out->qNorm)
                       : 4.0F;
}

static float l3_ball_fit_nearer(const l3_ball_fit_pred_t *pred)
{
    return (pred->directSq <= pred->imageSq) ? pred->directSq : pred->imageSq;
}

/* Sum over the points in use of w * huber(angle / sigma). */
static float l3_ball_fit_cost(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *tee,
                              const l3_vec3_t *u, const l3_ball_fit_obs_t *obs, uint32_t n)
{
    float teeU = l3_ball_fit_dot(tee, u);
    float teeSq = l3_ball_fit_dot(tee, tee);
    float mirrorZ = -2.0F * cfg->radarHeightM;
    float invSigmaSq = 1.0F / (cfg->angleSigmaRad * cfg->angleSigmaRad);
    float kSq = cfg->huberK * cfg->huberK;
    float cost = 0.0F;
    l3_ball_fit_pred_t pred;
    uint32_t i;

    for (i = 0U; i < n; i++) {
        float zSq;

        if (!obs[i].use) {
            continue;
        }
        l3_ball_fit_predict(mirrorZ, tee, u, teeU, teeSq, &obs[i], &pred);
        zSq = l3_ball_fit_nearer(&pred) * invSigmaSq;
        cost += obs[i].w *
                ((zSq <= kSq) ? 0.5F * zSq : cfg->huberK * sqrtf(zSq) - 0.5F * kSq);
    }
    return cost;
}

/* Coarse to fine over (HLA, VLA): each level a (steps+1)^2 grid, the next
 * spanning one step either side of the best so far, inside the limits.
 * Returns the evaluations made. */
static uint32_t l3_ball_fit_search(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *tee,
                                   const l3_ball_fit_obs_t *obs, uint32_t n, float *hla,
                                   float *vla)
{
    uint32_t steps = l3_ball_fit_steps(cfg);
    float hLo = cfg->hlaMinRad;
    float hHi = cfg->hlaMaxRad;
    float vLo = cfg->vlaMinRad;
    float vHi = cfg->vlaMaxRad;
    float best = 1.0e30F;
    float bestH = 0.5F * (hLo + hHi);
    float bestV = 0.5F * (vLo + vHi);
    uint32_t evaluations = 0U;
    uint32_t level;

    for (level = 0U; level < l3_ball_fit_levels(cfg); level++) {
        float hStep = (hHi - hLo) / (float)steps;
        float vStep = (vHi - vLo) / (float)steps;
        float cosH[L3_BALL_FIT_MAX_STEPS + 1U];
        float sinH[L3_BALL_FIT_MAX_STEPS + 1U];
        float cosV[L3_BALL_FIT_MAX_STEPS + 1U];
        float sinV[L3_BALL_FIT_MAX_STEPS + 1U];
        uint32_t a;
        uint32_t b;

        for (a = 0U; a <= steps; a++) {
            cosH[a] = cosf(hLo + (float)a * hStep);
            sinH[a] = sinf(hLo + (float)a * hStep);
            cosV[a] = cosf(vLo + (float)a * vStep);
            sinV[a] = sinf(vLo + (float)a * vStep);
        }
        for (a = 0U; a <= steps; a++) {
            for (b = 0U; b <= steps; b++) {
                l3_vec3_t u;
                float cost;

                u.x = cosV[b] * cosH[a];
                u.y = cosV[b] * sinH[a];
                u.z = sinV[b];
                cost = l3_ball_fit_cost(cfg, tee, &u, obs, n);
                evaluations++;
                if (cost < best) {
                    best = cost;
                    bestH = hLo + (float)a * hStep;
                    bestV = vLo + (float)b * vStep;
                }
            }
        }
        hLo = (bestH - hStep > cfg->hlaMinRad) ? bestH - hStep : cfg->hlaMinRad;
        hHi = (bestH + hStep < cfg->hlaMaxRad) ? bestH + hStep : cfg->hlaMaxRad;
        vLo = (bestV - vStep > cfg->vlaMinRad) ? bestV - vStep : cfg->vlaMinRad;
        vHi = (bestV + vStep < cfg->vlaMaxRad) ? bestV + vStep : cfg->vlaMaxRad;
    }
    *hla = bestH;
    *vla = bestV;
    return evaluations;
}

/* The direction's 1-sigma from the cost's curvature: a central-difference
 * Hessian in (HLA, VLA), inverted, scaled by the observed scatter's variance
 * relative to the assumed sigma. Returns 0 (and infinite sigmas) when the curvature is not
 * positive definite. */
static uint32_t l3_ball_fit_sigmas(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *tee,
                                   const l3_ball_fit_obs_t *obs, uint32_t n, float rmsRad,
                                   uint32_t accepted, float hla, float vla, float *hlaSigma,
                                   float *vlaSigma)
{
    float d = L3_BALL_FIT_CURV_STEP;
    float f[3][3];
    float hhh;
    float hvv;
    float hhv;
    float det;
    float scale;
    int32_t i;
    int32_t j;

    *hlaSigma = 1.0e30F;
    *vlaSigma = 1.0e30F;
    for (i = -1; i <= 1; i++) {
        for (j = -1; j <= 1; j++) {
            l3_vec3_t u;

            l3_ball_fit_direction(hla + (float)i * d, vla + (float)j * d, &u);
            f[i + 1][j + 1] = l3_ball_fit_cost(cfg, tee, &u, obs, n);
        }
    }
    hhh = (f[2][1] - 2.0F * f[1][1] + f[0][1]) / (d * d);
    hvv = (f[1][2] - 2.0F * f[1][1] + f[1][0]) / (d * d);
    hhv = (f[2][2] - f[2][0] - f[0][2] + f[0][0]) / (4.0F * d * d);
    det = hhh * hvv - hhv * hhv;
    if (!(hhh > 0.0F) || !(hvv > 0.0F) || !(det > 0.0F)) {
        return 0U;
    }
    /* Two fitted parameters (hla, vla): the residual variance is rms^2 * a / (a - 2),
     * a the accepted angles (a >= 3, the minAccepted clamp). */
    scale = rmsRad * rmsRad * (float)accepted / ((float)accepted - 2.0F) /
            (cfg->angleSigmaRad * cfg->angleSigmaRad);
    *hlaSigma = sqrtf(scale * hvv / det);
    *vlaSigma = sqrtf(scale * hhh / det);
    return 1U;
}

static uint32_t l3_ball_fit_fail(l3_club_track_t *core, l3_ball_fit_t *out, uint8_t why)
{
    l3_track_unfilter_all(core);
    out->valid = 0U;
    out->why = why;
    return 0U;
}

uint32_t l3_ball_fit_run(const l3_ball_fit_cfg_t *cfg, const l3_vec3_t *origin,
                         l3_club_track_t *core, l3_ball_fit_t *out)
{
    l3_ball_fit_obs_t obs[L3_TRACK_POINTS];
    l3_ball_fit_pred_t pred;
    uint32_t n = core->count;
    float gateSq = cfg->gateK * cfg->angleSigmaRad * cfg->gateK * cfg->angleSigmaRad;
    float mirrorZ = -2.0F * cfg->radarHeightM;
    float sepMinSq = cfg->imageSepMinRad * cfg->imageSepMinRad;
    float teeRange;
    float teeU;
    float teeSq;
    float sumW = 0.0F;
    float sumWSq = 0.0F;
    l3_vec3_t u;
    uint32_t minAccepted = (cfg->minAccepted < L3_BALL_FIT_MIN_ANGLES) ? L3_BALL_FIT_MIN_ANGLES
                                                                       : cfg->minAccepted;
    uint32_t i;
    uint32_t curved;

    memset(out, 0, sizeof(*out));
    memset(obs, 0, sizeof(obs));
    l3_track_unfilter_all(core);
    if (!l3_ball_fit_tee(cfg, origin, &out->tee)) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_NO_TEE);
    }
    teeRange = sqrtf(l3_ball_fit_dot(&out->tee, &out->tee));
    for (i = 0U; i < n; i++) {
        const l3_track_point_t *point = l3_track_point_mut(core, i);
        float r = sqrtf(l3_ball_fit_dot(&point->position, &point->position));
        uint8_t both = (uint8_t)((point->anglesValid & L3_BALL_FIT_ANGLES) == L3_BALL_FIT_ANGLES);

        obs[i].r = r;
        obs[i].ahead = (uint8_t)(r > teeRange);
        if (!obs[i].ahead) {
            continue;  /* short of the tee: no forward root, stays unfiltered */
        }
        obs[i].m.x = point->position.x / r;
        obs[i].m.y = point->position.y / r;
        obs[i].m.z = point->position.z / r;
        obs[i].w = (both && point->angleConfidence > 0.0F) ? point->angleConfidence : 0.0F;
        obs[i].use = (uint8_t)(obs[i].w > 0.0F);
        out->used += obs[i].use;
    }
    if (out->used < minAccepted) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_FEW_ANGLES);
    }
    out->evaluations = l3_ball_fit_search(cfg, &out->tee, obs, n, &out->hlaRad, &out->vlaRad);
    /* The gate: an angle further than gateK sigma from the fit is not the
     * ball's direction, whatever made it. */
    l3_ball_fit_direction(out->hlaRad, out->vlaRad, &u);
    teeU = l3_ball_fit_dot(&out->tee, &u);
    teeSq = l3_ball_fit_dot(&out->tee, &out->tee);
    for (i = 0U; i < n; i++) {
        if (!obs[i].use) {
            continue;
        }
        l3_ball_fit_predict(mirrorZ, &out->tee, &u, teeU, teeSq, &obs[i], &pred);
        if (l3_ball_fit_nearer(&pred) > gateSq) {
            obs[i].use = 0U;
        } else {
            out->accepted++;
        }
    }
    if (out->accepted < minAccepted) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_FEW_ANGLES);
    }
    if (out->accepted < out->used) {
        /* Something was gated out: fit again without it. */
        out->evaluations += l3_ball_fit_search(cfg, &out->tee, obs, n, &out->hlaRad,
                                               &out->vlaRad);
        l3_ball_fit_direction(out->hlaRad, out->vlaRad, &u);
        teeU = l3_ball_fit_dot(&out->tee, &u);
    }
    for (i = 0U; i < n; i++) {
        l3_track_point_t *point = l3_track_point_mut(core, i);

        if (!obs[i].ahead) {
            continue;
        }
        l3_ball_fit_predict(mirrorZ, &out->tee, &u, teeU, teeSq, &obs[i], &pred);
        point->filteredPosition = pred.p;
        point->filterAccepted = obs[i].use;
        if (!obs[i].use) {
            point->filterHypothesis = L3_FILTER_HYP_NONE;
            continue;
        }
        sumW += obs[i].w;
        sumWSq += obs[i].w * l3_ball_fit_nearer(&pred);
        {
            float sepSq = (pred.qNorm > 0.0F)
                              ? 2.0F * (1.0F - l3_ball_fit_dot(&pred.p, &pred.q) /
                                                   (obs[i].r * pred.qNorm))
                              : 4.0F;

            if (sepSq < sepMinSq) {
                point->filterHypothesis = L3_FILTER_HYP_AMBIGUOUS;
            } else if (pred.directSq <= pred.imageSq) {
                point->filterHypothesis = L3_FILTER_HYP_DIRECT;
            } else {
                point->filterHypothesis = L3_FILTER_HYP_IMAGE;
            }
        }
    }
    out->rmsRad = sqrtf(sumWSq / sumW);
    curved = l3_ball_fit_sigmas(cfg, &out->tee, obs, n, out->rmsRad, out->accepted, out->hlaRad, out->vlaRad,
                                &out->hlaSigmaRad, &out->vlaSigmaRad);
    out->evaluations += L3_BALL_FIT_CURV_EVALS;
    if (out->hlaRad <= cfg->hlaMinRad + L3_BALL_FIT_EDGE_RAD ||
        out->hlaRad >= cfg->hlaMaxRad - L3_BALL_FIT_EDGE_RAD ||
        out->vlaRad <= cfg->vlaMinRad + L3_BALL_FIT_EDGE_RAD ||
        out->vlaRad >= cfg->vlaMaxRad - L3_BALL_FIT_EDGE_RAD) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_GRID_EDGE);
    }
    if (out->rmsRad > cfg->maxRmsRad) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_SCATTER);
    }
    if (!curved || out->hlaSigmaRad > cfg->maxAngleSigmaRad ||
        out->vlaSigmaRad > cfg->maxAngleSigmaRad) {
        return l3_ball_fit_fail(core, out, L3_BALL_FIT_WHY_UNCERTAIN);
    }
    out->valid = 1U;
    out->why = L3_BALL_FIT_WHY_OK;
    return out->accepted;
}
