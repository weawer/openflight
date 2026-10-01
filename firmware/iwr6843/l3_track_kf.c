/* IWR6843 club trajectory reconstruction. See l3_track_kf.h. */
#include <math.h>
#include <string.h>

#include "l3_track_kf.h"

#define N L3_TRACK_KF_STATES
#define L3_KF_PI 3.14159265F
#define L3_KF_MIN_RANGE_M 1.0e-3F
#define L3_KF_ANGLES (L3_OBS_ANGLE_AZIMUTH | L3_OBS_ANGLE_ELEVATION)

static const char *const kWhyNames[L3_TRACK_KF_WHY_COUNT] = {
    "none", "ok", "few_points", "diverged"
};

void l3_track_kf_cfg_defaults(l3_track_kf_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->accelSigmaMps2 = 1500.0F;   /* a clubhead on its arc: ~40 m/s at ~1.1 m radius */
    cfg->rangeSigmaM = 0.03F;
    cfg->angleSigmaRad = 15.0F * (L3_KF_PI / 180.0F);
    cfg->minAngleConfidence = 0.05F;
    cfg->chi2Gate = 9.21F;           /* 99 % for 2 degrees of freedom */
    cfg->initPositionSigmaM = 0.5F;
    cfg->initVelocitySigmaMps = 50.0F;
}

uint32_t l3_track_kf_work_bytes(void)
{
    return (uint32_t)sizeof(l3_track_kf_work_t);
}

const char *l3_track_kf_why_name(uint8_t why)
{
    return (why < L3_TRACK_KF_WHY_COUNT) ? kWhyNames[why] : "?";
}

static int32_t l3_kf_finite(float v)
{
    return (v == v) && v < 1.0e30F && v > -1.0e30F;
}

static float l3_kf_wrap(float a)
{
    while (a > L3_KF_PI) {
        a -= 2.0F * L3_KF_PI;
    }
    while (a < -L3_KF_PI) {
        a += 2.0F * L3_KF_PI;
    }
    return a;
}

static void l3_kf_symmetrize(float P[N][N])
{
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < N; i++) {
        for (j = i + 1U; j < N; j++) {
            float mean = 0.5F * (P[i][j] + P[j][i]);

            P[i][j] = mean;
            P[j][i] = mean;
        }
    }
}

/* Range, azimuth and elevation of a state's position, with their rows of H
 * (position columns; velocity columns 0). 0 at the origin or on the z axis. */
static int32_t l3_kf_measure(const float x[N], float z[3], float H[3][N])
{
    float rhoSq = x[0] * x[0] + x[1] * x[1];
    float rSq = rhoSq + x[2] * x[2];
    float rho = sqrtf(rhoSq);
    float r = sqrtf(rSq);

    memset(H, 0, 3U * N * sizeof(float));
    if (r < L3_KF_MIN_RANGE_M || rho < L3_KF_MIN_RANGE_M) {
        return 0;
    }
    z[0] = r;
    z[1] = atan2f(x[1], x[0]);
    z[2] = atan2f(x[2], rho);
    H[0][0] = x[0] / r;
    H[0][1] = x[1] / r;
    H[0][2] = x[2] / r;
    H[1][0] = -x[1] / rhoSq;
    H[1][1] = x[0] / rhoSq;
    H[2][0] = -x[0] * x[2] / (rSq * rho);
    H[2][1] = -x[1] * x[2] / (rSq * rho);
    H[2][2] = rho / rSq;
    return 1;
}

/* x, P forward by dt: F = [I dt*I; 0 I], white-acceleration Q per axis.
 * The 2D arrays stay unqualified: TI armcl rejects float (*)[N] passed as
 * const float (*)[N]. */
static void l3_kf_predict(const l3_track_kf_cfg_t *cfg, float dt, const float x[N],
                          float P[N][N], float xo[N], float Po[N][N])
{
    float q = cfg->accelSigmaMps2 * cfg->accelSigmaMps2;
    float dt2 = dt * dt;
    float FP[N][N];
    uint32_t a;
    uint32_t i;

    for (a = 0U; a < 3U; a++) {
        xo[a] = x[a] + dt * x[a + 3U];
        xo[a + 3U] = x[a + 3U];
    }
    for (i = 0U; i < N; i++) {
        for (a = 0U; a < 3U; a++) {
            FP[a][i] = P[a][i] + dt * P[a + 3U][i];
            FP[a + 3U][i] = P[a + 3U][i];
        }
    }
    for (i = 0U; i < N; i++) {
        for (a = 0U; a < 3U; a++) {
            Po[i][a] = FP[i][a] + dt * FP[i][a + 3U];
            Po[i][a + 3U] = FP[i][a + 3U];
        }
    }
    for (a = 0U; a < 3U; a++) {
        Po[a][a] += 0.25F * dt2 * dt2 * q;
        Po[a][a + 3U] += 0.5F * dt2 * dt * q;
        Po[a + 3U][a] += 0.5F * dt2 * dt * q;
        Po[a + 3U][a + 3U] += dt2 * q;
    }
}

/* One scalar measurement: innovation innov, row H, variance R. 0 when the
 * innovation variance is not positive (divergence). */
static int32_t l3_kf_update1(float x[N], float P[N][N], const float H[N], float innov, float R)
{
    float PH[N];
    float S = R;
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < N; i++) {
        PH[i] = 0.0F;
        for (j = 0U; j < N; j++) {
            PH[i] += P[i][j] * H[j];
        }
        S += H[i] * PH[i];
    }
    if (!(S > 0.0F)) {
        return 0;
    }
    for (i = 0U; i < N; i++) {
        x[i] += PH[i] / S * innov;
    }
    for (i = 0U; i < N; i++) {
        for (j = 0U; j < N; j++) {
            P[i][j] -= PH[i] * PH[j] / S;
        }
    }
    l3_kf_symmetrize(P);
    return 1;
}

/* The angle pair's chi-square nu' S^-1 nu, S = H P H' + var I (2x2). -1 when
 * S is singular. */
static float l3_kf_angle_chi2(float P[N][N], float H[3][N], const float nu[2],
                              float var)
{
    float PH1[N];
    float PH2[N];
    float s11 = var;
    float s22 = var;
    float s12 = 0.0F;
    float det;
    uint32_t i;
    uint32_t j;

    for (i = 0U; i < N; i++) {
        PH1[i] = 0.0F;
        PH2[i] = 0.0F;
        for (j = 0U; j < N; j++) {
            PH1[i] += P[i][j] * H[1][j];
            PH2[i] += P[i][j] * H[2][j];
        }
    }
    for (i = 0U; i < N; i++) {
        s11 += H[1][i] * PH1[i];
        s22 += H[2][i] * PH2[i];
        s12 += H[1][i] * PH2[i];
    }
    det = s11 * s22 - s12 * s12;
    if (!(det > 0.0F)) {
        return -1.0F;
    }
    return (s22 * nu[0] * nu[0] - 2.0F * s12 * nu[0] * nu[1] + s11 * nu[1] * nu[1]) / det;
}

/* A y = b for symmetric positive-definite A by Cholesky; 0 when A is not. */
static int32_t l3_kf_solve(float A[N][N], const float b[N], float y[N])
{
    float L[N][N];
    float t[N];
    int32_t i;
    int32_t j;
    int32_t k;

    memset(L, 0, sizeof(L));
    for (i = 0; i < (int32_t)N; i++) {
        for (j = 0; j <= i; j++) {
            float sum = A[i][j];

            for (k = 0; k < j; k++) {
                sum -= L[i][k] * L[j][k];
            }
            if (i == j) {
                if (!(sum > 0.0F)) {
                    return 0;
                }
                L[i][i] = sqrtf(sum);
            } else {
                L[i][j] = sum / L[j][j];
            }
        }
    }
    for (i = 0; i < (int32_t)N; i++) {
        float sum = b[i];

        for (k = 0; k < i; k++) {
            sum -= L[i][k] * t[k];
        }
        t[i] = sum / L[i][i];
    }
    for (i = (int32_t)N - 1; i >= 0; i--) {
        float sum = t[i];

        for (k = i + 1; k < (int32_t)N; k++) {
            sum -= L[k][i] * y[k];
        }
        y[i] = sum / L[i][i];
    }
    return 1;
}

static uint32_t l3_kf_fail(l3_club_track_t *track, l3_track_kf_result_t *out, uint8_t why)
{
    l3_track_unfilter_all(track);
    out->accepted = 0U;
    out->why = why;
    return 0U;
}

/* The point's range, then (if it has both angles and they pass the gate) its
 * azimuth and elevation, each a scalar update relinearised at the latest
 * state. Returns -1 on divergence, else whether the angles were used. */
static int32_t l3_kf_update(const l3_track_kf_cfg_t *cfg, const l3_track_point_t *point,
                            float x[N], float P[N][N])
{
    float measured[N] = {0};
    float zm[3];
    float z[3];
    float H[3][N];
    float Hm[3][N];
    float nu[2];
    float confidence;
    float var;
    float chi2;

    measured[0] = point->position.x;
    measured[1] = point->position.y;
    measured[2] = point->position.z;
    if (!l3_kf_measure(measured, zm, Hm) || !l3_kf_measure(x, z, H)) {
        return -1;
    }
    if (!l3_kf_update1(x, P, H[0], zm[0] - z[0], cfg->rangeSigmaM * cfg->rangeSigmaM)) {
        return -1;
    }
    if ((point->anglesValid & L3_KF_ANGLES) != L3_KF_ANGLES || !(point->angleConfidence > 0.0F)) {
        return 0;
    }
    if (!l3_kf_measure(x, z, H)) {
        return -1;
    }
    confidence = (point->angleConfidence > cfg->minAngleConfidence) ? point->angleConfidence
                                                                    : cfg->minAngleConfidence;
    var = (cfg->angleSigmaRad / confidence) * (cfg->angleSigmaRad / confidence);
    nu[0] = l3_kf_wrap(zm[1] - z[1]);
    nu[1] = zm[2] - z[2];
    chi2 = l3_kf_angle_chi2(P, H, nu, var);
    if (chi2 < 0.0F) {
        return -1;
    }
    if (chi2 > cfg->chi2Gate) {
        return 0;
    }
    if (!l3_kf_update1(x, P, H[1], nu[0], var) || !l3_kf_measure(x, z, H) ||
        !l3_kf_update1(x, P, H[2], zm[2] - z[2], var)) {
        return -1;
    }
    return 1;
}

uint32_t l3_track_kf_run(const l3_track_kf_cfg_t *cfg, l3_club_track_t *track,
                         l3_track_kf_work_t *work, l3_track_kf_result_t *out)
{
    uint32_t n = track->count;
    uint32_t k;
    uint32_t i;

    memset(out, 0, sizeof(*out));
    out->points = n;
    if (n < 3U) {
        return l3_kf_fail(track, out, L3_TRACK_KF_WHY_FEW_POINTS);
    }
    for (k = 0U; k < n; k++) {
        const l3_track_point_t *point = l3_track_point_mut(track, k);

        if (k == 0U) {
            /* Seed from the first point: its position (whatever angles it has)
             * with a wide uncertainty, at rest. */
            float sp = cfg->initPositionSigmaM * cfg->initPositionSigmaM;
            float sv = cfg->initVelocitySigmaMps * cfg->initVelocitySigmaMps;

            memset(work->x[0], 0, sizeof(work->x[0]));
            memset(work->P[0], 0, sizeof(work->P[0]));
            work->x[0][0] = point->position.x;
            work->x[0][1] = point->position.y;
            work->x[0][2] = point->position.z;
            for (i = 0U; i < 3U; i++) {
                work->P[0][i][i] = sp;
                work->P[0][i + 3U][i + 3U] = sv;
            }
            memcpy(work->xp[0], work->x[0], sizeof(work->x[0]));
            memcpy(work->Pp[0], work->P[0], sizeof(work->P[0]));
            work->dt[0] = 0.0F;
            work->accepted[0] = 0U;
        } else {
            const l3_track_point_t *before = l3_track_point_mut(track, k - 1U);
            /* int32 difference: a wrap of the microsecond clock is one step. */
            float dt = (float)(int32_t)(point->timestampUs - before->timestampUs) * 1.0e-6F;
            int32_t used;

            work->dt[k] = (dt > 0.0F) ? dt : 0.0F;
            l3_kf_predict(cfg, work->dt[k], work->x[k - 1U], work->P[k - 1U], work->xp[k],
                          work->Pp[k]);
            memcpy(work->x[k], work->xp[k], sizeof(work->x[k]));
            memcpy(work->P[k], work->Pp[k], sizeof(work->P[k]));
            used = l3_kf_update(cfg, point, work->x[k], work->P[k]);
            if (used < 0) {
                return l3_kf_fail(track, out, L3_TRACK_KF_WHY_DIVERGED);
            }
            work->accepted[k] = (uint8_t)used;
            out->accepted += (uint32_t)used;
        }
    }
    /* RTS over the full state (only the smoothed covariance is skipped): x_k += P_k F' Pp_{k+1}^-1 (xs_{k+1} - xp_{k+1}). */
    for (k = n - 1U; k-- > 0U;) {
        float d[N];
        float y[N];
        float g[N];

        for (i = 0U; i < N; i++) {
            d[i] = work->x[k + 1U][i] - work->xp[k + 1U][i];
        }
        if (!l3_kf_solve(work->Pp[k + 1U], d, y)) {
            return l3_kf_fail(track, out, L3_TRACK_KF_WHY_DIVERGED);
        }
        for (i = 0U; i < 3U; i++) {
            g[i] = y[i];
            g[i + 3U] = work->dt[k + 1U] * y[i] + y[i + 3U];
        }
        for (i = 0U; i < N; i++) {
            uint32_t j;

            for (j = 0U; j < N; j++) {
                work->x[k][i] += work->P[k][i][j] * g[j];
            }
        }
    }
    for (k = 0U; k < n; k++) {
        if (!l3_kf_finite(work->x[k][0]) || !l3_kf_finite(work->x[k][1]) ||
            !l3_kf_finite(work->x[k][2])) {
            return l3_kf_fail(track, out, L3_TRACK_KF_WHY_DIVERGED);
        }
    }
    for (k = 0U; k < n; k++) {
        l3_track_point_t *point = l3_track_point_mut(track, k);

        point->filteredPosition.x = work->x[k][0];
        point->filteredPosition.y = work->x[k][1];
        point->filteredPosition.z = work->x[k][2];
        point->filterAccepted = work->accepted[k];
        point->filterHypothesis = work->accepted[k] ? L3_FILTER_HYP_DIRECT : L3_FILTER_HYP_NONE;
    }
    out->why = L3_TRACK_KF_WHY_OK;
    return out->accepted;
}

static int32_t l3_track_filtered_at(const void *ctx, uint32_t index, l3_track_point_t *out)
{
    if (!l3_track_point((const l3_club_track_t *)ctx, index, out)) {
        return 0;
    }
    if (out->filterHypothesis != L3_FILTER_HYP_UNFILTERED) {
        out->position = out->filteredPosition;
    }
    return 1;
}

uint32_t l3_track_delivery_filtered(const l3_club_track_t *track, uint32_t maxPoints,
                                    l3_delivery_t *out)
{
    uint32_t first = l3_track_newest_first(track, maxPoints);

    memset(out, 0, sizeof(*out));
    return l3_delivery_fit(l3_track_filtered_at, track, first, track->count, L3_TRACK_FULL_POINTS,
                           track->cfg.binWidthM, track->cfg.maxAngleResidualM, out);
}
