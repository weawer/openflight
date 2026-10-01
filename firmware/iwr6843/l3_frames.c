/* See l3_frames.h. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "l3_frames.h"
#include "l3_text.h"

void l3_cal_identity(l3_radar_cal_t *cal, uint32_t virtualElements)
{
    uint32_t i;

    memset(cal, 0, sizeof(*cal));
    if (virtualElements > L3_CAL_MAX_VIRTUAL) {
        virtualElements = L3_CAL_MAX_VIRTUAL;
    }
    cal->virtualElements = virtualElements;
    for (i = 0U; i < L3_CAL_MAX_VIRTUAL; i++) {
        cal->correctionRe[i] = 1.0F;
        cal->correctionIm[i] = 0.0F;
    }
}

void l3_frames_from_spherical(const l3_spherical_t *in, l3_vec3_t *radar)
{
    float horizontal = in->rangeM * cosf(in->elevationRad);

    radar->x = horizontal * cosf(in->azimuthRad);
    radar->y = horizontal * sinf(in->azimuthRad);
    radar->z = in->rangeM * sinf(in->elevationRad);
}

void l3_frames_to_spherical(const l3_vec3_t *radar, l3_spherical_t *out)
{
    float horizontal = sqrtf(radar->x * radar->x + radar->y * radar->y);

    out->rangeM = sqrtf(horizontal * horizontal + radar->z * radar->z);
    out->azimuthRad = (horizontal > 0.0F || radar->y != 0.0F) ? atan2f(radar->y, radar->x) : 0.0F;
    out->elevationRad = (out->rangeM > 0.0F) ? atan2f(radar->z, horizontal) : 0.0F;
}

/* golf = Yaw(psi) . Pitch(theta) . Roll(phi) . radar. Each step undoes one
 * component of the enclosure's attitude: a radar rolled right-side-down sees
 * a level target on its right below its own horizon, a radar pitched nose-up
 * sees a level target ahead below boresight, a radar yawed right sees the
 * target line to its left. */
void l3_frames_radar_to_golf(const l3_radar_cal_t *cal, const l3_vec3_t *radar, l3_vec3_t *golf)
{
    float cr = cosf(cal->radarRollRad);
    float sr = sinf(cal->radarRollRad);
    float cp = cosf(cal->radarPitchRad);
    float sp = sinf(cal->radarPitchRad);
    float cy = cosf(cal->radarYawRad);
    float sy = sinf(cal->radarYawRad);
    l3_vec3_t rolled;
    l3_vec3_t pitched;

    rolled.x = radar->x;
    rolled.y = radar->y * cr - radar->z * sr;
    rolled.z = radar->y * sr + radar->z * cr;

    pitched.x = rolled.x * cp - rolled.z * sp;
    pitched.y = rolled.y;
    pitched.z = rolled.x * sp + rolled.z * cp;

    golf->x = pitched.x * cy - pitched.y * sy;
    golf->y = pitched.x * sy + pitched.y * cy;
    golf->z = pitched.z;
}

void l3_frames_golf_to_radar(const l3_radar_cal_t *cal, const l3_vec3_t *golf, l3_vec3_t *radar)
{
    float cr = cosf(cal->radarRollRad);
    float sr = sinf(cal->radarRollRad);
    float cp = cosf(cal->radarPitchRad);
    float sp = sinf(cal->radarPitchRad);
    float cy = cosf(cal->radarYawRad);
    float sy = sinf(cal->radarYawRad);
    l3_vec3_t pitched;
    l3_vec3_t rolled;

    pitched.x = golf->x * cy + golf->y * sy;
    pitched.y = -golf->x * sy + golf->y * cy;
    pitched.z = golf->z;

    rolled.x = pitched.x * cp + pitched.z * sp;
    rolled.y = pitched.y;
    rolled.z = -pitched.x * sp + pitched.z * cp;

    radar->x = rolled.x;
    radar->y = rolled.y * cr + rolled.z * sr;
    radar->z = -rolled.y * sr + rolled.z * cr;
}

void l3_frames_observe(const l3_radar_cal_t *cal, float rangeM, float azimuthRad,
                       float elevationRad, l3_vec3_t *golf)
{
    l3_spherical_t spherical;
    l3_vec3_t radar;

    spherical.rangeM = rangeM - cal->rangeBiasM;
    if (spherical.rangeM < 0.0F) {
        spherical.rangeM = 0.0F;
    }
    spherical.azimuthRad = azimuthRad;
    spherical.elevationRad = elevationRad;
    l3_frames_from_spherical(&spherical, &radar);
    l3_frames_radar_to_golf(cal, &radar, golf);
}

float l3_frames_horizontal_rad(const l3_vec3_t *velocity)
{
    if (velocity->x == 0.0F && velocity->y == 0.0F) {
        return 0.0F;
    }
    return atan2f(velocity->y, velocity->x);
}

float l3_frames_vertical_rad(const l3_vec3_t *velocity)
{
    float horizontal = sqrtf(velocity->x * velocity->x + velocity->y * velocity->y);

    if (horizontal == 0.0F && velocity->z == 0.0F) {
        return 0.0F;
    }
    return atan2f(velocity->z, horizontal);
}

float l3_frames_speed(const l3_vec3_t *velocity)
{
    return sqrtf(velocity->x * velocity->x + velocity->y * velocity->y + velocity->z * velocity->z);
}

int32_t l3_cal_set_element(l3_radar_cal_t *cal, uint32_t index, float gain, float phaseRad)
{
    if (index >= L3_CAL_MAX_VIRTUAL || !(gain > 0.0F)) {
        return -1;
    }
    cal->correctionRe[index] = cosf(-phaseRad) / gain;
    cal->correctionIm[index] = sinf(-phaseRad) / gain;
    if (index >= cal->virtualElements) {
        cal->virtualElements = index + 1U;
    }
    return 0;
}

int32_t l3_cal_element(const l3_radar_cal_t *cal, uint32_t index, float *gain, float *phaseRad)
{
    float re;
    float im;
    float magnitude;

    if (index >= L3_CAL_MAX_VIRTUAL) {
        return -1;
    }
    re = cal->correctionRe[index];
    im = cal->correctionIm[index];
    magnitude = sqrtf(re * re + im * im);
    if (gain != NULL) {
        *gain = (magnitude > 0.0F) ? 1.0F / magnitude : 0.0F;
    }
    if (phaseRad != NULL) {
        *phaseRad = (magnitude > 0.0F) ? -atan2f(im, re) : 0.0F;
    }
    return 0;
}

int32_t l3_cal_format(const l3_radar_cal_t *cal, char *out, uint32_t cap)
{
    char az[16];
    char el[16];
    char pitch[16];
    char yaw[16];
    char roll[16];
    char bias[16];

    l3_text_fixed2(cal->azimuthOffsetRad, az, sizeof(az));
    l3_text_fixed2(cal->elevationOffsetRad, el, sizeof(el));
    l3_text_degrees2(cal->radarPitchRad, pitch, sizeof(pitch));
    l3_text_degrees2(cal->radarYawRad, yaw, sizeof(yaw));
    l3_text_degrees2(cal->radarRollRad, roll, sizeof(roll));
    l3_text_fixed2(cal->rangeBiasM, bias, sizeof(bias));
    return snprintf(out, cap, "cal elems=%u az0=%s el0=%s pitch=%s yaw=%s roll=%s bias=%s",
                    (unsigned)cal->virtualElements, az, el, pitch, yaw, roll, bias);
}

int32_t l3_cal_format_element(const l3_radar_cal_t *cal, uint32_t index, char *out, uint32_t cap)
{
    float gain = 0.0F;
    float phase = 0.0F;
    char gainText[16];
    char phaseText[16];

    if (l3_cal_element(cal, index, &gain, &phase) != 0) {
        return snprintf(out, cap, "elem %u invalid", (unsigned)index);
    }
    l3_text_fixed2(gain, gainText, sizeof(gainText));
    l3_text_fixed2(phase, phaseText, sizeof(phaseText));
    return snprintf(out, cap, "elem %u gain=%s phase=%s", (unsigned)index, gainText, phaseText);
}
