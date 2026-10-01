/* IWR6843 coordinate frames and radar calibration.
 *
 * Two frames, one transformation between them, defined here and nowhere else:
 *
 *   RADAR frame (metres, right-handed):
 *     x  boresight, out of the antenna face
 *     y  to the radar's right, seen from behind it (the golfer's view)
 *     z  up
 *     azimuth   = atan2(y, x)              positive right
 *     elevation = atan2(z, hypot(x, y))    positive up
 *
 *   GOLF frame (metres, right-handed, origin at the antenna centre):
 *     x  along the target line, away from the golfer
 *     y  to the right of the target line
 *     z  up
 *
 * The radar sits behind the ball looking down the target line, so the two
 * frames differ by the enclosure's installation attitude only: roll about
 * the boresight, pitch (boresight raised above horizontal, nose up positive),
 * yaw (boresight aimed right of the target line, positive). The calibration
 * also carries the per-virtual-element complex corrections the corner
 * reflector solve produced, the electrical zeros of the azimuth and
 * elevation baselines, and the range bias.
 *
 * Angle conventions derived from a velocity vector in the golf frame, used
 * by every metric so no two pieces of code can disagree about a sign:
 *     horizontal angle (club path, horizontal launch) = atan2(vy, vx)
 *         positive = right of the target line (in-to-out for a right-hander,
 *         ball starting right)
 *     vertical angle (angle of attack, vertical launch) = atan2(vz, hypot(vx, vy))
 *         positive = up (ascending club, ball launched upward)
 * Pure C, no hardware.
 */
#ifndef L3_FRAMES_H
#define L3_FRAMES_H

#include <stdint.h>

#define L3_CAL_MAX_VIRTUAL 8U
#define L3_FRAMES_PI       3.14159265F

typedef struct {
    float x;
    float y;
    float z;
} l3_vec3_t;

typedef struct {
    float rangeM;
    float azimuthRad;    /* positive right */
    float elevationRad;  /* positive up */
} l3_spherical_t;

typedef struct {
    uint32_t virtualElements;                 /* corrections in force (8 for 2 TX x 4 RX) */
    float    correctionRe[L3_CAL_MAX_VIRTUAL]; /* complex per element, PHYSICAL order */
    float    correctionIm[L3_CAL_MAX_VIRTUAL]; /* (after the orientation flip) */
    float    azimuthOffsetRad;   /* electrical zero of the azimuth baseline */
    float    elevationOffsetRad; /* likewise for the elevation array */
    float    radarPitchRad;      /* boresight above horizontal, nose up positive */
    float    radarYawRad;        /* boresight right of the target line, positive */
    float    radarRollRad;       /* right side down, seen from behind, positive */
    float    rangeBiasM;         /* subtracted from every measured range */
} l3_radar_cal_t;

/* Unit corrections, zero offsets and attitude. */
void l3_cal_identity(l3_radar_cal_t *cal, uint32_t virtualElements);
/* One element's correction from its measured gain and phase offset: the
 * element is divided by the gain and rotated back by the phase, so a
 * calibrated array reads the physical steering vector. Returns -1 for an
 * index outside the array or a gain that is not positive. */
int32_t l3_cal_set_element(l3_radar_cal_t *cal, uint32_t index, float gain, float phaseRad);
/* The gain and phase offset a correction encodes (the inverse of the above). */
int32_t l3_cal_element(const l3_radar_cal_t *cal, uint32_t index, float *gain, float *phaseRad);
/* "cal elems=8 az0=0.00 el0=0.00 pitch=0.00 yaw=0.00 roll=0.00 bias=0.00" (radians,
 * degrees for the attitude, metres) */
int32_t l3_cal_format(const l3_radar_cal_t *cal, char *out, uint32_t cap);
/* "elem 3 gain=1.00 phase=0.00" (radians) */
int32_t l3_cal_format_element(const l3_radar_cal_t *cal, uint32_t index, char *out, uint32_t cap);

/* Spherical (radar) <-> Cartesian (radar). */
void l3_frames_from_spherical(const l3_spherical_t *in, l3_vec3_t *radar);
void l3_frames_to_spherical(const l3_vec3_t *radar, l3_spherical_t *out);
/* Radar <-> golf, the attitude part of the calibration. */
void l3_frames_radar_to_golf(const l3_radar_cal_t *cal, const l3_vec3_t *radar, l3_vec3_t *golf);
void l3_frames_golf_to_radar(const l3_radar_cal_t *cal, const l3_vec3_t *golf, l3_vec3_t *radar);
/* A measured point in the golf frame: the range less rangeBiasM, then the
 * attitude rotation. The angles are l3_angle_estimate's, which has already
 * removed the az/el offsets (the azimuth one as a phase), so they are not
 * applied here again. */
void l3_frames_observe(const l3_radar_cal_t *cal, float rangeM, float azimuthRad,
                       float elevationRad, l3_vec3_t *golf);
/* Velocity-vector angles per the header conventions. */
float l3_frames_horizontal_rad(const l3_vec3_t *velocity);
float l3_frames_vertical_rad(const l3_vec3_t *velocity);
float l3_frames_speed(const l3_vec3_t *velocity);

#endif /* L3_FRAMES_H */
