/* IWR6843 impact anchor: where and when the ball was struck, for the ball
 * search to back-project to.
 *
 * Where: the tee bin. Points may only join from acceptFromBin on (the tee
 * band's far edge when one is placed), which is a different number. When:
 * the club's approach fitted to the tee range (l3_impact_fit_track, club in)
 * when that fit is tight, else the gate / range-crossing time with the gate's
 * tolerance. The legacy acquisition keeps gateUs. Pure C, no hardware.
 */
#ifndef L3_BALL_ANCHOR_H
#define L3_BALL_ANCHOR_H

#include <stdint.h>

#include "l3_club_track.h"
#include "l3_impact_fit.h"

enum { L3_BALL_ANCHOR_GATE = 0, L3_BALL_ANCHOR_CLUB = 1 };

/* The club fit's tolerance is 3 sigma, never under this. */
#define L3_BALL_ANCHOR_MIN_TOL_US 2000U

typedef struct {
    float    anchorBin;      /* the ball's range at impact (the tee), global sub-bin */
    float    acceptFromBin;  /* nothing short of this joins: the band's far edge, else the tee */
    uint32_t gateUs;         /* the gate / range-crossing time: the legacy impact time */
    uint32_t anchorUs;       /* the impact time the search back-projects to */
    uint32_t anchorTolUs;
    float    anchorSigmaUs;  /* the club fit's sigma; 0 from the gate */
    uint8_t  source;         /* L3_BALL_ANCHOR_* */
} l3_ball_anchor_t;

/* The gate anchor (anchorUs = gateUs, tolerance gateTolUs), replaced by the
 * club's approach when club holds points, maxSigmaUs > 0 and the club-in fit
 * against anchorBin x fitCfg->binWidthM is OK within maxSigmaUs. */
void l3_ball_anchor_make(float anchorBin, float acceptFromBin, uint32_t gateUs,
                         uint32_t gateTolUs, const l3_impact_fit_cfg_t *fitCfg,
                         const l3_club_track_t *club, float maxSigmaUs, l3_ball_anchor_t *out);
const char *l3_ball_anchor_source_name(uint8_t source);
uint32_t l3_ball_anchor_struct_bytes(void);

#endif /* L3_BALL_ANCHOR_H */
