/* IWR6843 ball recovery: the frames a chosen ball hypothesis missed.
 *
 * A short ring holds each post-impact frame's targets (the strongest
 * L3_BALL_HISTORY_TARGETS, the club's claimed one flagged). When a hypothesis
 * is adopted as the ball, every held frame up to its newest point without a
 * point of its own is searched along its fitted line: the nearest return
 * within gateM, beyond acceptFromBin and not the club's, joins; two within
 * tieBins of each other go to the one whose Doppler agrees with the rate
 * (Doppler only breaks ties: as a gate it made the ball worse). The merge is
 * kept only while the refit stays within maxResidualBins. Recovered points
 * carry no angles. Pure C, fixed size, no hardware.
 */
#ifndef L3_BALL_RECOVER_H
#define L3_BALL_RECOVER_H

#include <stdint.h>

#include "l3_ball_hyp.h"
#include "l3_observation.h"

#ifndef L3_BALL_RECOVER
#define L3_BALL_RECOVER L3_BALL_HYPOTHESES
#endif

#if L3_BALL_RECOVER && !L3_BALL_HYPOTHESES
#error "L3_BALL_RECOVER needs L3_BALL_HYPOTHESES"
#endif

#define L3_BALL_HISTORY_FRAMES  24U
#define L3_BALL_HISTORY_TARGETS 6U
typedef struct { float rangeBin; float dopplerAliasMps; float stat; float coherence; } l3_ball_history_target_t;
typedef struct { uint32_t frame; uint32_t timestampUs; uint8_t count; uint8_t clubMask;
                 l3_ball_history_target_t targets[L3_BALL_HISTORY_TARGETS]; } l3_ball_history_frame_t;
typedef struct { uint32_t next; uint32_t count; l3_ball_history_frame_t frames[L3_BALL_HISTORY_FRAMES]; } l3_ball_history_t;
typedef struct { float binWidthM; float velocitySpanMps; float gateM; float tieBins;
                 float maxResidualBins; float dopplerToleranceMps; } l3_ball_recover_cfg_t;
typedef struct { uint32_t count; uint32_t recovered; uint32_t firstFrame; uint32_t mask; float residualBins; } l3_ball_recover_result_t;
void l3_ball_recover_cfg_defaults(l3_ball_recover_cfg_t *cfg);
void l3_ball_history_reset(l3_ball_history_t *h);
void l3_ball_history_push(l3_ball_history_t *h, const l3_target_obs_t *targets, uint32_t n,
                          uint32_t frame, uint32_t timestampUs, uint32_t clubIndex);
/* Flag index (into the targets as pushed) as the club's on the newest frame,
 * when that frame is frame and index is within its count; otherwise nothing.
 * The club is followed after the ball's update, so its claim arrives here. */
void l3_ball_history_mark_club(l3_ball_history_t *h, uint32_t frame, uint32_t index);
const l3_ball_history_frame_t *l3_ball_history_at(const l3_ball_history_t *h, uint32_t index); /* 0 oldest; NULL past count */
uint32_t l3_ball_recover(const l3_ball_recover_cfg_t *cfg, const l3_ball_history_t *h,
                         const l3_ball_hyp_t *hyp, float acceptFromBin,
                         l3_ball_hyp_point_t *out, uint32_t cap, l3_ball_recover_result_t *result);
uint32_t l3_ball_history_struct_bytes(void);

#endif /* L3_BALL_RECOVER_H */
