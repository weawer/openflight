/* IWR6843 self-trigger: an approaching-clubhead detector over MTI residuals.
 *
 * l3_dump.c reduces each completed frame to one observation per range bin of
 * the watch region (integrated burst-MTI residual energy across every loop,
 * plus the lag-1 loop autocorrelation for Doppler). This module owns
 * everything after that: the adaptive noise floor, candidate selection, the
 * short range track, the impact gate, and a flight-recorder log that says
 * per frame what was seen and why it did or did not fire. It touches no
 * hardware, so tests/test_iwr6843_firmware_trigger.py builds it with the
 * host compiler and drives synthetic swings through it.
 *
 * Geometry: the clubhead approaches from short of the ball, so "toward the
 * tee" means a rising local bin. State machine:
 *
 *   IDLE      no candidate above floor * snr in the watch region
 *   TRACKING  a candidate is followed frame to frame by range continuity
 *   FIRED     the track entered the impact gate around the tee bin old
 *             enough, fast enough and from far enough away
 *
 * The approach is measured from the track's nearest point to the radar:
 * mean rate in bins per frame (minStepBins) and total progress in bins
 * (minApproachBins). A return standing in the gate resets that nearest
 * point every frame, so it has no approach to judge and never fires: a
 * hand placing the ball, a player's arm at address.
 *
 * The club is never required to be seen moving away again: that only makes
 * the trigger late and adds a condition a real swing can fail.
 */
#ifndef L3_TRIGGER_H
#define L3_TRIGGER_H

#include <stdint.h>

#include "l3_observation.h"

/* Watch-region and gate limits. The region is at most one capture window. */
#define L3_TRIG_MAX_BINS          64U
/* Flight-recorder depth: frames with a candidate or an active track. Idle
 * frames only count toward the next record's gap, so a missed swing stays
 * readable for as long as the player takes to ask for it. */
#define L3_TRIG_LOG_DEPTH         64U
/* Raw-input trace: the region's strongest bin, every frame it reaches
 * L3_TRIG_TRACE_RATIO times the floor (well under any snr worth arming
 * with), with its energy, strongest loop and loop-0 power. Answers "did the
 * radar see anything at all" when the log stays empty. */
#define L3_TRIG_TRACE_DEPTH       32U
#define L3_TRIG_TRACE_RATIO       2.0F
/* A track survives this many frames without a candidate. */
#define L3_TRIG_MAX_MISSES        1U
/* A candidate up to this many bins short of the last one still continues
 * the track (scatterer wander, a slow backswing); more is a retreat and
 * restarts the track. The approach rate is measured from the track's
 * nearest point to the radar, so a tolerated retreat does not count
 * against the downswing that follows it. */
#define L3_TRIG_JITTER_BINS       2U
/* A candidate more than this many bins ahead of the last one is a jump
 * (another scatterer), not the same target: 8 bins/frame is 125 m/s at
 * 4.7 cm bins and 3 ms frames. */
#define L3_TRIG_MAX_STEP_BINS     8U
/* Noise-floor smoothing: floor += (median - floor) / 2^shift each frame. */
#define L3_TRIG_FLOOR_SHIFT       3U
/* Residual-energy floor never falls below this, so snr stays finite. */
#define L3_TRIG_FLOOR_MIN         1.0F
/* Carrier wavelength for the reported Doppler velocity. The 60 GHz profiles
 * sweep 60.3-63.8 GHz; 62 GHz is close enough for a diagnostic. */
#define L3_TRIG_WAVELENGTH_M      0.00484F

/* Defaults for the optional triggerCfg parameters. */
#define L3_TRIG_DEFAULT_APPROACH_BINS 12U   /* ~0.56 m short of the tee */
#define L3_TRIG_DEFAULT_GATE_BINS     3U    /* ~0.14 m either side of it */
#define L3_TRIG_DEFAULT_MIN_COHERENCE 0.0F  /* off until measured */
#define L3_TRIG_DEFAULT_MIN_STEP_BINS 1.0F  /* ~15 m/s radial at 3 ms */
#define L3_TRIG_DEFAULT_MIN_SPEED_MPS 0.0F  /* Doppler gate off until measured */
/* A club is seen crossing at least a gate's width (~0.14 m) before it may
 * fire; two bins of progress is what an arm settling at the tee showed. */
#define L3_TRIG_DEFAULT_MIN_APPROACH_BINS 3U
#define L3_TRIG_DEFAULT_STAT          L3_TRIG_STAT_PEAK

/* The detection statistic is the observation layer's (l3_observation.h). */
#define L3_TRIG_STAT_ENERGY L3_OBS_STAT_ENERGY
#define L3_TRIG_STAT_PEAK   L3_OBS_STAT_PEAK

enum {
    L3_TRIG_STATE_IDLE = 0,
    L3_TRIG_STATE_TRACKING = 1,
    L3_TRIG_STATE_FIRED = 2
};

/* Why a frame was logged. QUIET frames are never logged. */
enum {
    L3_TRIG_WHY_QUIET = 0,
    L3_TRIG_WHY_ACQUIRED,       /* new track started at the candidate */
    L3_TRIG_WHY_ADVANCED,       /* candidate continued the track */
    L3_TRIG_WHY_JUMPED,         /* candidate broke continuity; track restarted */
    L3_TRIG_WHY_MISSED,         /* no candidate; track kept for now */
    L3_TRIG_WHY_LOST,           /* no candidate too many times; track dropped */
    L3_TRIG_WHY_LOW_COHERENCE,  /* strongest bin above floor but not coherent */
    L3_TRIG_WHY_LOW_DOPPLER,    /* strongest bin above floor but too slow in Doppler */
    L3_TRIG_WHY_TOO_YOUNG,      /* in the gate before trackFrames observations */
    L3_TRIG_WHY_TOO_SLOW,       /* in the gate but approaching under minStep, or not at all */
    L3_TRIG_WHY_TOO_SHORT,      /* in the gate but fewer than minApproachBins of approach seen */
    L3_TRIG_WHY_FIRED,
    L3_TRIG_WHY_COUNT
};

/* Counters reported by the summary line, in this order. */
enum {
    L3_TRIG_COUNT_FRAMES = 0,
    L3_TRIG_COUNT_CANDIDATES,
    L3_TRIG_COUNT_ACQUIRED,
    L3_TRIG_COUNT_ADVANCED,
    L3_TRIG_COUNT_JUMPED,
    L3_TRIG_COUNT_MISSED,
    L3_TRIG_COUNT_LOST,
    L3_TRIG_COUNT_LOW_COHERENCE,
    L3_TRIG_COUNT_LOW_DOPPLER,
    L3_TRIG_COUNT_TOO_YOUNG,
    L3_TRIG_COUNT_TOO_SLOW,
    L3_TRIG_COUNT_TOO_SHORT,
    L3_TRIG_COUNT_FIRED,
    L3_TRIG_COUNT_TOTAL
};

typedef struct {
    uint32_t teeBin;        /* global range bin of the tee (default destination) */
    float    snr;           /* candidate threshold = floor * snr (>= 1) */
    uint32_t trackFrames;   /* observations a track needs before it can fire */
    uint32_t approachBins;  /* watch this many bins short of the tee */
    uint32_t gateBins;      /* impact gate half-width around the tee */
    float    minCoherence;  /* 0..1; 0 disables the Doppler coherence test */
    float    minStepBins;   /* minimum mean approach rate, bins per frame */
    uint32_t stat;          /* L3_TRIG_STAT_ENERGY or L3_TRIG_STAT_PEAK */
    /* Minimum apparent Doppler speed of a candidate, m/s; 0 disables. A body
     * in the lane moves under 1 m/s and reads as such; a clubhead aliases
     * across the +/- lambda/(4T) span and reads as |v| uniformly over it, so
     * a gate of v rejects a club frame with probability v / span (~1.5/9),
     * which one bridged miss mostly absorbs. */
    float    minSpeedMps;
    /* Minimum bins of approach a track must have shown, from its nearest
     * point to the radar to the frame in the gate, before it may fire. The
     * mean rate (minStepBins) alone is measured from that nearest point, so
     * a return that steps two bins in two frames reads as a bin per frame
     * whether it is a clubhead or the strongest scatterer of an arm
     * wandering; a clubhead also covers ground. 0 disables. */
    uint32_t minApproachBins;
} l3_trig_cfg_t;

/* One range bin of one frame is the observation layer's l3_bin_obs_t. */
typedef l3_bin_obs_t l3_trig_obs_t;

/* One traced frame: the region's strongest bin by the configured statistic. */
typedef struct {
    uint32_t frame;
    uint16_t gap;           /* untraced frames since the previous entry */
    uint8_t  bin;           /* strongest global bin */
    uint8_t  state;         /* detector state after the frame */
    float    energy;
    float    peak;
    float    loop0;
    float    floor;         /* in the configured statistic's units */
    float    threshold;     /* floor x snr in force this frame */
    uint8_t  coherencePct;  /* |lag-1 autocorrelation| / energy */
    uint8_t  dest;          /* destination global bin; dist = dest - bin */
} l3_trig_trace_t;

typedef struct {
    uint32_t frame;
    uint16_t gap;           /* quiet frames since the previous record */
    uint8_t  state;         /* after this frame */
    uint8_t  why;
    uint8_t  bin;           /* candidate global bin; 0xFF when none */
    uint8_t  age;
    int16_t  velocityCms;   /* apparent (aliased) Doppler velocity */
    float    energy;
    float    peak;
    float    floor;         /* in the configured statistic's units */
    uint8_t  coherencePct;
    uint8_t  dest;          /* destination (tee or locked ball) global bin */
} l3_trig_record_t;

typedef struct {
    l3_trig_cfg_t cfg;
    uint8_t  state;
    float    floor;
    float    loopPeriodS;   /* for the velocity readout; 0 disables it */
    /* Track. */
    uint8_t  trackBin;
    uint8_t  trackStartBin;  /* nearest bin to the radar the track has held */
    uint8_t  trackAge;
    uint8_t  trackMisses;
    uint32_t trackStartFrame;
    /* Flight recorder. */
    uint32_t counters[L3_TRIG_COUNT_TOTAL];
    uint32_t quietSince;    /* quiet frames since the last record */
    uint32_t logNext;       /* ring write index */
    uint32_t logCount;      /* records held, at most L3_TRIG_LOG_DEPTH */
    l3_trig_record_t log[L3_TRIG_LOG_DEPTH];
    /* Raw-input trace and per-bin maximum since arming or the last clear. */
    uint32_t traceQuiet;
    uint32_t traceNext;
    uint32_t traceCount;
    l3_trig_trace_t trace[L3_TRIG_TRACE_DEPTH];
    uint32_t maxFirstBin;   /* region start the max-hold indices refer to */
    uint32_t maxBins;
    float    maxStat[L3_TRIG_MAX_BINS];
    uint32_t maxFrame[L3_TRIG_MAX_BINS];
} l3_trig_t;

/* Fill cfg with the defaults for the optional parameters. */
void l3_trig_cfg_defaults(l3_trig_cfg_t *cfg);
/* Reject a configuration the detector cannot run. Returns 0 when usable. */
int32_t l3_trig_cfg_check(const l3_trig_cfg_t *cfg);
/* Reset state, floor, track, counters and the log. */
void l3_trig_init(l3_trig_t *trig, const l3_trig_cfg_t *cfg, float loopPeriodS);
/* The capture ring was re-armed for the next shot: drop the track and any
 * fired state so the detector can fire again. The floor, counters and log
 * survive, so a shot's log is still readable after its ring was read. */
void l3_trig_rearm(l3_trig_t *trig);
/* Every bin the detector speaks of is a GLOBAL range-FFT bin (0..127 on a
 * 128-point FFT), never a capture-window offset: the window start moves
 * between profiles and between the pre and post phases, and a tee bin read
 * in the wrong coordinates watched an empty stretch of air.
 *
 * Watch region for a frame whose window holds binCount bins from global bin
 * windowStart, around the destination teeBin (global). firstLocal is the
 * window offset to index the frame with; firstLocal + windowStart is the
 * global bin of obs[0]. Returns 0 when the tee is outside the window. */
int32_t l3_trig_region(const l3_trig_cfg_t *cfg, uint32_t teeBin, uint32_t windowStart,
                       uint32_t binCount, uint32_t *firstLocal, uint32_t *count);
/* Feed one frame: obs[i] describes global bin firstBin + i and the impact
 * gate sits around teeBin (global; the configured tee or, when the ball
 * detector is followed, the ball's bin). Returns 1 when this frame fires the
 * trigger, else 0. After firing, further frames are ignored until
 * l3_trig_init or l3_trig_rearm. */
int32_t l3_trig_update(l3_trig_t *trig, uint32_t frame, uint32_t teeBin, uint32_t firstBin,
                       const l3_trig_obs_t *obs, uint32_t count);

/* Read side for the triggerLog command and the host tests. */
uint32_t l3_trig_log_count(const l3_trig_t *trig);
/* Record index 0 is the oldest held. Returns 0 when index is out of range. */
int32_t l3_trig_log_get(const l3_trig_t *trig, uint32_t index,
                        l3_trig_record_t *out);
/* Integer-only text (the R4F CLI printf may lack %f). Both return the
 * length snprintf reports; cap counts the terminating NUL. */
int32_t l3_trig_format_summary(const l3_trig_t *trig, char *out, uint32_t cap);
int32_t l3_trig_format_config(const l3_trig_t *trig, char *out, uint32_t cap);
int32_t l3_trig_format_record(const l3_trig_record_t *record, char *out,
                              uint32_t cap);
const char *l3_trig_why_name(uint8_t why);

/* Raw-input trace. clear empties the trace and the max-hold only. */
void l3_trig_trace_clear(l3_trig_t *trig);
uint32_t l3_trig_trace_count(const l3_trig_t *trig);
int32_t l3_trig_trace_get(const l3_trig_t *trig, uint32_t index, l3_trig_trace_t *out);
int32_t l3_trig_format_trace_header(const l3_trig_t *trig, char *out, uint32_t cap);
int32_t l3_trig_format_trace(const l3_trig_trace_t *entry, char *out, uint32_t cap);
/* Max-hold for up to count region bins from index start: "trigmax b:stat@frame ...". */
int32_t l3_trig_format_maxhold(const l3_trig_t *trig, uint32_t start, uint32_t count,
                               char *out, uint32_t cap);

#endif /* L3_TRIGGER_H */
