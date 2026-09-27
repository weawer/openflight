/* IWR6843 self-trigger detector. See l3_trigger.h for the design. Pure C,
 * no hardware: l3_dump.c feeds it observations and acts on the result. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "l3_trigger.h"

#define L3_TRIG_NO_BIN 0xFFU
#define L3_TRIG_PI 3.14159265F

static void l3_trig_dropTrack(l3_trig_t *trig);

static const char *const kWhyNames[L3_TRIG_WHY_COUNT] = {
    "quiet", "acquired", "advanced", "jumped", "missed", "lost",
    "lowcoh", "slowdop", "young", "slow", "fired"};

static const char *const kStateNames[3] = {"idle", "tracking", "fired"};

/* Which counter a logged frame reason bumps; -1 for none. */
static const int8_t kWhyCounter[L3_TRIG_WHY_COUNT] = {
    -1,
    (int8_t)L3_TRIG_COUNT_ACQUIRED,
    (int8_t)L3_TRIG_COUNT_ADVANCED,
    (int8_t)L3_TRIG_COUNT_JUMPED,
    (int8_t)L3_TRIG_COUNT_MISSED,
    (int8_t)L3_TRIG_COUNT_LOST,
    (int8_t)L3_TRIG_COUNT_LOW_COHERENCE,
    (int8_t)L3_TRIG_COUNT_LOW_DOPPLER,
    (int8_t)L3_TRIG_COUNT_TOO_YOUNG,
    (int8_t)L3_TRIG_COUNT_TOO_SLOW,
    (int8_t)L3_TRIG_COUNT_FIRED};

void l3_trig_cfg_defaults(l3_trig_cfg_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->approachBins = L3_TRIG_DEFAULT_APPROACH_BINS;
    cfg->gateBins = L3_TRIG_DEFAULT_GATE_BINS;
    cfg->minCoherence = L3_TRIG_DEFAULT_MIN_COHERENCE;
    cfg->minStepBins = L3_TRIG_DEFAULT_MIN_STEP_BINS;
    cfg->stat = L3_TRIG_DEFAULT_STAT;
    cfg->minSpeedMps = L3_TRIG_DEFAULT_MIN_SPEED_MPS;
}

int32_t l3_trig_cfg_check(const l3_trig_cfg_t *cfg)
{
    if (!(cfg->snr >= 1.0F) || cfg->trackFrames == 0U)
    {
        return -1;
    }
    if (cfg->approachBins == 0U || cfg->approachBins > L3_TRIG_MAX_BINS)
    {
        return -1;
    }
    /* The gate must sit inside the watched approach, or nothing can be
     * tracked before it crosses. The record stores bins in a byte. */
    if (cfg->gateBins >= cfg->approachBins ||
        cfg->teeBin + cfg->gateBins >= L3_TRIG_NO_BIN)
    {
        return -1;
    }
    if (!(cfg->minCoherence >= 0.0F) || cfg->minCoherence > 1.0F)
    {
        return -1;
    }
    if (!(cfg->minStepBins >= 0.0F))
    {
        return -1;
    }
    if (cfg->stat > L3_TRIG_STAT_PEAK)
    {
        return -1;
    }
    if (!(cfg->minSpeedMps >= 0.0F))
    {
        return -1;
    }
    return 0;
}

/* Apparent radial velocity of one observation from its lag-1 phase, m/s;
 * aliased at +/- wavelength / (4 * loopPeriod). Zero without a loop period.
 * The sign follows the stored (Im, Re) order and is a readout, not a gate. */
static float l3_trig_velocity(const l3_trig_t *trig, const l3_trig_obs_t *obs)
{
    if (trig->loopPeriodS <= 0.0F || obs->energy <= 0.0F)
    {
        return 0.0F;
    }
    return atan2f(obs->r1Im, obs->r1Re) * L3_TRIG_WAVELENGTH_M /
           (4.0F * L3_TRIG_PI * trig->loopPeriodS);
}

/* The configured detection statistic of one observation. */
static float l3_trig_stat(const l3_trig_cfg_t *cfg, const l3_trig_obs_t *obs)
{
    return (cfg->stat == L3_TRIG_STAT_PEAK) ? obs->peak : obs->energy;
}

void l3_trig_init(l3_trig_t *trig, const l3_trig_cfg_t *cfg, float loopPeriodS)
{
    memset(trig, 0, sizeof(*trig));
    trig->cfg = *cfg;
    trig->loopPeriodS = loopPeriodS;
    trig->state = L3_TRIG_STATE_IDLE;
    trig->trackBin = L3_TRIG_NO_BIN;
}

void l3_trig_trace_enable(l3_trig_t *trig, uint8_t enabled)
{
    trig->traceEnabled = enabled ? 1U : 0U;
}

void l3_trig_trace_clear(l3_trig_t *trig)
{
    trig->traceQuiet = 0U;
    trig->traceNext = 0U;
    trig->traceCount = 0U;
    trig->maxBins = 0U;
    trig->maxFirstBin = 0U;
    memset(trig->maxStat, 0, sizeof(trig->maxStat));
    memset(trig->maxFrame, 0, sizeof(trig->maxFrame));
}

/* Every frame: the region's strongest bin into the max-hold, and into the
 * trace when it clears the trace bar. Independent of the track, so the
 * trace shows what the detector was offered, not what it took. */
static void l3_trig_trace(l3_trig_t *trig, uint32_t frame, uint32_t teeBin, uint32_t firstBin,
                          const l3_trig_obs_t *obs, uint32_t count)
{
    const l3_trig_cfg_t *cfg = &trig->cfg;
    uint32_t strongest = 0U;
    uint32_t i;
    float strongestStat;

    if (trig->maxFirstBin != firstBin || trig->maxBins != count)
    {
        /* A different region (re-arm on another tee, another window). */
        trig->maxFirstBin = firstBin;
        trig->maxBins = count;
        memset(trig->maxStat, 0, sizeof(trig->maxStat));
        memset(trig->maxFrame, 0, sizeof(trig->maxFrame));
    }
    for (i = 0U; i < count; i++)
    {
        float stat = l3_trig_stat(cfg, &obs[i]);
        if (stat > trig->maxStat[i])
        {
            trig->maxStat[i] = stat;
            trig->maxFrame[i] = frame;
        }
        if (stat > l3_trig_stat(cfg, &obs[strongest]))
        {
            strongest = i;
        }
    }
    strongestStat = l3_trig_stat(cfg, &obs[strongest]);
    if (strongestStat < L3_TRIG_TRACE_RATIO * trig->floor)
    {
        trig->traceQuiet++;
        return;
    }
    {
        l3_trig_trace_t *entry = &trig->trace[trig->traceNext];
        entry->frame = frame;
        entry->gap = (trig->traceQuiet > 0xFFFFU) ? 0xFFFFU : (uint16_t)trig->traceQuiet;
        entry->bin = (uint8_t)(firstBin + strongest);
        entry->state = trig->state;
        entry->energy = obs[strongest].energy;
        entry->peak = obs[strongest].peak;
        entry->loop0 = obs[strongest].loop0;
        entry->floor = trig->floor;
        entry->threshold = trig->floor * cfg->snr;
        entry->dest = (uint8_t)teeBin;
        entry->coherencePct = 0U;
        if (obs[strongest].energy > 0.0F)
        {
            float magnitude = sqrtf(obs[strongest].r1Re * obs[strongest].r1Re +
                                    obs[strongest].r1Im * obs[strongest].r1Im);
            float coherence = magnitude / obs[strongest].energy;
            if (coherence > 1.0F)
            {
                coherence = 1.0F;
            }
            entry->coherencePct = (uint8_t)(coherence * 100.0F + 0.5F);
        }
    }
    trig->traceNext = (trig->traceNext + 1U) % L3_TRIG_TRACE_DEPTH;
    if (trig->traceCount < L3_TRIG_TRACE_DEPTH)
    {
        trig->traceCount++;
    }
    trig->traceQuiet = 0U;
}

void l3_trig_rearm(l3_trig_t *trig)
{
    l3_trig_dropTrack(trig);
    trig->state = L3_TRIG_STATE_IDLE;
}

int32_t l3_trig_region(const l3_trig_cfg_t *cfg, uint32_t teeBin, uint32_t windowStart,
                       uint32_t binCount, uint32_t *firstLocal, uint32_t *count)
{
    uint32_t teeLocal;
    uint32_t first;
    uint32_t last;

    *firstLocal = 0U;
    *count = 0U;
    if (binCount == 0U || teeBin < windowStart || teeBin - windowStart >= binCount)
    {
        return 0;
    }
    teeLocal = teeBin - windowStart;
    first = (teeLocal > cfg->approachBins) ? (teeLocal - cfg->approachBins) : 0U;
    last = teeLocal + cfg->gateBins;
    if (last > binCount - 1U)
    {
        last = binCount - 1U;
    }
    *firstLocal = first;
    *count = last - first + 1U;
    if (*count > L3_TRIG_MAX_BINS)
    {
        *count = L3_TRIG_MAX_BINS;
    }
    return 1;
}

/* Median of count energies (count <= L3_TRIG_MAX_BINS). Insertion sort of a
 * copy: the region is a dozen or so bins, so this is cheaper than anything
 * cleverer. */
static float l3_trig_median(const l3_trig_cfg_t *cfg, const l3_trig_obs_t *obs,
                            uint32_t count)
{
    /* Static: the caller's task stack is small and only one task scores
     * frames. Not reentrant. */
    static float sorted[L3_TRIG_MAX_BINS];
    uint32_t i;

    for (i = 0U; i < count; i++)
    {
        float value = l3_trig_stat(cfg, &obs[i]);
        uint32_t j = i;
        while (j > 0U && sorted[j - 1U] > value)
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = value;
    }
    if ((count & 1U) != 0U)
    {
        return sorted[count / 2U];
    }
    return 0.5F * (sorted[count / 2U - 1U] + sorted[count / 2U]);
}

static void l3_trig_record(l3_trig_t *trig, uint32_t frame, uint8_t why,
                           uint8_t bin, uint32_t teeBin, const l3_trig_obs_t *obs)
{
    l3_trig_record_t *record = &trig->log[trig->logNext];
    float magnitude;
    float coherence = 0.0F;
    float velocity = 0.0F;

    if (obs != NULL && obs->energy > 0.0F)
    {
        magnitude = sqrtf(obs->r1Re * obs->r1Re + obs->r1Im * obs->r1Im);
        coherence = magnitude / obs->energy;
        if (coherence > 1.0F)
        {
            coherence = 1.0F;
        }
        velocity = l3_trig_velocity(trig, obs);
    }
    record->frame = frame;
    record->dest = (uint8_t)teeBin;
    record->gap = (trig->quietSince > 0xFFFFU) ? 0xFFFFU : (uint16_t)trig->quietSince;
    record->state = trig->state;
    record->why = why;
    record->bin = bin;
    record->age = trig->trackAge; /* zero once a track is dropped */
    velocity = velocity * 100.0F + ((velocity < 0.0F) ? -0.5F : 0.5F);
    if (velocity > 32767.0F)
    {
        velocity = 32767.0F;
    }
    else if (velocity < -32768.0F)
    {
        velocity = -32768.0F;
    }
    record->velocityCms = (int16_t)velocity;
    record->energy = (obs != NULL) ? obs->energy : 0.0F;
    record->peak = (obs != NULL) ? obs->peak : 0.0F;
    record->floor = trig->floor;
    record->coherencePct = (uint8_t)(coherence * 100.0F + 0.5F);
    trig->logNext = (trig->logNext + 1U) % L3_TRIG_LOG_DEPTH;
    if (trig->logCount < L3_TRIG_LOG_DEPTH)
    {
        trig->logCount++;
    }
    trig->quietSince = 0U;
}

static void l3_trig_startTrack(l3_trig_t *trig, uint32_t frame, uint8_t bin)
{
    trig->trackBin = bin;
    trig->trackStartBin = bin;
    trig->trackAge = 1U;
    trig->trackMisses = 0U;
    trig->trackStartFrame = frame;
}

static void l3_trig_dropTrack(l3_trig_t *trig)
{
    trig->trackBin = L3_TRIG_NO_BIN;
    trig->trackStartBin = L3_TRIG_NO_BIN;
    trig->trackAge = 0U;
    trig->trackMisses = 0U;
    trig->trackStartFrame = 0U;
}

int32_t l3_trig_update(l3_trig_t *trig, uint32_t frame, uint32_t teeBin, uint32_t firstBin,
                       const l3_trig_obs_t *obs, uint32_t count)
{
    const l3_trig_cfg_t *cfg = &trig->cfg;
    const l3_trig_obs_t *best = NULL;
    uint32_t bestIndex = 0U;
    uint32_t i;
    uint8_t bin = L3_TRIG_NO_BIN;
    uint8_t why = L3_TRIG_WHY_QUIET;
    uint8_t haveCandidate = 0U;
    uint8_t continuation = 0U;
    uint8_t trackWasActive = (trig->trackBin != L3_TRIG_NO_BIN) ? 1U : 0U;
    float median;
    float threshold;
    int32_t fired = 0;

    if (trig->state == L3_TRIG_STATE_FIRED || count == 0U)
    {
        return 0;
    }
    if (count > L3_TRIG_MAX_BINS)
    {
        count = L3_TRIG_MAX_BINS;
    }
    trig->counters[L3_TRIG_COUNT_FRAMES]++;

    /* Adaptive floor: the median of the region is noise even while the club
     * occupies a few bins of it. The first frame seeds it outright. */
    median = l3_trig_median(cfg, obs, count);
    if (trig->floor <= 0.0F)
    {
        trig->floor = median;
    }
    else
    {
        trig->floor += (median - trig->floor) / (float)(1U << L3_TRIG_FLOOR_SHIFT);
    }
    if (trig->floor < L3_TRIG_FLOOR_MIN)
    {
        trig->floor = L3_TRIG_FLOOR_MIN;
    }
    threshold = trig->floor * cfg->snr;
    if (trig->traceEnabled)
    {
        l3_trig_trace(trig, frame, teeBin, firstBin, obs, count);
    }

    /* An existing track is continued by the strongest bin above threshold
     * inside its continuation window, even when a stronger return sits
     * elsewhere in the region: the strongest bin can swap between clubhead,
     * shaft, hands and reflections from frame to frame, and following it
     * blindly restarts the track each time. Only without a continuation is
     * the region's strongest bin taken, as a new track. */
    if (trackWasActive)
    {
        int32_t low = (int32_t)trig->trackBin - (int32_t)L3_TRIG_JITTER_BINS;
        int32_t high = (int32_t)trig->trackBin + (int32_t)L3_TRIG_MAX_STEP_BINS;
        for (i = 0U; i < count; i++)
        {
            int32_t candidateBin = (int32_t)(firstBin + i);
            if (candidateBin < low || candidateBin > high)
            {
                continue;
            }
            if (best == NULL || l3_trig_stat(cfg, &obs[i]) > l3_trig_stat(cfg, best))
            {
                best = &obs[i];
                bestIndex = i;
            }
        }
        if (best != NULL && l3_trig_stat(cfg, best) >= threshold)
        {
            continuation = 1U;
        }
        else
        {
            best = NULL;
        }
    }
    if (best == NULL)
    {
        for (i = 0U; i < count; i++)
        {
            if (best == NULL || l3_trig_stat(cfg, &obs[i]) > l3_trig_stat(cfg, best))
            {
                best = &obs[i];
                bestIndex = i;
            }
        }
    }
    if (l3_trig_stat(cfg, best) >= threshold)
    {
        /* The bin is logged either way; only a coherent one is a candidate. */
        bin = (uint8_t)(firstBin + bestIndex);
        haveCandidate = 1U;
        if (cfg->minCoherence > 0.0F)
        {
            float magnitude = sqrtf(best->r1Re * best->r1Re + best->r1Im * best->r1Im);
            if (magnitude < cfg->minCoherence * best->energy)
            {
                haveCandidate = 0U;
                why = L3_TRIG_WHY_LOW_COHERENCE;
            }
        }
        if (haveCandidate && cfg->minSpeedMps > 0.0F && trig->loopPeriodS > 0.0F)
        {
            float speed = l3_trig_velocity(trig, best);
            if (speed < 0.0F)
            {
                speed = -speed;
            }
            if (speed < cfg->minSpeedMps)
            {
                haveCandidate = 0U;
                why = L3_TRIG_WHY_LOW_DOPPLER;
            }
        }
    }
    if (haveCandidate)
    {
        trig->counters[L3_TRIG_COUNT_CANDIDATES]++;
    }

    if (!haveCandidate)
    {
        /* A coherence rejection is the more useful reason to keep when the
         * track also misses because of it; the record's state shows the
         * track's fate. */
        if (trackWasActive)
        {
            trig->trackMisses++;
            if (trig->trackMisses > L3_TRIG_MAX_MISSES)
            {
                l3_trig_dropTrack(trig);
                if (why == L3_TRIG_WHY_QUIET)
                {
                    why = L3_TRIG_WHY_LOST;
                }
            }
            else if (why == L3_TRIG_WHY_QUIET)
            {
                why = L3_TRIG_WHY_MISSED;
            }
        }
    }
    else if (!trackWasActive)
    {
        l3_trig_startTrack(trig, frame, bin);
        why = L3_TRIG_WHY_ACQUIRED;
    }
    else if (!continuation)
    {
        l3_trig_startTrack(trig, frame, bin);
        why = L3_TRIG_WHY_JUMPED;
    }
    else
    {
        trig->trackBin = bin;
        trig->trackMisses = 0U;
        if (trig->trackAge < 0xFFU)
        {
            trig->trackAge++;
        }
        if (bin < trig->trackStartBin)
        {
            /* A real retreat resets the approach origin; equal-bin returns
             * must not keep restarting the approach timer. */
            trig->trackStartBin = bin;
            trig->trackStartFrame = frame;
        }
        why = L3_TRIG_WHY_ADVANCED;
    }

    /* Impact gate: fire on entry, given enough history and a clubhead's
     * approach rate. No post-impact reversal is needed or waited for. */
    if (haveCandidate &&
        (uint32_t)bin + cfg->gateBins >= teeBin &&
        (uint32_t)bin <= teeBin + cfg->gateBins)
    {
        uint32_t elapsed = frame - trig->trackStartFrame;
        int32_t progress = (int32_t)trig->trackBin - (int32_t)trig->trackStartBin;
        if (trig->trackAge < cfg->trackFrames)
        {
            why = L3_TRIG_WHY_TOO_YOUNG;
        }
        else if ((elapsed > 0U && (float)progress < cfg->minStepBins * (float)elapsed) ||
                 (elapsed == 0U && cfg->minStepBins > 0.0F && cfg->trackFrames > 1U))
        {
            why = L3_TRIG_WHY_TOO_SLOW;
        }
        else
        {
            why = L3_TRIG_WHY_FIRED;
            fired = 1;
        }
    }

    if (fired)
    {
        trig->state = L3_TRIG_STATE_FIRED;
    }
    else
    {
        trig->state = (trig->trackBin != L3_TRIG_NO_BIN)
                          ? L3_TRIG_STATE_TRACKING
                          : L3_TRIG_STATE_IDLE;
    }
    if (why == L3_TRIG_WHY_QUIET)
    {
        trig->quietSince++;
        return 0;
    }
    if (kWhyCounter[why] >= 0)
    {
        trig->counters[kWhyCounter[why]]++;
    }
    /* The strongest bin is worth logging even when it failed the coherence
     * test: that is exactly the case the tuner needs to see. */
    l3_trig_record(trig, frame, why, bin, teeBin, best);
    return fired;
}

uint32_t l3_trig_trace_count(const l3_trig_t *trig)
{
    return trig->traceCount;
}

int32_t l3_trig_trace_get(const l3_trig_t *trig, uint32_t index, l3_trig_trace_t *out)
{
    uint32_t oldest;

    if (index >= trig->traceCount)
    {
        return 0;
    }
    oldest = (trig->traceNext + L3_TRIG_TRACE_DEPTH - trig->traceCount) % L3_TRIG_TRACE_DEPTH;
    *out = trig->trace[(oldest + index) % L3_TRIG_TRACE_DEPTH];
    return 1;
}

uint32_t l3_trig_log_count(const l3_trig_t *trig)
{
    return trig->logCount;
}

int32_t l3_trig_log_get(const l3_trig_t *trig, uint32_t index, l3_trig_record_t *out)
{
    uint32_t oldest;

    if (index >= trig->logCount)
    {
        return 0;
    }
    oldest = (trig->logNext + L3_TRIG_LOG_DEPTH - trig->logCount) % L3_TRIG_LOG_DEPTH;
    *out = trig->log[(oldest + index) % L3_TRIG_LOG_DEPTH];
    return 1;
}

const char *l3_trig_why_name(uint8_t why)
{
    return (why < L3_TRIG_WHY_COUNT) ? kWhyNames[why] : "?";
}

/* Fixed-point text for a float: "-12.34". Integer-only printf underneath,
 * so it works wherever %f does not. Values beyond +/- 4e9 saturate. */
static void l3_trig_fmtFixed(float value, uint32_t decimals, char *out, uint32_t cap)
{
    const char *sign = "";
    uint32_t scale = 1U;
    uint32_t whole;
    uint32_t fraction;
    uint32_t i;

    for (i = 0U; i < decimals; i++)
    {
        scale *= 10U;
    }
    if (value < 0.0F)
    {
        sign = "-";
        value = -value;
    }
    if (value > 4.0e9F)
    {
        value = 4.0e9F;
    }
    whole = (uint32_t)value;
    fraction = (uint32_t)((value - (float)whole) * (float)scale + 0.5F);
    if (fraction >= scale)
    {
        whole++;
        fraction = 0U;
    }
    /* Fixed formats rather than "%0*u": the R4F runtime's printf subset is
     * not guaranteed to take a '*' width. */
    if (decimals == 0U)
    {
        (void)snprintf(out, cap, "%s%u", sign, (unsigned)whole);
    }
    else if (decimals == 1U)
    {
        (void)snprintf(out, cap, "%s%u.%01u", sign, (unsigned)whole, (unsigned)fraction);
    }
    else
    {
        (void)snprintf(out, cap, "%s%u.%02u", sign, (unsigned)whole, (unsigned)fraction);
    }
}

int32_t l3_trig_format_summary(const l3_trig_t *trig, char *out, uint32_t cap)
{
    char floorText[16];
    const uint32_t *c = trig->counters;

    l3_trig_fmtFixed(trig->floor, 0U, floorText, sizeof(floorText));
    return snprintf(out, cap,
                    "trig state=%s floor=%s frames=%u cand=%u acq=%u adv=%u "
                    "jump=%u miss=%u lost=%u lowcoh=%u slowdop=%u young=%u slow=%u "
                    "fired=%u records=%u",
                    kStateNames[trig->state], floorText,
                    (unsigned)c[L3_TRIG_COUNT_FRAMES],
                    (unsigned)c[L3_TRIG_COUNT_CANDIDATES],
                    (unsigned)c[L3_TRIG_COUNT_ACQUIRED],
                    (unsigned)c[L3_TRIG_COUNT_ADVANCED],
                    (unsigned)c[L3_TRIG_COUNT_JUMPED],
                    (unsigned)c[L3_TRIG_COUNT_MISSED],
                    (unsigned)c[L3_TRIG_COUNT_LOST],
                    (unsigned)c[L3_TRIG_COUNT_LOW_COHERENCE],
                    (unsigned)c[L3_TRIG_COUNT_LOW_DOPPLER],
                    (unsigned)c[L3_TRIG_COUNT_TOO_YOUNG],
                    (unsigned)c[L3_TRIG_COUNT_TOO_SLOW],
                    (unsigned)c[L3_TRIG_COUNT_FIRED],
                    (unsigned)trig->logCount);
}

int32_t l3_trig_format_config(const l3_trig_t *trig, char *out, uint32_t cap)
{
    char snrText[16];
    char coherenceText[16];
    char stepText[16];
    char speedText[16];
    char loopText[16];
    const char *statText = (trig->cfg.stat == L3_TRIG_STAT_PEAK) ? "peak" : "energy";

    l3_trig_fmtFixed(trig->cfg.minSpeedMps, 2U, speedText, sizeof(speedText));
    l3_trig_fmtFixed(trig->cfg.snr, 2U, snrText, sizeof(snrText));
    l3_trig_fmtFixed(trig->cfg.minCoherence, 2U, coherenceText, sizeof(coherenceText));
    l3_trig_fmtFixed(trig->cfg.minStepBins, 2U, stepText, sizeof(stepText));
    l3_trig_fmtFixed(trig->loopPeriodS * 1.0e6F, 1U, loopText, sizeof(loopText));
    return snprintf(out, cap,
                    "trigcfg tee=%u snr=%s track=%u approach=%u gate=%u "
                    "mincoh=%s minstep=%s stat=%s minspeed=%s loopus=%s",
                    (unsigned)trig->cfg.teeBin, snrText,
                    (unsigned)trig->cfg.trackFrames,
                    (unsigned)trig->cfg.approachBins,
                    (unsigned)trig->cfg.gateBins,
                    coherenceText, stepText, statText, speedText, loopText);
}

int32_t l3_trig_format_record(const l3_trig_record_t *record, char *out, uint32_t cap)
{
    char binText[8];
    char distText[8];
    char energyText[16];
    char peakText[16];
    char floorText[16];
    char velocityText[16];

    if (record->bin == L3_TRIG_NO_BIN)
    {
        (void)snprintf(binText, sizeof(binText), "-");
    }
    else
    {
        (void)snprintf(binText, sizeof(binText), "%u", (unsigned)record->bin);
    }
    l3_trig_fmtFixed(record->energy, 0U, energyText, sizeof(energyText));
    l3_trig_fmtFixed(record->peak, 0U, peakText, sizeof(peakText));
    l3_trig_fmtFixed(record->floor, 0U, floorText, sizeof(floorText));
    l3_trig_fmtFixed((float)record->velocityCms / 100.0F, 2U, velocityText,
                     sizeof(velocityText));
    /* floor is in the configured statistic's units; the host divides. */
    /* dist = dest - bin: bins short of impact, comparable across setups. */
    if (record->bin == L3_TRIG_NO_BIN)
    {
        (void)snprintf(distText, sizeof(distText), "-");
    }
    else
    {
        (void)snprintf(distText, sizeof(distText), "%d",
                       (int)record->dest - (int)record->bin);
    }
    return snprintf(out, cap,
                    "frame=%u gap=%u state=%s why=%s bin=%s dest=%u dist=%s age=%u energy=%s "
                    "peak=%s floor=%s v=%s coh=%u",
                    (unsigned)record->frame, (unsigned)record->gap,
                    (record->state < 3U) ? kStateNames[record->state] : "?",
                    l3_trig_why_name(record->why), binText, (unsigned)record->dest, distText,
                    (unsigned)record->age, energyText, peakText, floorText,
                    velocityText, (unsigned)record->coherencePct);
}

int32_t l3_trig_format_trace_header(const l3_trig_t *trig, char *out, uint32_t cap)
{
    char floorText[16];
    char ratioText[16];

    l3_trig_fmtFixed(trig->floor, 0U, floorText, sizeof(floorText));
    l3_trig_fmtFixed(L3_TRIG_TRACE_RATIO, 1U, ratioText, sizeof(ratioText));
    return snprintf(out, cap,
                    "trigtrace state=%s stat=%s floor=%s bar=%sx frames=%u "
                    "region=%u+%u entries=%u",
                    kStateNames[trig->state],
                    (trig->cfg.stat == L3_TRIG_STAT_PEAK) ? "peak" : "energy",
                    floorText, ratioText,
                    (unsigned)trig->counters[L3_TRIG_COUNT_FRAMES],
                    (unsigned)trig->maxFirstBin, (unsigned)trig->maxBins,
                    (unsigned)trig->traceCount);
}

/* One traced frame. e/f and p/f are energy and peak over the floor, which is
 * in the configured statistic's units, so the ratio for that statistic is
 * the one the threshold (thr = floor x snr) applies to. */
int32_t l3_trig_format_trace(const l3_trig_trace_t *entry, char *out, uint32_t cap)
{
    char energyText[16];
    char peakText[16];
    char loop0Text[16];
    char floorText[16];
    char thresholdText[16];
    char energyRatio[16];
    char peakRatio[16];
    float floor = (entry->floor > 0.0F) ? entry->floor : 1.0F;

    l3_trig_fmtFixed(entry->energy, 0U, energyText, sizeof(energyText));
    l3_trig_fmtFixed(entry->peak, 0U, peakText, sizeof(peakText));
    l3_trig_fmtFixed(entry->loop0, 0U, loop0Text, sizeof(loop0Text));
    l3_trig_fmtFixed(entry->floor, 0U, floorText, sizeof(floorText));
    l3_trig_fmtFixed(entry->threshold, 0U, thresholdText, sizeof(thresholdText));
    l3_trig_fmtFixed(entry->energy / floor, 1U, energyRatio, sizeof(energyRatio));
    l3_trig_fmtFixed(entry->peak / floor, 1U, peakRatio, sizeof(peakRatio));
    return snprintf(out, cap,
                    "t frame=%u gap=%u bin=%u dest=%u dist=%d state=%s energy=%s peak=%s "
                    "loop0=%s floor=%s thr=%s e/f=%s p/f=%s coh=%u",
                    (unsigned)entry->frame, (unsigned)entry->gap, (unsigned)entry->bin,
                    (unsigned)entry->dest, (int)entry->dest - (int)entry->bin,
                    (entry->state < 3U) ? kStateNames[entry->state] : "?",
                    energyText, peakText, loop0Text, floorText, thresholdText,
                    energyRatio, peakRatio, (unsigned)entry->coherencePct);
}

int32_t l3_trig_format_maxhold(const l3_trig_t *trig, uint32_t start, uint32_t count,
                               char *out, uint32_t cap)
{
    int32_t used = snprintf(out, cap, "trigmax");
    uint32_t i;

    for (i = start; i < start + count && i < trig->maxBins; i++)
    {
        char statText[16];
        int32_t written;

        if (used < 0 || (uint32_t)used >= cap)
        {
            break;
        }
        l3_trig_fmtFixed(trig->maxStat[i], 0U, statText, sizeof(statText));
        written = snprintf(out + used, cap - (uint32_t)used, " %u:%s@%u",
                           (unsigned)(trig->maxFirstBin + i), statText,
                           (unsigned)trig->maxFrame[i]);
        if (written < 0)
        {
            break;
        }
        used += written;
    }
    return used;
}
