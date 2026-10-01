/* IWR6843 detect core: which core scores a frame's bins.
 *
 * "trackCfg detectCore dss|verify" (l3_dump.c) asks for a core; this
 * decides per frame where its bins are scored and keeps the record of how
 * that went. The observation -> tracker -> trigger path stays on the MSS
 * whatever is chosen: only l3_dsp_spans_score's work moves.
 *
 *   dss     (the default) the DSS scores (SCORE, l3_dsp_ipc.h) while the
 *           MSS detect task waits, blocked, so the CLI and the notices run
 *   verify  both score the same frame at once and must agree bit for bit;
 *           the MSS's observations drive the detector. The live A/B check.
 *
 * mss is not a choice: it is where a frame goes that the DSS cannot take.
 * Only an IQ16 ring frame can go to the DSS (the shared scorer reads int16,
 * and the compact formats' detect frames live in MSS scratch): any other
 * frame, or one that finds the link busy or down, is scored on the MSS and
 * counted as ineligible. verify is refused for a capture the DSS cannot
 * read; dss is not, its frames are all ineligible then. A DSS that fails a
 * frame (no answer in time, a refused or stale result) has the MSS score it
 * instead, counted as a fallback; failLimit failures in a row latch the MSS
 * until dss or verify is chosen again. Verify never latches: a failure
 * there costs nothing, the MSS already scored.
 * Pure C, no hardware.
 */
#ifndef L3_DETECT_CORE_H
#define L3_DETECT_CORE_H

#include <stdint.h>

#define L3_DETECT_CORE_MSS    0U
#define L3_DETECT_CORE_DSS    1U
#define L3_DETECT_CORE_VERIFY 2U
#define L3_DETECT_CORE_COUNT  3U

/* One frame's outcome on the DSS (dss and verify frames). */
#define L3_DETECT_OUTCOME_OK       0U
#define L3_DETECT_OUTCOME_FAILED   1U /* no answer, refused, or stale */
#define L3_DETECT_OUTCOME_MISMATCH 2U /* verify: the cores disagreed */

#define L3_DETECT_CORE_FAIL_LIMIT_DEFAULT 3U

typedef struct {
    uint8_t  requested;   /* what the CLI asked for */
    uint8_t  active;      /* what frames route to: mss once latched */
    uint8_t  latched;     /* failLimit DSS failures in a row fell back to mss */
    uint8_t  failLimit;
    uint32_t failStreak;  /* consecutive dss failures */
    uint32_t mssFrames;   /* routed to the MSS (ineligible frames included) */
    uint32_t dssFrames;   /* routed to the DSS */
    uint32_t verifyFrames;
    uint32_t ineligible;  /* dss/verify asked, but the frame could not go */
    uint32_t failures;    /* dss and verify frames the DSS failed */
    uint32_t fallbacks;   /* dss frames the MSS scored instead */
    uint32_t latches;
    uint32_t mismatches;  /* verify frames the cores disagreed on */
    uint8_t  haveMismatch;
    uint32_t mismatchSlot; /* the first mismatch: ring slot, local bin, field */
    uint32_t mismatchBin;
    uint32_t mismatchField;
    uint32_t dssInvCyclesLast;
    uint32_t dssInvCyclesMax;
    uint32_t dssScoreCyclesLast;
    uint32_t dssScoreCyclesMax;
} l3_detect_core_t;

/* dss, no latch, failLimit L3_DETECT_CORE_FAIL_LIMIT_DEFAULT, no counts. */
void l3_detect_core_init(l3_detect_core_t *core);
/* Zero the counts and the first mismatch; keep the choice and any latch. */
void l3_detect_core_reset_counts(l3_detect_core_t *core);
/* Choose dss or verify. 0, or -1 (nothing changes) for mss or an unknown
 * core, or for verify when the capture cannot feed the DSS (captureEligible
 * 0). Choosing clears a latch and the failure streak. */
int32_t l3_detect_core_set(l3_detect_core_t *core, uint32_t which, uint8_t captureEligible);
/* The core this frame's bins go to, counted. frameEligible 0 sends a dss or
 * verify frame to the MSS as ineligible. */
uint32_t l3_detect_core_route(l3_detect_core_t *core, uint8_t frameEligible);
/* How a dss or verify frame went on the DSS (route as returned by
 * l3_detect_core_route; mss frames are ignored), with the DSS's cycles
 * when it answered (0 otherwise; only an OK or MISMATCH outcome records
 * them). Returns 1 when this failure latched the MSS, else 0. */
int32_t l3_detect_core_report(l3_detect_core_t *core, uint32_t route, uint32_t outcome,
                              uint32_t invCycles, uint32_t scoreCycles);
/* A verify mismatch's place, kept only for the first since the counts were
 * reset. Report the frame's outcome as MISMATCH as well. */
void l3_detect_core_note_mismatch(l3_detect_core_t *core, uint32_t slot, uint32_t bin,
                                  uint32_t field);

/* "mss", "dss", "verify"; NULL for anything else. */
const char *l3_detect_core_name(uint32_t which);
/* 0 with *which set, or -1 for a name that is not "dss" or "verify". */
int32_t l3_detect_core_parse(const char *name, uint32_t *which);
/* "detect core=dss active=mss latched=1 mss=N dss=N verify=N ineligible=N
 *  failures=N fallbacks=N streak=N latches=N mismatches=N
 *  dss_inv_us=L/M dss_score_us=L/M" (last/max at dssClockMhz), then
 *  " first_mismatch=slot:bin:field" when there is one. */
int32_t l3_detect_core_format(const l3_detect_core_t *core, uint32_t dssClockMhz, char *out,
                              uint32_t cap);

#endif /* L3_DETECT_CORE_H */
