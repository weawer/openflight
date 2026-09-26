# Plan: 2 ms IQ16 capture with onboard sample retention

> Source requirements: this conversation, clarified on 2026-09-26. Capture more
> full-precision measurements during the same useful ball flight by processing
> and selecting samples before they occupy long-term L3 storage.
> Baseline: `feat/iwr-offloading`, commit `a38fbe8`.
> Status: five-phase breakdown accepted by the user; detailed budgets and
> hardware gates remain provisional. No new capture mode implemented or
> hardware validation performed as part of this planning task.

## Objective and scope

Target 36 frames at 2 ms spacing, nominally 72 ms, retaining complex IQ16 values
for all three TX and four RX channels and all 12 loops at selected ranges. This
provides 50% more frame observations than the current 24-frame/3 ms IQ16 profile.
It does not change the ADC sample rate or shorten the same-TX chirp interval.

The first implementation changes acquisition and retention, not launch-angle,
club-path, or spin algorithms. Keep dense evidence around impact and use narrower
windows only when tracking supports that decision. Do not extend flight time,
retune RF mid-shot, drop antenna channels, reduce loop count, or compress to IQ8
to claim the target has been achieved. Retain the existing profiles as explicit
alternatives.

User stories:

- U1: Obtain 2 ms IQ16 observations during the useful flight without exceeding L3.
- U2: Preserve the evidence needed for club and ball measurements.
- U3: Repeatedly capture and rearm without lost triggers, overwritten data or
  competing serial readers.
- U4: Inspect retained samples, selection decisions and timing to validate the
  result before trusting it on the range.

## Double-check findings

### What is established from the current code

- The capture arena is 786,432 bytes (768 KiB). It is explicit RAM storage,
  rather than a cache whose capacity can be bypassed simply by using the CPU.
- HWA already calculates all 128 range-FFT bins. EDMA currently crops those
  outputs into saved windows; IQ16 normally goes directly into the capture arena.
- Current onboard track selection runs after freezing the recording. It reduces
  transfer volume but cannot recover storage that was already used during capture.
- The dense 2 ms alternative uses IQ8. Existing sparse/onboard dump selection
  rejects IQ8, and the self-trigger sample reader assumes IQ16. Do not use that
  combination as an unqualified baseline. Unsupported combinations need an
  explicit rejection until supported.
- Existing dump descriptors already support varying contiguous range windows,
  IQ16, three TX channels and per-frame elapsed times. Reuse that contract where
  possible instead of expanding sparse two-TX dumps back into nominal full cubes.
- Existing onboard/sparse transfer selects the vertical TX pair. The new retained
  capture must preserve all physical TX channels; that transfer path cannot be
  reused unchanged as the complete acquisition record.
- Some current tracking configuration paths use a 90 microsecond loop-period
  constant inherited from two-TX operation. The target three-TX profile has a
  135 microsecond same-TX interval. Timing must come from the physical profile,
  even when downstream code selects only two TX channels.

### Timing constraint

The existing chirp has 7 microseconds idle plus 38 microseconds ramp time:

- 45 microseconds per chirp.
- Three TX times 12 loops = 36 chirps per frame.
- Nominal RF burst = 1,620 microseconds.
- A 2,000 microsecond frame period leaves about 380 microseconds outside the burst.

The frame rearm path must meet that short deadline. Selection must not become a
large synchronous job before rearming. Use two temporary frame buffers: rearm
acquisition into the other buffer immediately, then process the completed frame
while the next frame is acquired. Processing and copying must finish before that
buffer is reused. Exact deadlines, bus contention and CPU headroom require actual
measurements; two buffers alone do not prove this will work.

Staging all 128 bins at this cadence writes about 36.9 MB/s through EDMA before
additional CPU reads and retained-data copies. That is a real increase over
cropping 53 bins and must be included in the timing test.

### Provisional memory budget

Each retained range bin in one frame costs:

`12 loops × 3 TX × 4 RX × 4 bytes/complex sample = 576 bytes`.

Keeping 53 bins for all 36 frames would require 1,099,008 bytes before scratch
space. It does not fit. A possible bounded layout is:

| Allocation | Geometry | Bytes |
| --- | --- | ---: |
| Circular pre-trigger history | 14 frames × 32 bins, all channels | 258,048 |
| Broad early post-trigger evidence | 6 frames × 53 bins, all channels | 183,168 |
| Tracked flight evidence | 16 frames × 12 bins, all channels | 110,592 |
| Two full-range temporary IQ16 frames | 2 × 128 bins, all channels | 147,456 |
| Descriptor/diagnostic reservation | Budget, to be measured | 16,384 |
| Additional selection workspace reservation | Budget, to be measured | 32,768 |
| **Total reservation** | | **748,416** |
| **Unallocated margin** | 786,432 − 748,416 | **38,016** |

This is a feasibility budget, not a validated production setting. It assumes
exactly 12 loops; a generic maximum-loop scratch allocation must not silently
replace this calculation. Existing firmware globals, task stacks and tracker
workspace also occupy other memory regions: audit the complete linker map and
runtime stack peaks, not only L3.

The 32-bin pre-trigger window spans approximately 1.5 m with the current range
geometry. It must be placed relative to the measured tee location and retain the
club approach. The 12-bin flight window spans about 0.56 m. At 75 m/s, the ball
moves about 0.15 m, or 3.2 bins, in 2 ms. Window margins must cover motion during
all chirps of a frame, prediction error and noise/context requirements.

The split is nominally 28 ms before the trigger, 12 ms broad post-trigger and
32 ms tracked post-trigger. These times are relative to the IWR trigger, not
necessarily impact. Replay and hardware tests must verify that actual impact
and the useful club track remain inside the broad evidence. If those tests fail,
revise this budget rather than silently sacrificing club measurements.

Staging the full range means the selector sees the current frame before deciding
what to retain; it does not have to trust a predicted narrow acquisition window.
Prediction helps associate candidates, but does not prevent seeing an unexpected
return elsewhere in that frame.

### Separate acquisition capacity from transfer time

The illustrative retained sample payload is 551,808 bytes: approximately 5.30 s
at the current 1,041,667-baud 8N1 UART, before headers and processing. This design
solves the 2 ms IQ16 storage problem, not automatically the entire results-latency
problem. Keep reduced transfer as an option and measure it separately; selected
transfer must preserve the channels and evidence its consumer needs. Full retained
captures remain the diagnostic reference. A retained capture is not a full cube:
bins discarded onboard are irrecoverable.

## Architectural decisions

- **Acquisition:** keep the current chirps, FFT size, 3 TX, 4 RX and 12 loops;
  change frame period and retention. Leave RF power and range span unchanged.
- **Storage:** partition one bounded arena into temporary acquisition buffers,
  pre-trigger history, post-trigger retained evidence and measured reservations.
  Reject configurations that exceed the complete budget before starting RF.
- **Ownership:** each temporary buffer has explicit filling, completed,
  processing and reusable states. EDMA cannot overwrite a buffer still being
  read. Frame descriptors belong to a particular frame/slot generation, not
  mutable global settings for the next frame.
- **Bounds:** distinguish the 128-bin temporary acquisition frame from narrower
  retained windows. Existing 64-bin arrays/bitmasks are not a safe representation
  of the complete search range; verify every consumer and reject unsupported layouts.
- **Selection:** use bounded-cost range-power extraction, a small candidate set,
  motion consistency and a noise estimate. Do not run the existing whole-capture
  iterative track fitter on every frame. Start with at most two candidates and
  one contiguous retained interval spanning the accepted evidence and margins.
- **Evidence:** preserve IQ16 values without additional quantization, all loops,
  physical TX/RX identity, frame sequence, device timing, bin origin/count, FFT
  scaling and clipping status. Preserve noise measurements independently of a
  narrow selected window. Missing data must never masquerade as measured zeros.
- **Timing:** distinguish frame interval, chirp interval, same-TX loop interval,
  device trigger time and host receipt time. Attach trigger identity to OPS,
  camera and IWR evidence; do not derive sub-frame timing from UART arrival.
- **Compatibility:** prefer the existing timed contiguous-window capture format.
  Add versioned diagnostic metadata only where the current contract cannot
  express required quality/timing facts. Keep old captures and profiles readable.
- **Serial lifecycle:** preserve the single reader/worker, pending trigger
  notices, startup ordering, release retries and full-history rearm safeguards.
  No per-frame blocking UART writes in the acquisition path.
- **Failure behavior:** bound candidate counts, compute time, total retained
  bytes and recovery attempts. On ambiguity, widen only within an explicit
  budget; otherwise terminate with an incomplete/uncertain status. Never publish
  overwritten, silently truncated or stale samples as a valid complete shot.
- **Processing placement:** HWA handles range FFTs; R4F handles lightweight
  selection and lifecycle initially. Heavy measurement calculations remain on
  the Pi. Evaluate the DSP only if measured R4F deadlines fail; DSP enablement
  requires a separate memory/build assessment.
- **Rollout:** explicit experimental capture mode until the validation gates pass.
  Trigger logic changes are not bundled into the selection algorithm.

## Phase 1: Prove the 2 ms IQ16 acquisition and timing contract

**User stories:** U1, U3, U4.

### What to build

An opt-in 2 ms IQ16 diagnostic capture that travels from firmware through the
existing serial reader into a saved, decoded host capture. Initially use a shorter
full-window recording that fits existing memory. Label its duration accurately;
it is a benchmark, not the final 72 ms feature.

Expose device-side frame counts, completion/rearm timestamps or cycle counters,
deadline misses, queue/buffer occupancy and high-water marks in capture diagnostics.
Resolve physical chirp/loop timing consistently across onboard and host processing.
Explicitly reject the unsupported IQ8 self-trigger combination rather than treating
it as verified.

### Acceptance criteria

- [ ] Profile geometry and timing tests establish 2,000 microseconds/frame,
  45 microseconds/chirp and 135 microseconds/same-TX loop for the target profile.
- [ ] Host replay and onboard tracking consume the same physical timing, including
  captures projected onto fewer TX channels.
- [ ] A real-device run of at least 100,000 frames records no new frame/rearm/DMA
  errors; report maxima and high-water marks, not just average processing time.
- [ ] Any observed gaps remain explicit and invalidate a nominally complete capture.
- [ ] At least 100 repeatable forced capture/rearm cycles finish without locking.
  Forced triggers validate transport/lifecycle only, not golf-shot detection.
- [ ] Saved records remain correctly associated with their OPS/camera trigger.

## Phase 2: Prove lossless compaction during acquisition

**User stories:** U1, U2, U3, U4.

### What to build

A complete acquisition-to-replay path using full-range IQ16 temporary buffers and
fixed, known retained windows. Use a reduced-duration test capture initially so
this step fits while buffer ownership and storage partitioning are established.
Rearm into the alternate buffer before compacting the completed frame.

Record the selected windows and exact IQ values through the existing timed capture
format, preserving all physical TX/RX channels. Selection decisions are fixed in
this step so corruption and scheduling failures cannot be confused with tracker errors.

### Acceptance criteria

- [ ] Synthetic patterns verify exact I/Q values, component order, all channels,
  all loops and descriptor offsets after compaction and host decoding.
- [ ] Tests cover wrapping prehistory, capture boundaries, two buffers ready at
  once, cancellation, malformed/truncated transfer and restart after failure.
- [ ] Allocation tests and the linker map prove scratch, capture, metadata,
  workspace and stacks fit with an explicit margin.
- [ ] Deliberately delayed processing causes a reported overrun/incomplete capture,
  never an overwritten record presented as valid.
- [ ] On hardware, full-range staging and compaction sustain the Phase 1 cadence
  test. Rearm meets the measured short deadline; processing finishes before reuse.

## Phase 3: Evaluate a live selector without discarding its reference data

**User stories:** U2, U4.

### What to build

Run the bounded selector on completed frames, recording its candidate ranges,
proposed retained windows, confidence, noise estimate and execution cost. Keep
reference samples in the shorter diagnostic captures so proposed decisions can
be checked against data that the selector would otherwise discard.

Replay the same frames through a Python reference and native C implementation.
Associate candidates over time instead of following whichever bin is loudest.
Use the known tee/net geometry and broad impact context; avoid committing to a
single ball candidate before club and ball can be distinguished.

### Acceptance criteria

- [ ] Python and C agree on deterministic selection vectors and boundary cases.
- [ ] Evidence tests include club/ball overlap, hands/body returns, stronger
  distractors, reversals, bin-edge crossing, ties, saturation, missing returns,
  net approach and targets entering/leaving the sampled range.
- [ ] Every accepted selection retains the reference target plus its declared
  uncertainty margin; misses/ambiguities are counted, not removed from evaluation.
- [ ] Measured CPU/EDMA behavior passes the 2 ms deadline tests with selection active.
- [ ] Real-shot evidence establishes whether the provisional 32/53/12-bin windows
  and trigger-relative split preserve the required club and ball tracks.
- [ ] Shadow testing on shorter captures is not described as proof that every part
  of a complete 72 ms record is preserved. Validate the remaining window in Phase 5.

## Phase 4: Deliver the complete 36-frame adaptive IQ16 capture

**User stories:** U1, U2, U3, U4.

### What to build

Enable selection to control storage and deliver the full nominal 72 ms capture
through the application. Start from the checked budget above, adjusting it only
with evidence from earlier phases. Maintain circular tee/club history, broad early
post-trigger evidence and tracked flight windows. End early with an explicit
reason when range, net or track loss makes further useful observation unavailable.

Keep reduced transfer and full retained-capture diagnostics distinguishable. Route
new acquisition records through a channel-preserving path rather than silently
projecting them onto the old vertical-only sparse format. Preserve the existing
OPS/camera coordination and trigger-lock safeguards.

### Acceptance criteria

- [ ] Under valid tracking, 36 correctly timed IQ16 frames fit within the enforced
  budget with all 3 TX, 4 RX and 12 loops retained for each selected bin.
- [ ] Dense impact evidence is not overwritten by later frames or by an early rearm.
- [ ] No frame decimation, hidden channel reduction or IQ8 conversion satisfies
  the 2 ms IQ16 target.
- [ ] Low-confidence selection cannot exhaust the arena or silently narrow away
  a plausible target. Budget exhaustion yields an explicit incomplete result.
- [ ] Host replay and live measurements use identical retained data and timing.
- [ ] Repeated accepted/rejected triggers, shutdown, UART stalls and configuration
  changes preserve one owner and recover without requiring an extra trigger.
- [ ] Report retained payload size, actual transfer latency, processing latency
  and time until rearmed separately.

## Phase 5: Qualify the mode for range use

**User stories:** U1–U4.

### What to build

A recorded comparison of the new mode against the established IQ16 baseline and
an external launch monitor where available. Keep raw retained evidence, selection
diagnostics and OPS/camera timing so failures remain investigable. Promote the
mode only after agreed acceptance thresholds pass; retain an explicit rollback.

### Acceptance criteria

- [ ] At least 100 paired reference shots spanning driver, iron and wedge, with
  club/ball speed, launch angles, club path, trigger failures and missing-result
  rates reported separately. Set measurement non-inferiority margins before the
  data collection using the baseline's measured repeatability.
- [ ] Include mis-hits and ordinary setup movement; do not count only successful shots.
- [ ] Run a 30-minute capture/processing soak and at least 100 repeated capture
  cycles with no deadlocks or unexplained sample loss. Report induced failures too.
- [ ] Verify full pre-trigger history, actual impact coverage and useful post-impact
  frame count across speeds and tee placements.
- [ ] Demonstrate more useful IQ16 frame observations in overlapping valid flight
  intervals; do not claim that the changed profile necessarily has identical shots
  or that 36 retained frames all contain useful ball returns.
- [ ] Save firmware/profile identifiers, checksums, memory map and timing results
  with the test report before changing the default.
- [ ] At-home motion tests can establish timing, transport and rearming. They do
  not qualify golf-ball selection or launch/club measurement accuracy.

## Decision gates and fallback

1. **Cadence fails:** first move copying off the rearm path and measure DMA/CPU
   contention. A bounded smaller acquisition search window is a separate tradeoff
   that requires coverage tests. If lightweight R4F selection still cannot keep
   up, assess DSP offload before changing the required cadence, precision or channels.
2. **Selection loses evidence:** increase the retained window or revise the split
   within an explicitly recalculated budget. Do not ship the nominal 72 ms target
   by sacrificing required impact data without surfacing the tradeoff.
3. **Transfer remains slow:** optimize the validated output separately. Onboard
   final-angle/spin calculation is a later feature, not a prerequisite or an
   implicit substitute for capturing the evidence.

## Evidence and remaining uncertainty

Checked against the baseline's capture geometry, arena allocation, HWA/EDMA flow,
packet descriptors, onboard track flow, loop-timing configuration and existing
firmware development notes. The memory and nominal timing calculations above were
recomputed during planning. Previously documented IQ8 cadence measurements are
not proof of IQ16 live-selection throughput.

Hardware capabilities are described in the [TI IWR6843 datasheet](https://www.ti.com/lit/ds/symlink/iwr6843.pdf).
This plan targets IWR6843 with the existing mmWave SDK, not IWRL6843 or its different
SDK. Actual selection cost, sample-retention quality and new-mode hardware timing
remain unmeasured. No firmware or application code was changed to produce this plan.
