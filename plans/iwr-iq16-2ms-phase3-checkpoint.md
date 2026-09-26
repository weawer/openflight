# Phase 3 checkpoint: IWR6843 shadow selector

Date: 2026-09-26

This checkpoint records the last hardware evidence before the adaptive selector
is allowed to control retained IQ16 storage in Phase 4. The test kept all 128
range bins, paired each frame with the firmware's proposed 12-bin window, and
therefore allowed every proposal to be checked against data it would discard.

## Reproduction identifiers

- Firmware: `firmware/releases/l3_dump_2ms_iq16_shadow_association_20260926.bin`
- Firmware SHA-256: `8930da7f7e66722acb3cd750eae37595802fa37dd7eb23a2866b2d50e0f4bf99`
- Configuration: `config/iwr6843_l3dump_shadow_reference_7f2ms_128bin_iq16.cfg`
- Run log: `openflight_sessions/iwr-shadow-association/iwr-shadow-association.jsonl`
- Paired reference capture: `openflight_sessions/iwr-shadow-association/shadow-reference-001.l3dump`
- Reference capture size: 516,168 bytes

The hardware command was:

```bash
uv run python scripts/hardware-test/check_iwr_2ms_iq16.py \
  --config config/iwr6843_l3dump_shadow_reference_7f2ms_128bin_iq16.cfg \
  --shadow \
  --soak-frames 1000 \
  --poll-s 2 \
  --cycles 1 \
  --capture-dir openflight_sessions/iwr-shadow-association \
  --output openflight_sessions/iwr-shadow-association/iwr-shadow-association.jsonl
```

## Observed result

The device completed 1,000 soak frames and one capture/rearm cycle. It reported
no RF, HWA, rearm, freeze, compaction, selector, or incomplete-capture errors.
The maximum measured times were 40 microseconds for rearming, 537 microseconds
for IQ16 compaction, and 773 microseconds for selection. Selection plus
compaction therefore took at most 1,310 microseconds, leaving 690 microseconds
inside the 2,000-microsecond frame period. Reported device temperatures were
45–49 degrees Celsius.

Offline decoding of the paired full-range capture reproduced the firmware's
candidate bins and noise values exactly for frames 1–6. Frame 0 depends on the
preceding live frame, which is intentionally outside the frozen seven-frame
reference capture. The selected-bin sequence was:

```text
12, 12, 15, 17, 18, 18, 18
```

The strongest motion candidate moved back to bin 13 in frames 5 and 6. The
association rule rejected that reversal and held the prior selection at bin 18,
but its proposed window covered bins 12–23 and retained bin 13. Across all seven
frames, every strongest motion candidate was inside the proposed retained
window.

## Scope of this evidence

This was an indoor moving-object test. It validates frame/decision association,
IQ16 integrity, proposed-window coverage for this capture, timing headroom, and
capture/rearm operation. It does not validate golf-ball identification, club and
ball separation, launch-angle accuracy, or preservation across the complete
36-frame record. Those remain Phase 4 and Phase 5 requirements.
