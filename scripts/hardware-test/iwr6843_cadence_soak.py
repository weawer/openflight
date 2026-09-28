#!/usr/bin/env python3
"""Cadence acceptance soak for the IWR6843 capture path.

Runs the sensor for a fixed number of frames and fails if the firmware
reports dropped frame starts (`hwa_missed`), IQ8 pack overruns
(`iq8_overrun`), EDMA errors (`iq8_edma_err`), or a detect-task frame drop
(`scratch_stale`, compact16/adaptive16 only) above the stated bound.
This is the acceptance gate for moving the IQ16 scratch buffer out of L3
into DATA_RAM — memory the CPU also uses — proving the relocation did not
blow the ~380 us inter-frame budget on real silicon. Nothing else in this
plan validates that on hardware; a failure here means revert to the L3
fallback rather than tuning around it.

scratch_stale specifically is the ball tracker / club track / trigger's
detect task losing the race against the HWA reusing its scratch buffer
(l3_dump.c, l3_detectFrameStale) — the risk this whole pipeline carries at
a shorter frame period, since it is unmeasured even at 3 ms. A nonzero
count here is not a timing near-miss to tune around; it means frames were
silently dropped from tracking.

Usage:
    uv run python scripts/hardware-test/iwr6843_cadence_soak.py \\
        --config config/iwr6843_l3dump_dense_51f2ms_53bin_iq8.cfg \\
        --frames 50000
    uv run python scripts/hardware-test/iwr6843_cadence_soak.py \\
        --config config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg \\
        --frames 50000
"""

from __future__ import annotations

import argparse
import sys
import time

sys.path.insert(0, "src")

from openflight.iwr6843.driver import IWR6843Radar  # noqa: E402
from openflight.iwr6843.firmware_checks import parse_stats  # noqa: E402

# Recorded baseline for the shipped wide/iq16 profile: a 0.0089% HWA
# miss rate over a long run. The relocation must not make this materially
# worse. Doubling the baseline is a materiality band, not measurement
# noise tolerance -- the DMA/CPU contention this soak exists to catch would
# blow well past 2x, not sit just above the baseline.
BASELINE_MISS_RATE = 0.000089  # 0.0089%, as a fraction
MAX_MISS_RATE = BASELINE_MISS_RATE * 2

# The soak must capture most of the requested frames to be a meaningful
# sample; a firmware wedge or early sensorStop failure would otherwise pass
# trivially with a near-zero denominator.
MIN_FRAME_COVERAGE = 0.9

# Exact firmware field names, from the `stats` CLI handler
# (firmware/iwr6843/l3_dump.c, l3_cli_stats): hwa_missed counts dropped
# HWA frame starts; iq8_overrun and iq8_edma_err are always emitted by this
# build (L3_RING_IQ8 and L3_IQ8_EDMA_PACK are unconditional Makefile
# defines), so both the wide/iq16 and dense/iq8 profiles report them even
# though only the iq8 profile actually exercises the pack/EDMA path.
REQUIRED_STAT_FIELDS = ("hwa_frames", "hwa_missed", "iq8_overrun", "iq8_edma_err")

# frameCfg <chirpStart> <chirpEnd> <numLoops> <numFrames> <periodicity_ms> ...
_FRAME_CFG_PERIOD_INDEX = 5


def rearm_summary(stats: dict[str, int], period_s: float) -> str | None:
    """Queue-to-rearm latency against the frame period, or None on older firmware.

    Reported, not judged: no deadline has been measured yet, and a late
    rearm already shows up as ``hwa_missed``.
    """
    if "rearm_max_us" not in stats:
        return None
    period_us = period_s * 1e6
    share = stats["rearm_max_us"] / period_us
    return (
        f"rearm_last_us={stats['rearm_last_us']} rearm_max_us={stats['rearm_max_us']} "
        f"({share:.1%} of the {period_us:.0f} us frame) timed={stats['rearm_timed']}"
    )


def frame_period_s(cfg_path: str) -> float:
    """Read the frameCfg periodicity (ms) out of a .cfg file.

    Profiles differ (3 ms for the wide/iq16 default, 2 ms for the dense/iq8
    profile), so this is read from the config rather than assumed.
    """
    with open(cfg_path, encoding="utf-8") as cfg:
        for rawline in cfg:
            tokens = rawline.split()
            if tokens and tokens[0] == "frameCfg":
                return float(tokens[_FRAME_CFG_PERIOD_INDEX]) / 1000.0
    raise ValueError(f"no frameCfg line found in {cfg_path}")


def evaluate(
    stats: dict[str, int],
    *,
    args_frames: int,
    period_s: float = 0.0,
    rate_cap: float = MAX_MISS_RATE,
    coverage_min: float = MIN_FRAME_COVERAGE,
) -> tuple[bool, list[str]]:
    """Pass/fail verdict and report lines from one soak's parsed ``stats``.

    Pure, so the gating logic is testable without hardware. ``stats`` must
    already have passed the ``REQUIRED_STAT_FIELDS`` presence check in
    ``main`` -- this only reports the optional ``rearm_*``/``scratch_stale``
    fields when present. ``period_s`` is only used to express rearm_max_us
    as a share of the frame period; omit it (or the rearm fields) and that
    line just says latency wasn't reported.
    """
    lines: list[str] = []
    frames = stats["hwa_frames"]
    missed = stats["hwa_missed"]
    rate = missed / frames if frames else 1.0
    lines.append(
        f"frames={frames} missed={missed} rate={rate:.6%} "
        f"iq8_overrun={stats['iq8_overrun']} iq8_edma_err={stats['iq8_edma_err']}"
    )

    lines.append(rearm_summary(stats, period_s) or "rearm latency: not reported by this firmware")

    if "scratch_stale" in stats:
        lines.append(f"scratch_stale={stats['scratch_stale']}")
    else:
        lines.append("scratch_stale: not reported (not a compact16/adaptive16 profile)")

    ok = True
    if frames < args_frames * coverage_min:
        lines.append(f"FAIL: only {frames} frames captured, expected ~{args_frames}")
        ok = False
    if rate > rate_cap:
        lines.append(f"FAIL: miss rate {rate:.6%} exceeds {rate_cap:.6%}")
        ok = False
    if stats["iq8_overrun"]:
        lines.append(f"FAIL: {stats['iq8_overrun']} IQ8 pack overrun(s)")
        ok = False
    if stats["iq8_edma_err"]:
        lines.append(f"FAIL: {stats['iq8_edma_err']} IQ8 EDMA error(s)")
        ok = False
    if stats.get("scratch_stale"):
        lines.append(
            f"FAIL: {stats['scratch_stale']} detect-task frame(s) dropped "
            "(scratch reused before the ball tracker / trigger read it)"
        )
        ok = False

    lines.append("PASS" if ok else "FAILURES ABOVE")
    return ok, lines


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 2)[1])
    parser.add_argument("--config", required=True, help="path to an IWR6843 .cfg profile")
    parser.add_argument("--frames", type=int, default=50_000, help="frames to soak")
    parser.add_argument("--port", default=None, help="serial port (default: auto-detect)")
    args = parser.parse_args()

    period_s = frame_period_s(args.config)
    target_s = args.frames * period_s
    print(
        f"soaking ~{args.frames} frames of {args.config} "
        f"({period_s * 1000:.1f} ms/frame, ~{target_s:.0f}s)"
    )

    with IWR6843Radar(port=args.port) as radar:
        print(f"IWR6843 on {radar.port}")
        radar.send_config(args.config)
        time.sleep(target_s)
        stats_text = radar.stats()
        radar.stop_sensor()

    stats = parse_stats(stats_text)

    for field in REQUIRED_STAT_FIELDS:
        if field not in stats:
            print(f"FAIL: firmware stats did not report {field!r}: {stats_text.strip()!r}")
            return 1

    ok, lines = evaluate(stats, args_frames=args.frames, period_s=period_s)
    for line in lines:
        print(line)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
