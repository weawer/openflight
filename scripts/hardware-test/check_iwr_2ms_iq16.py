#!/usr/bin/env python3
"""Validate a 2 ms IQ16 profile on a connected IWR6843."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, "src")

from openflight.iwr6843.driver import IWR6843Radar  # noqa: E402
from openflight.iwr6843.dump import (  # noqa: E402
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    parse_dump,
)

DEFAULT_CONFIG = "config/iwr6843_l3dump_diagnostic_24f2ms_53bin_iq16.cfg"
ERROR_FIELDS = (
    "rf_faults",
    "hwa_rearm_err",
    "hwa_missed",
    "freeze_to",
    "iq8_overrun",
    "iq8_edma_err",
    "compact16_err",
    "shadow_err",
    "incomplete",
)


def parse_capture_stats(response: str) -> dict[str, int | str]:
    """Parse firmware ``stats`` key/value fields."""
    parsed: dict[str, int | str] = {}
    for token in response.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        try:
            parsed[key] = int(value, 0)
        except ValueError:
            parsed[key] = value
    return parsed


def _numeric(stats: dict[str, int | str], name: str) -> int:
    value = stats.get(name, 0)
    if not isinstance(value, int):
        raise RuntimeError(f"stats field {name} is not numeric: {value!r}")
    return value


def _health(radar: IWR6843Radar) -> dict[str, int | str]:
    response = radar.stats()
    if "Done" not in response or "Error" in response:
        raise RuntimeError(f"stats failed: {response.strip()}")
    return parse_capture_stats(response)


def _check_errors(stats: dict[str, int | str], baseline: dict[str, int | str]) -> None:
    changed = {
        field: (_numeric(baseline, field), _numeric(stats, field))
        for field in ERROR_FIELDS
        if _numeric(stats, field) != _numeric(baseline, field)
    }
    if changed:
        raise RuntimeError(f"capture error counters changed: {changed}")


def _check_timing(current, expected_period_us, compact_mode):
    if compact_mode and _numeric(current, "compact16_max_us") >= expected_period_us:
        raise RuntimeError(
            "compaction exceeded the frame period: "
            f"{_numeric(current, 'compact16_max_us')} >= {expected_period_us} us"
        )
    if compact_mode and (
        _numeric(current, "shadow_max_us") + _numeric(current, "compact16_max_us")
        >= expected_period_us
    ):
        raise RuntimeError(
            "selector plus compaction exceeded the frame period: "
            f"{_numeric(current, 'shadow_max_us')} + "
            f"{_numeric(current, 'compact16_max_us')} >= "
            f"{expected_period_us} us"
        )


def _write_event(output, event: str, **fields) -> None:
    if output is None:
        return
    output.write(json.dumps({"event": event, "ts": time.time(), **fields}) + "\n")
    output.flush()


def _expected_geometry(config_path: str) -> tuple[int, int]:
    commands = {
        fields[0]: fields
        for line in Path(config_path).read_text(encoding="utf-8").splitlines()
        if (fields := line.split()) and not fields[0].startswith("%")
    }
    frame_period_us = int(float(commands["frameCfg"][5]) * 1000)
    phase = commands["phaseCaptureCfg"]
    return int(phase[3]) + int(phase[6]) + int(phase[10]), frame_period_us


def _phase_capture_cfg(config_path: str) -> list[int]:
    return next(
        list(map(int, line.split()[1:]))
        for line in Path(config_path).read_text(encoding="utf-8").splitlines()
        if line.startswith("phaseCaptureCfg ")
    )


def flight_frames_kept(metadata: dict, config_path: str) -> int:
    """Number of retained frames beyond the fixed pre-trigger/impact windows.

    These are the frames the selector chose to keep, as opposed to the fixed
    windows every capture retains regardless of tracking. Any positive count
    here means a track was accepted (or coasted) long enough to keep at least
    one selector-controlled window; a static scene must report zero.
    """
    if "retention" not in metadata:
        return 0
    phase = _phase_capture_cfg(config_path)
    pre, impact = phase[2], phase[5]
    return max(0, metadata["n_frames"] - pre - impact)


def _check_retention_layout(metadata, decisions, config_path):
    if "retention" not in metadata:
        return
    phase = _phase_capture_cfg(config_path)
    pre, impact, flight = phase[2], phase[5], phase[9]
    counts = [phase[1]] * pre + [phase[4]] * impact + [phase[7]] * flight
    fixed_starts = [phase[0]] * pre + [phase[3]] * impact
    if metadata["retention"]["pre_frames"] != pre:
        raise RuntimeError("adaptive capture did not retain its complete pre-trigger history")
    if (metadata["chirps_per_frame"], metadata["n_tx"], metadata["n_rx"]) != (36, 3, 4):
        raise RuntimeError("adaptive capture lost loops or antenna channels")
    for frame, (start, count) in enumerate(
        zip(metadata["range_bin_starts"], metadata["range_bin_counts"], strict=True)
    ):
        if count != counts[frame] or (frame < pre + impact and start != fixed_starts[frame]):
            raise RuntimeError(f"frame {frame}: unexpected adaptive storage window")
        if frame >= pre + impact and decisions:
            decision = decisions[frame]
            tracked = decision["accepted"] or decision.get("coasting", 0)
            if not tracked or (start, count) != (
                decision["proposed_start"],
                decision["proposed_bins"],
            ):
                raise RuntimeError(f"frame {frame}: stored window differs from selector decision")


def run(args: argparse.Namespace) -> None:
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    output = Path(args.output).open("w", encoding="utf-8") if args.output else None
    capture_dir = Path(args.capture_dir) if args.capture_dir else None
    if capture_dir is not None:
        capture_dir.mkdir(parents=True, exist_ok=True)
    radar = IWR6843Radar(args.port)
    configured = False
    try:
        radar.send_config(args.config)
        configured = True
        baseline = _health(radar)
        expected_frames, expected_period_us = _expected_geometry(args.config)
        compact_mode = baseline.get("format") in ("compact16", "adaptive16")
        start_frames = _numeric(baseline, "frames")
        target = start_frames + args.soak_frames
        _write_event(output, "start", config=args.config, stats=baseline)

        while _numeric(baseline, "frames") < target:
            time.sleep(min(args.poll_s, 60.0))
            current = _health(radar)
            _check_errors(current, baseline)
            _check_timing(current, expected_period_us, compact_mode)
            baseline = current
            done = _numeric(current, "frames") - start_frames
            print(
                f"frames {done}/{args.soak_frames}, "
                f"rearm max {_numeric(current, 'rearm_max_us')} us, "
                f"compact max {_numeric(current, 'compact16_max_us')} us"
                f", selector max {_numeric(current, 'shadow_max_us')} us"
            )
            _write_event(output, "stats", stats=current)

        for cycle in range(1, args.cycles + 1):
            capture_start = time.monotonic()
            if args.shadow:
                raw, decisions = radar.read_shadow_dump()
            else:
                raw = radar.read_dump()
                decisions = []
            capture_roundtrip_s = time.monotonic() - capture_start
            capture_path = None
            if capture_dir is not None:
                capture_path = capture_dir / f"shadow-reference-{cycle:03d}.l3dump"
                capture_path.write_bytes(raw)
            _write_event(
                output,
                "capture_received",
                cycle=cycle,
                shadow_decisions=decisions,
                capture_path=str(capture_path) if capture_path else None,
            )
            metadata, _cube = parse_dump(raw)
            retention = metadata.get("retention")
            stopped = retention is not None and retention["reason"] != "complete"
            if stopped and not args.allow_early_stop:
                raise RuntimeError(f"cycle {cycle}: retention stopped: {retention['reason']}")
            if retention and retention["planned_frames"] != expected_frames:
                raise RuntimeError(f"cycle {cycle}: unexpected retention plan {retention}")
            if (metadata["n_frames"] != expected_frames and not stopped) or metadata[
                "frame_period_us"
            ] != expected_period_us:
                raise RuntimeError(f"cycle {cycle}: unexpected geometry {metadata}")
            if metadata["sample_fmt"] != SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED:
                raise RuntimeError(f"cycle {cycle}: capture is not timed IQ16")
            if args.shadow and len(decisions) != metadata["n_frames"]:
                raise RuntimeError(
                    f"cycle {cycle}: got {len(decisions)} shadow decisions for "
                    f"{metadata['n_frames']} frames"
                )
            if any(
                (decision["accepted"] or decision.get("coasting", 0))
                and not (
                    decision["proposed_start"]
                    <= decision["selected"]
                    < decision["proposed_start"] + decision["proposed_bins"]
                )
                for decision in decisions
            ):
                raise RuntimeError(f"cycle {cycle}: proposed window excludes its candidate")
            _check_retention_layout(metadata, decisions, args.config)
            kept_flight_frames = flight_frames_kept(metadata, args.config)
            if args.expect_no_flight_frames and kept_flight_frames > 0:
                raise RuntimeError(
                    f"cycle {cycle}: expected a static scene but retained "
                    f"{kept_flight_frames} flight frame(s); a track was accepted "
                    "or coasted from something other than trigger noise"
                )
            offsets = metadata.get("frame_time_offsets_us")
            if offsets is not None and any(
                later - earlier != expected_period_us
                for earlier, later in zip(offsets, offsets[1:])
            ):
                raise RuntimeError(f"cycle {cycle}: explicit frame gap in {offsets}")
            current = _health(radar)
            _check_errors(current, baseline)
            _check_timing(current, expected_period_us, compact_mode)
            baseline = current
            outcome = f"early stop: {retention['reason']}" if stopped else "complete"
            print(
                f"capture cycle {cycle}/{args.cycles} passed ({outcome}, "
                f"{kept_flight_frames} flight frame(s) kept)"
            )
            _write_event(
                output,
                "capture",
                cycle=cycle,
                metadata=metadata,
                flight_frames_kept=kept_flight_frames,
                payload_bytes=len(raw),
                capture_roundtrip_s=capture_roundtrip_s,
                shadow_decisions=decisions,
                capture_path=str(capture_path) if capture_path else None,
                stats=current,
            )

        print(
            f"PASS: {args.soak_frames} frames, {args.cycles} capture cycles, "
            f"maximum rearm latency {_numeric(baseline, 'rearm_max_us')} us, "
            f"maximum compaction latency "
            f"{_numeric(baseline, 'compact16_max_us')} us"
            f", maximum selector latency {_numeric(baseline, 'shadow_max_us')} us"
        )
    finally:
        try:
            if configured:
                radar.stop_sensor()
        finally:
            radar.close()
            if output is not None:
                output.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--soak-frames", type=int, default=100_000)
    parser.add_argument("--cycles", type=int, default=0)
    parser.add_argument("--poll-s", type=float, default=10.0)
    parser.add_argument("--output")
    parser.add_argument(
        "--allow-early-stop",
        action="store_true",
        help="accept explicitly labelled adaptive track loss for indoor lifecycle tests",
    )
    parser.add_argument("--capture-dir", help="directory for raw .l3dump captures")
    parser.add_argument(
        "--expect-no-flight-frames",
        action="store_true",
        help="fail if any capture retains a selector-controlled flight frame "
        "(use for a static-scene run, where none should ever be confirmed)",
    )
    parser.add_argument(
        "--shadow",
        action="store_true",
        help="use l3shadow and save decisions paired with each capture",
    )
    args = parser.parse_args()
    if args.soak_frames < 0 or args.cycles < 0 or args.poll_s <= 0:
        parser.error("soak frames and cycles must be non-negative; poll interval must be positive")
    run(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
