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
    parse_header,
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


def run(args: argparse.Namespace) -> None:
    output = Path(args.output).open("w", encoding="utf-8") if args.output else None
    radar = IWR6843Radar(args.port)
    try:
        radar.send_config(args.config)
        baseline = _health(radar)
        expected_frames, expected_period_us = _expected_geometry(args.config)
        compact_mode = baseline.get("format") == "compact16"
        start_frames = _numeric(baseline, "frames")
        target = start_frames + args.soak_frames
        _write_event(output, "start", config=args.config, stats=baseline)

        while _numeric(baseline, "frames") < target:
            time.sleep(min(args.poll_s, 60.0))
            current = _health(radar)
            _check_errors(current, baseline)
            if compact_mode and _numeric(current, "compact16_max_us") >= expected_period_us:
                raise RuntimeError(
                    "compaction exceeded the frame period: "
                    f"{_numeric(current, 'compact16_max_us')} >= {expected_period_us} us"
                )
            if compact_mode and (
                _numeric(current, "shadow_max_us")
                + _numeric(current, "compact16_max_us")
                >= expected_period_us
            ):
                raise RuntimeError(
                    "selector plus compaction exceeded the frame period: "
                    f"{_numeric(current, 'shadow_max_us')} + "
                    f"{_numeric(current, 'compact16_max_us')} >= "
                    f"{expected_period_us} us"
                )
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
            raw = radar.read_dump()
            metadata = parse_header(raw)
            if (
                metadata["n_frames"] != expected_frames
                or metadata["frame_period_us"] != expected_period_us
            ):
                raise RuntimeError(f"cycle {cycle}: unexpected geometry {metadata}")
            if metadata["sample_fmt"] != SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED:
                raise RuntimeError(f"cycle {cycle}: capture is not timed IQ16")
            offsets = metadata.get("frame_time_offsets_us")
            if offsets is not None and any(
                later - earlier != expected_period_us
                for earlier, later in zip(offsets, offsets[1:])
            ):
                raise RuntimeError(f"cycle {cycle}: explicit frame gap in {offsets}")
            current = _health(radar)
            _check_errors(current, baseline)
            baseline = current
            print(f"capture cycle {cycle}/{args.cycles} passed")
            _write_event(output, "capture", cycle=cycle, metadata=metadata, stats=current)

        print(
            f"PASS: {args.soak_frames} frames, {args.cycles} capture cycles, "
            f"maximum rearm latency {_numeric(baseline, 'rearm_max_us')} us, "
            f"maximum compaction latency "
            f"{_numeric(baseline, 'compact16_max_us')} us"
            f", maximum selector latency {_numeric(baseline, 'shadow_max_us')} us"
        )
    finally:
        try:
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
    args = parser.parse_args()
    if args.soak_frames < 0 or args.cycles < 0 or args.poll_s <= 0:
        parser.error("soak frames and cycles must be non-negative; poll interval must be positive")
    run(args)
    return 0


if __name__ == "__main__":
    sys.exit(main())
