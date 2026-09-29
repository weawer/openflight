#!/usr/bin/env python3
"""Record live club-track diagnostics and self-triggered dumps. Stop the kiosk first.

Uses firmware triggerLog/trace/perf evidence, not the retired ball-leave replay.
Run with --tee-m set to the measured antenna-to-ball slant distance.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from openflight.iwr6843.calibration import DEFAULT_CAL_PATH, DEFAULT_TEE_RANGE_M, Calibration
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.firmware_checks import parse_snapshot, parse_stats, parse_trig
from openflight.iwr6843.monitor import SelfTriggerConfig, tee_global_bin

DEFAULT_CONFIG = "config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg"


def port_name_error(port: str | None, platform: str) -> str | None:
    if port and platform != "win32" and port.upper().startswith("COM") and port[3:].isdigit():
        return f"{port} is a Windows port name; leave --port off or use /dev/ttyUSB0."
    return None


def arm(radar, config: str, tee_bin: int, snr: float, frames: int) -> None:
    command = SelfTriggerConfig(tee_bin, snr, frames).command
    radar.send_config(config)
    reply = radar.cmd(command)
    if "Done" not in reply or "Error" in reply:
        raise RuntimeError(f"triggerCfg rejected: {reply.strip()}")


def record(output, event: str, **fields) -> None:
    output.write(json.dumps({"event": event, "ts": time.time(), **fields}) + "\n")
    output.flush()


def check_health(raw: str, *, rearmed: bool = False) -> None:
    if "Done" not in raw or "Error" in raw:
        raise RuntimeError(f"stats failed: {raw.strip()}")
    snapshot = parse_snapshot(raw)
    if snapshot.enabled != 1:
        raise RuntimeError("firmware self-trigger is disabled")
    counters = parse_stats(raw)
    required = ("scratch_stale", "hwa_missed", "iq8_overrun", "iq8_edma_err")
    for key in required:
        if key not in counters:
            raise RuntimeError(f"stats missing {key}")
        if counters[key]:
            raise RuntimeError(f"capture health failed: {key}={counters[key]}")
    for key in ("detect_dropped", "detect_stale"):
        value = getattr(snapshot, key)
        if value is None or value != 0:
            raise RuntimeError(f"capture health failed: {key}={value}")
    if rearmed and (snapshot.active != 1 or snapshot.latched != 0):
        raise RuntimeError("rearm not confirmed: expected active=1 latched=0 enabled=1")


def observe(radar, output, capture_dir: Path, poll_s: float) -> None:
    pending = b""
    next_poll = 0.0
    captures = 0
    while True:
        fired, pending = radar.wait_trigger_notice(pending)
        if not fired and time.monotonic() < next_poll:
            continue
        health = radar.stats()
        if "Done" not in health or "Error" in health:
            raise RuntimeError(f"stats failed: {health.strip()}")
        fields = next((p for line in health.splitlines() if (p := parse_trig(line))), None)
        if fields is None:
            raise RuntimeError("stats missing trigger state")
        fired = fired or fields.get("latched") == "1"
        # Polled CLI output preempts detection; only print traces on a frozen ring.
        logs = {}
        if fired and parse_snapshot(health).active == 0:
            logs = {
                command: radar.cmd(command)
                for command in ("triggerLog", "triggerLog trace", "triggerLog perf")
            }
        record(output, "trigger_diagnostics", stats=health, logs=logs, fired=fired)
        if logs:
            print(logs["triggerLog"], flush=True)
        check_health(health, rearmed=not fired)
        if fired:
            raw = radar.read_dump()
            try:
                metadata, _ = parse_dump(raw)
                if metadata["n_frames"] == 0:
                    raise ValueError("empty capture")
            except ValueError as exc:
                path = capture_dir / f"failed-{time.time_ns()}.invalid.bin"
                path.write_bytes(raw)
                record(output, "capture_failed", path=str(path), bytes=len(raw), error=str(exc))
                raise RuntimeError(f"invalid dump: {exc}; evidence saved to {path}") from exc
            captures += 1
            path = capture_dir / f"swing-{time.time_ns()}-{captures:03d}.l3dump"
            path.write_bytes(raw)
            record(output, "capture", path=str(path), bytes=len(raw))
            health = radar.stats()
            record(output, "rearm_check", stats=health)
            check_health(health, rearmed=True)
            record(output, "rearmed", capture=captures)
            print(f"Capture {captures}: valid dump saved to {path}; rearm confirmed", flush=True)
            pending = b""
        next_poll = time.monotonic() + poll_s


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--cal", default=DEFAULT_CAL_PATH)
    parser.add_argument("--tee-m", type=float, default=DEFAULT_TEE_RANGE_M)
    parser.add_argument("--snr", type=float, default=6.0)
    parser.add_argument("--frames", type=int, default=2)
    parser.add_argument("--poll-s", type=float, default=2.0)
    parser.add_argument("--capture-dir", type=Path, default=Path("openflight_sessions/iwr-trigger"))
    args = parser.parse_args()
    error = port_name_error(args.port, sys.platform)
    if error:
        parser.error(error)
    if args.poll_s <= 0:
        parser.error("--poll-s must be positive")
    tee_bin = tee_global_bin(
        args.tee_m, args.config, range_bias_m=Calibration.load(args.cal).range_bias_m
    )
    args.capture_dir.mkdir(parents=True, exist_ok=True)
    with (args.capture_dir / "run.jsonl").open("a", encoding="utf-8") as output:
        with IWR6843Radar(port=args.port) as radar:
            try:
                arm(radar, args.config, tee_bin, args.snr, args.frames)
                record(
                    output,
                    "armed",
                    config=args.config,
                    cal=args.cal,
                    tee_bin=tee_bin,
                    snr=args.snr,
                    frames=args.frames,
                )
                print(f"Armed global bin {tee_bin}, SNR {args.snr:g}. Ctrl+C to stop.", flush=True)
                observe(radar, output, args.capture_dir, args.poll_s)
            except KeyboardInterrupt:
                print("Stopped.")
            except Exception as exc:
                record(output, "error", error=str(exc))
                raise
            finally:
                radar.stop_sensor()


if __name__ == "__main__":
    main()
