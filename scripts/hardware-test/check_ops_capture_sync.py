#!/usr/bin/env python3
"""Observe the OPS243 J3 pin 1 data-capture synchronization signal.

Stop the kiosk before running. Connect OPS J3 pin 1 (GPIO_0) to the selected
Raspberry Pi BCM GPIO, with a shared ground. The Pi GPIO is input-only. This
diagnostic does not save settings to OPS flash.

AN-023-B describes J3 pin 1 as the ADC sampling indicator when the low-speed
alert is zero. The document describes conflicting signal polarity, so this
tool records both physical edges and the initial level. ``--mode speed`` is a
baseline test; ``--mode internal`` compares the signal with an ST-triggered
rolling-buffer dump.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT / "src"))

from openflight.gpio_factory import ensure_lgpio_pin_factory  # noqa: E402
from openflight.ops243 import OPS243Radar  # noqa: E402
from openflight.rolling_buffer.processor import RollingBufferProcessor  # noqa: E402


def run_test(
    *,
    radar,
    gpio,
    mode: str,
    duration: float,
    trigger_threshold: float,
    trigger_magnitude: int,
    pre_trigger_segments: int,
    emit,
) -> None:
    """Configure one mode and record J3 pin 1 edges and UART timing."""
    phase = "setup"
    gpio.when_pressed = lambda: emit("sync_falling", phase=phase)
    gpio.when_released = lambda: emit("sync_rising", phase=phase)

    if mode == "internal":
        radar.configure_for_internal_speed_trigger(
            trigger_threshold_mph=trigger_threshold,
            trigger_magnitude=trigger_magnitude,
            pre_trigger_segments=pre_trigger_segments,
        )
    else:
        radar.configure_for_speed_trigger()

    emit("low_alert_reset", response=radar._send_command("Y>0"))
    emit("alert_readback", response=radar._send_command("Y?"))
    phase = "observe"
    emit(
        "ready",
        mode=mode,
        pin_level="low" if gpio.is_pressed else "high",
        duration_s=duration,
        trigger_threshold_mph=trigger_threshold,
        trigger_magnitude=trigger_magnitude,
        pre_trigger_segments=pre_trigger_segments,
    )

    if mode == "speed":
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            waiting = radar.serial.in_waiting
            if waiting:
                emit("uart", text=radar.serial.read(waiting).decode("ascii", errors="replace"))
            time.sleep(0.01)
        emit("finished", pin_level="low" if gpio.is_pressed else "high")
        return

    response = radar.wait_for_hardware_trigger(
        timeout=duration,
        on_first_byte=lambda: emit(
            "dump_start",
            first_byte_timestamp=radar.last_hardware_trigger_first_byte_timestamp,
            pin_level="low" if gpio.is_pressed else "high",
        ),
    )
    if not response:
        emit("no_dump", pin_level="low" if gpio.is_pressed else "high")
        return

    capture = RollingBufferProcessor().parse_capture(
        response,
        first_byte_timestamp=radar.last_hardware_trigger_first_byte_timestamp,
    )
    if capture is None:
        emit("parse_failed", response_bytes=len(response))
        return
    emit(
        "capture_timing",
        first_byte_timestamp=radar.last_hardware_trigger_first_byte_timestamp,
        sample_time=capture.sample_time,
        trigger_time=capture.trigger_time,
        trigger_offset_ms=capture.trigger_offset_ms,
        post_trigger_ms=capture.post_trigger_duration_ms,
        samples=len(capture.i_samples),
        pin_level="low" if gpio.is_pressed else "high",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--gpio", type=int, default=17, help="Raspberry Pi BCM pin number")
    parser.add_argument("--mode", choices=["internal", "speed"], default="internal")
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--trigger-threshold", type=float, default=40.0)
    parser.add_argument("--trigger-magnitude", type=int, default=600)
    parser.add_argument("--pre-trigger-segments", type=int, default=8)
    parser.add_argument("--output", type=Path, required=True, help="New JSONL evidence file")
    args = parser.parse_args()
    if not 0 < args.duration <= 600:
        parser.error("--duration must be greater than 0 and at most 600 seconds")
    if args.trigger_threshold < 0:
        parser.error("--trigger-threshold must be non-negative")
    if not 1 <= args.trigger_magnitude <= 2000:
        parser.error("--trigger-magnitude must be between 1 and 2000")
    if not 0 <= args.pre_trigger_segments <= 32:
        parser.error("--pre-trigger-segments must be between 0 and 32")

    ensure_lgpio_pin_factory()
    from gpiozero import Button

    lock = threading.Lock()
    with args.output.open("x", encoding="utf-8") as output:

        def emit(event, **fields):
            record = {
                "event": event,
                "ts": time.time(),
                "monotonic_ns": time.monotonic_ns(),
                **fields,
            }
            with lock:
                output.write(json.dumps(record) + "\n")
                output.flush()
                visible = {key: value for key, value in record.items() if key != "response"}
                print(json.dumps(visible), flush=True)

        radar = OPS243Radar(port=args.port)
        try:
            with Button(args.gpio, pull_up=True, bounce_time=None) as gpio:
                radar.connect()
                emit("connected", port=args.port, firmware=radar.get_firmware_version())
                run_test(
                    radar=radar,
                    gpio=gpio,
                    mode=args.mode,
                    duration=args.duration,
                    trigger_threshold=args.trigger_threshold,
                    trigger_magnitude=args.trigger_magnitude,
                    pre_trigger_segments=args.pre_trigger_segments,
                    emit=emit,
                )
        finally:
            radar.disconnect()


if __name__ == "__main__":
    main()
