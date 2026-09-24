#!/usr/bin/env python3
"""Compare J3 pin 2 alert timing with an internal-trigger UART dump.

Stop the kiosk before running. Connect OPS J3 pin 2 to Pi BCM17 (physical
pin 11), with a shared ground. GPIO is input-only. No flash settings are saved.
Stand still during setup; move only after the ready event. Each internal run
collects one dump. Use --mode speed as a wiring/alert positive control.
"""

import argparse
import json
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))

from openflight.gpio_factory import ensure_lgpio_pin_factory  # noqa: E402
from openflight.ops243 import OPS243Radar  # noqa: E402
from openflight.rolling_buffer.processor import RollingBufferProcessor  # noqa: E402


def run_test(radar, gpio, mode, duration, emit):
    """Log setup edges separately from the armed observation window."""
    phase = "setup"
    gpio.when_pressed = lambda: emit("alert_falling", phase=phase)
    gpio.when_released = lambda: emit("alert_rising", phase=phase)
    if mode == "internal":
        radar.configure_for_internal_speed_trigger(
            trigger_threshold_mph=5, trigger_magnitude=6, pre_trigger_segments=6
        )
    else:
        radar.configure_for_speed_trigger()

    # Set the alert after GC, which can reset detector settings.
    emit("alert_setting", response=radar._send_command("Y<40"))
    emit("alert_readback", response=radar._send_command("Y?"))
    phase = "observe"
    emit("ready", mode=mode, alert_active=bool(gpio.is_pressed), duration_s=duration)
    if mode == "speed":
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            waiting = radar.serial.in_waiting
            if waiting:
                emit("uart", text=radar.serial.read(waiting).decode("ascii", errors="replace"))
            time.sleep(0.01)
        emit("finished")
        return

    response = radar.wait_for_hardware_trigger(
        timeout=duration,
        on_first_byte=lambda: emit("dump_start"),
    )
    emit("dump", response=response)
    if not response:
        emit("no_dump")
        return
    capture = RollingBufferProcessor().parse_capture(
        response, first_byte_timestamp=radar.last_hardware_trigger_first_byte_timestamp
    )
    if capture is None:
        emit("parse_failed")
        return
    emit(
        "capture_timing",
        first_byte_timestamp=radar.last_hardware_trigger_first_byte_timestamp,
        sample_time=capture.sample_time,
        trigger_time=capture.trigger_time,
        post_trigger_ms=capture.post_trigger_duration_ms,
        samples=len(capture.i_samples),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyAMA0")
    parser.add_argument("--gpio", type=int, default=17, help="BCM pin number")
    parser.add_argument("--mode", choices=["internal", "speed"], default="internal")
    parser.add_argument("--duration", type=float, default=60)
    parser.add_argument("--output", type=Path, required=True, help="New JSONL evidence file")
    args = parser.parse_args()
    if not 0 < args.duration <= 600:
        parser.error("--duration must be greater than 0 and at most 600 seconds")

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
                print(json.dumps({k: v for k, v in record.items() if k != "response"}), flush=True)

        radar = OPS243Radar(port=args.port)
        try:
            with Button(args.gpio, pull_up=True, bounce_time=None) as gpio:
                radar.connect()
                emit("connected", port=args.port, firmware=radar.get_firmware_version())
                run_test(radar, gpio, args.mode, args.duration, emit)
        finally:
            radar.disconnect()


if __name__ == "__main__":
    main()
