#!/usr/bin/env python3
"""Test whether an OPS UART dump can freeze the IWR6843 ring in time.

Stop the kiosk before running. This diagnostic configures both radars, waits
for one OPS internal-speed trigger, and requests the IWR6843 dump as soon as
the first OPS dump byte arrives. It saves both captures for offline review.
No settings are written to flash.
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

from openflight.iwr6843.monitor import IWR6843CaptureMonitor  # noqa: E402
from openflight.ops243 import OPS243Radar  # noqa: E402
from openflight.rolling_buffer.processor import RollingBufferProcessor  # noqa: E402

OPS_SEGMENTS = 32
OPS_SAMPLES_PER_SEGMENT = 128


def estimated_trigger_timestamp(first_byte_timestamp: float, pre_segments: int) -> float:
    """Infer the OPS trigger epoch from its fixed post-trigger sample count."""
    post_samples = (OPS_SEGMENTS - pre_segments) * OPS_SAMPLES_PER_SEGMENT
    return first_byte_timestamp - post_samples / 30_000


def run_capture(*, ops, iwr, processor, pre_segments: int, timeout_s: float, emit) -> bool:
    """Capture one synchronized OPS/IWR event and report whether both arrived."""
    state: dict[str, float | bool | None] = {
        "first_byte_timestamp": None,
        "trigger_timestamp": None,
        "iwr_trigger_accepted": False,
    }

    def on_first_byte() -> None:
        first_byte = ops.last_hardware_trigger_first_byte_timestamp or time.time()
        trigger_timestamp = estimated_trigger_timestamp(first_byte, pre_segments)
        state.update(
            first_byte_timestamp=first_byte,
            trigger_timestamp=trigger_timestamp,
            iwr_trigger_accepted=iwr.notify_trigger(trigger_timestamp),
        )
        emit(
            "ops_dump_start",
            first_byte_timestamp=first_byte,
            estimated_ops_trigger_timestamp=trigger_timestamp,
            uart_delay_ms=(first_byte - trigger_timestamp) * 1000,
            iwr_trigger_accepted=state["iwr_trigger_accepted"],
        )

    response = ops.wait_for_hardware_trigger(timeout=timeout_s, on_first_byte=on_first_byte)
    if not response:
        emit("ops_timeout")
        return False

    capture = processor.parse_capture(
        response,
        first_byte_timestamp=state["first_byte_timestamp"],
    )
    if capture is None:
        emit("ops_parse_failed", response_bytes=len(response))
        return False

    ops_path = Path(emit.output_dir) / "ops_capture.json"
    ops_path.write_text(
        json.dumps(
            {
                "sample_time": capture.sample_time,
                "trigger_time": capture.trigger_time,
                "I": capture.i_samples,
                "Q": capture.q_samples,
            }
        ),
        encoding="utf-8",
    )
    timeline = processor.process_standard(capture)
    outbound = [reading.speed_mph for reading in timeline.readings if reading.is_outbound]
    emit(
        "ops_capture",
        path=str(ops_path),
        response_bytes=len(response),
        post_trigger_ms=capture.post_trigger_duration_ms,
        outbound_readings=len(outbound),
        peak_outbound_mph=max(outbound, default=0.0),
    )

    trigger_timestamp = state["trigger_timestamp"]
    iwr_capture = iwr.capture_for_shot(trigger_timestamp, timeout_s=20.0)
    if iwr_capture is None:
        emit("iwr_timeout", trigger_timestamp=trigger_timestamp)
        return False
    emit(
        "iwr_capture",
        trigger_timestamp=iwr_capture.trigger_timestamp,
        completed_timestamp=iwr_capture.completed_timestamp,
        dump_duration_s=iwr_capture.dump_duration_s,
        path=str(iwr_capture.path) if iwr_capture.path else None,
        capture_bytes=len(iwr_capture.raw) if iwr_capture.raw else 0,
        error=iwr_capture.error,
    )
    return iwr_capture.valid


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ops-port", default="/dev/ttyAMA0")
    parser.add_argument("--iwr-port", default="/dev/ttyUSB0")
    parser.add_argument(
        "--iwr-config",
        default="config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg",
    )
    parser.add_argument("--trigger-threshold", type=float, default=40.0)
    parser.add_argument("--trigger-magnitude", type=int, default=600)
    parser.add_argument("--pre-trigger-segments", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if not 0 <= args.pre_trigger_segments <= OPS_SEGMENTS:
        parser.error("--pre-trigger-segments must be between 0 and 32")
    args.output_dir.mkdir(parents=True, exist_ok=False)

    evidence = args.output_dir / "events.jsonl"
    lock = threading.Lock()
    with evidence.open("x", encoding="utf-8") as output:

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
                print(json.dumps(record), flush=True)

        emit.output_dir = args.output_dir
        ops = OPS243Radar(port=args.ops_port)
        iwr = IWR6843CaptureMonitor(
            config_path=args.iwr_config,
            output_dir=args.output_dir,
            port=args.iwr_port,
            save_dumps=True,
        )
        try:
            iwr.start(armed=False)
            ops.connect()
            ops.configure_for_internal_speed_trigger(
                trigger_threshold_mph=args.trigger_threshold,
                trigger_magnitude=args.trigger_magnitude,
                pre_trigger_segments=args.pre_trigger_segments,
            )
            iwr.arm()
            emit(
                "ready",
                trigger_threshold_mph=args.trigger_threshold,
                trigger_magnitude=args.trigger_magnitude,
                pre_trigger_segments=args.pre_trigger_segments,
            )
            success = run_capture(
                ops=ops,
                iwr=iwr,
                processor=RollingBufferProcessor(),
                pre_segments=args.pre_trigger_segments,
                timeout_s=args.timeout,
                emit=emit,
            )
            emit("finished", success=success)
        finally:
            ops.disconnect()
            iwr.stop()


if __name__ == "__main__":
    main()
