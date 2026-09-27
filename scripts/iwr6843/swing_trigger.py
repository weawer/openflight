#!/usr/bin/env python3
"""Swing the IWR6843 self-trigger and print what the frozen ring contains.

Stop the kiosk first. It owns this UART, so a swing there cannot show up
here, and this script cannot show up there.

With no ``--level``, it samples the empty lane for 2s and arms above that
floor. Pass ``--level`` to skip the sample.

The script arms ``triggerCfg``, prints each detector phase as it changes,
and polls ``stats`` while the phase sits still so tee power stays visible.
On ``Triggered`` (or ``latched=1``) it reads the frozen ring, replays the
ball-leave detector, and prints PASS or FAIL. Ctrl+C stops.

    uv run python scripts/iwr6843/swing_trigger.py --tee-m 1.575
    uv run python scripts/iwr6843/swing_trigger.py --port /dev/ttyUSB0 --tee-m 1.575

For a latency measurement, pin the CPU governor first so frequency scaling
doesn't add its own jitter to the result:
    scripts/hardware-test/set_cpu_governor.sh
    ... run the swing ...
    scripts/hardware-test/set_cpu_governor.sh restore
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from openflight.iwr6843.calibration import DEFAULT_TEE_RANGE_M
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.firmware_checks import parse_trig
from openflight.iwr6843.monitor import measure_trigger_level, tee_global_bin
from openflight.iwr6843.self_trigger import BallLeaveDetector, TriggerObservation, replay_dump
from openflight.iwr6843.sparse import SparsePlan

_DEFAULT_CFG = "config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg"
_QUIET_POLL_S = 2.0


def port_name_error(port: str | None, platform: str) -> str | None:
    """Windows ``COMn`` names are not device paths on the Pi."""
    if not port or platform == "win32":
        return None
    suffix = port[3:]
    if port.upper().startswith("COM") and suffix.isdigit():
        return (
            f"{port} is a Windows port name. On this machine leave --port off, "
            "or pass a device path such as /dev/ttyUSB0."
        )
    return None


def is_latched(fields: dict[str, str] | None) -> bool:
    """True when this line says the ring is frozen.

    ``phase=fired`` with ``latched=0`` is the leftover phase after the ring
    was released. Treating that as a new swing would freeze the live ring.
    """
    if not fields:
        return False
    if fields.get("latched") == "1":
        return True
    return fields.get("phase") == "fired" and "latched" not in fields


def format_status(fields: dict[str, str], level: float) -> str:
    """One live detector line."""
    try:
        tee = float(fields["tee"])
    except ValueError:
        tee = float("nan")
    occupied = "ball" if tee >= level else "empty"
    parts = [
        f"{fields['phase']:<12} tee={fields['tee']}  level={level:g}  {occupied}",
    ]
    for key in ("approach", "peak", "latched", "enabled"):
        if key in fields:
            parts.append(f"{key}={fields[key]}")
    return "  ".join(parts)


def replay_loop0(
    power: np.ndarray,
    n_loops: int,
    tee_bin: int,
    level: float,
    hits: int,
    frame_bins: list[int] | None = None,
    frame_period_s: float | None = None,
) -> list[TriggerObservation]:
    """Replay loop-0 residual power, the same probe the firmware uses.

    ``frame_period_s`` times the detector's motion timeout; None keeps the
    wide profile's default.
    """
    rows, bins = np.asarray(power).shape
    if n_loops < 1 or rows % n_loops:
        raise ValueError(f"power rows {rows} are not a multiple of {n_loops} loops")
    loop0 = np.asarray(power).reshape(rows // n_loops, n_loops, bins)[:, 0, :]
    detector = BallLeaveDetector(
        level=level,
        hits=hits,
        **({} if frame_period_s is None else {"frame_period_s": frame_period_s}),
    )
    observations: list[TriggerObservation] = []
    for frame, row in enumerate(loop0):
        count = bins if frame_bins is None else frame_bins[frame]
        tee_local = tee_bin if 0 <= tee_bin < count else None
        observations.append(detector.step(frame, row, tee_local, count))
    return observations


def format_hotspot(row: np.ndarray, tee_bin: int, level: float, approach_bins: int = 12) -> str:
    """Where the fire frame was loud: tee, closer bins, and bins just past the tee."""
    power = np.asarray(row, dtype=float)
    count = len(power)
    if not 0 <= tee_bin < count:
        return f"  fire frame: tee bin {tee_bin} is outside 0..{count - 1}"
    first = max(0, tee_bin - approach_bins)
    past_end = min(count, tee_bin + 1 + approach_bins)
    approach = power[first:tee_bin]
    past = power[tee_bin + 1 : past_end]
    hot = int(np.count_nonzero(power >= level))

    def loudest(segment: np.ndarray, offset: int) -> str:
        if segment.size == 0:
            return "none"
        index = int(np.argmax(segment))
        return f"bin {offset + index}={segment[index]:.0f}"

    return (
        f"  fire frame: tee bin {tee_bin}={power[tee_bin]:.0f}  "
        f"approach {loudest(approach, first)}  "
        f"past {loudest(past, tee_bin + 1)}  "
        f"{hot}/{count} bins >= {level:g}"
    )


def format_swing(observations: list[TriggerObservation]) -> str:
    """Phase changes, then whether the replay latched."""
    if not observations:
        return "  no frames in the frozen ring"
    lines = []
    previous = None
    for obs in observations:
        if obs.phase == previous:
            continue
        lines.append(
            f"  {obs.phase:<12} frame={obs.frame}  tee={obs.tee:.0f}  "
            f"approach={obs.approach:.0f}  peak={obs.peak_bin}"
        )
        previous = obs.phase
    fired = next((obs for obs in observations if obs.fired), None)
    if fired is None:
        lines.append(f"  FAIL  replay never fired (last {observations[-1].phase})")
    else:
        lines.append(f"  PASS  replay fired at frame {fired.frame}")
    return "\n".join(lines)


def _read_power(radar: IWR6843Radar):
    """Power map from ``l3sparse``, or None when that command is refused.

    The map arrives before any complex cells. An empty cell request releases
    the ring; assembling that empty reply has no noise sample, which the
    driver rejects after the firmware has already re-armed.
    """
    held: dict = {}

    def plan(summary):
        held["summary"] = summary
        return SparsePlan(cells=())

    try:
        capture = radar.read_sparse(plan)
    except Exception:
        if "summary" not in held:
            raise
        return held["summary"]
    if capture is None:
        return None
    return held.get("summary")


def _fire_hotspot(summary, tee_bin: int, level: float, observations) -> str | None:
    fired = next((obs for obs in observations if obs.fired), None)
    if fired is None:
        return None
    power = np.asarray(summary.power)
    rows, bins = power.shape
    if summary.n_loops < 1 or rows % summary.n_loops:
        return None
    loop0 = power.reshape(rows // summary.n_loops, summary.n_loops, bins)[:, 0, :]
    if fired.frame >= len(loop0):
        return None
    count = summary.geometry.frame_bin_count(fired.frame)
    return format_hotspot(loop0[fired.frame, :count], tee_bin, level)


def _read_swing(radar: IWR6843Radar, tee_bin: int, tee_m: float, level: float, hits: int):
    summary = _read_power(radar)
    if summary is not None:
        counts = [
            summary.geometry.frame_bin_count(frame) for frame in range(summary.geometry.n_frames)
        ]
        observations = replay_loop0(
            summary.power,
            summary.n_loops,
            tee_bin,
            level,
            hits,
            counts,
            summary.geometry.frame_period_s,
        )
        return observations, _fire_hotspot(summary, tee_bin, level, observations)
    print("  sparse read unavailable, reading the full ring...", flush=True)
    raw = radar.read_dump()
    return replay_dump(raw, tee_range_m=tee_m, level=level, hits=hits), None


def _validate_swing(
    radar: IWR6843Radar,
    swing: int,
    tee_bin: int,
    tee_m: float,
    level: float,
    hits: int,
) -> bool:
    print(f"\nswing {swing}: reading frozen ring...", flush=True)
    try:
        observations, hotspot = _read_swing(radar, tee_bin, tee_m, level, hits)
    except Exception as error:  # pylint: disable=broad-exception-caught
        print(f"  FAIL  {error}", flush=True)
        return False
    print(format_swing(observations), flush=True)
    if hotspot:
        print(hotspot, flush=True)
    health = radar.stats()
    for line in health.splitlines():
        fields = parse_trig(line)
        if fields is None:
            continue
        print(f"  after {format_status(fields, level)}", flush=True)
        if is_latched(fields):
            print("  FAIL  ring still latched after the read", flush=True)
            return False
        break
    return any(obs.fired for obs in observations)


def _poll_status(radar: IWR6843Radar) -> tuple[dict[str, str] | None, str | None]:
    """Trigger fields, plus ``active`` from the capture-counter line."""
    health = radar.stats()
    found = None
    active = None
    for line in health.splitlines():
        if active is None and "active=" in line:
            for token in line.split():
                if token.startswith("active="):
                    active = token.split("=", 1)[1]
        fields = parse_trig(line)
        if fields is not None:
            found = fields
    return found, active


def _status_key(fields: dict[str, str] | None) -> tuple | None:
    if fields is None:
        return None
    return (fields.get("phase"), fields.get("tee"), fields.get("latched"), fields.get("enabled"))


def _consume_line(line: str, level: float) -> dict[str, str] | None:
    if not line or line == "Done" or line.endswith(":/>"):
        return None
    fields = parse_trig(line)
    if fields is not None and not is_latched(fields):
        print(format_status(fields, level), flush=True)
    return fields


def watch(
    radar: IWR6843Radar,
    tee_bin: int,
    tee_m: float,
    level: float,
    hits: int,
) -> tuple[int, int]:
    """Print phase changes until Ctrl+C. Returns (fired, swings)."""
    pending = b""
    last_print = time.monotonic()
    last_key = None
    reported_stuck = False
    swings = 0
    fired = 0
    try:
        while True:
            waiting = radar.ser.in_waiting
            chunk = radar.ser.read(waiting or 1)
            now = time.monotonic()
            if chunk:
                pending += chunk
                if len(pending) > 8192 and b"\n" not in pending:
                    pending = b""
                    print("  dropped a non-text burst", flush=True)
                    continue
                while b"\n" in pending:
                    raw, pending = pending.split(b"\n", 1)
                    line = raw.decode(errors="replace").strip()
                    if "Triggered" in line:
                        pending = b""
                        print("  Triggered", flush=True)
                        swings += 1
                        if _validate_swing(radar, swings, tee_bin, tee_m, level, hits):
                            fired += 1
                        last_print = time.monotonic()
                        last_key = None
                        reported_stuck = False
                        print("\nwatching. swing when ready.", flush=True)
                        break
                    fields = _consume_line(line, level)
                    if is_latched(fields):
                        print(format_status(fields, level), flush=True)
                        pending = b""
                        swings += 1
                        if _validate_swing(radar, swings, tee_bin, tee_m, level, hits):
                            fired += 1
                        last_print = time.monotonic()
                        last_key = None
                        reported_stuck = False
                        print("\nwatching. swing when ready.", flush=True)
                        break
                    if fields is not None:
                        last_print = time.monotonic()
                continue
            if pending or now - last_print < _QUIET_POLL_S:
                continue
            fields, active = _poll_status(radar)
            last_print = time.monotonic()
            if is_latched(fields):
                print(format_status(fields, level), flush=True)
                swings += 1
                if _validate_swing(radar, swings, tee_bin, tee_m, level, hits):
                    fired += 1
                last_key = None
                reported_stuck = False
                print("\nwatching. swing when ready.", flush=True)
                continue
            key = _status_key(fields)
            if fields is None or key == last_key:
                if (
                    not reported_stuck
                    and fields is not None
                    and fields.get("phase") == "fired"
                    and fields.get("latched") == "0"
                ):
                    reported_stuck = True
                    sensor = f" active={active}" if active is not None else ""
                    print(
                        f"  still fired, latched=0, tee={fields['tee']}{sensor}. "
                        "That sample is not changing, so the detector is not running.",
                        flush=True,
                    )
                continue
            reported_stuck = False
            last_key = key
            print(format_status(fields, level), flush=True)
    except KeyboardInterrupt:
        print(f"\n{fired}/{swings} swings replayed as fired")
    return fired, swings


def _arm(radar: IWR6843Radar, config: str, tee_bin: int, level: float | None, hits: int) -> float:
    """Load the cfg, optionally sample the empty lane, and return the armed level."""
    radar.send_config(config)
    reply = radar.cmd("debugCfg 1")
    if "Done" not in reply:
        raise SystemExit(f"debugCfg rejected: {reply.strip()}")
    if level is None:
        print("Measuring the empty lane for 2s. Keep it clear.", flush=True)
        floor, level = measure_trigger_level(radar, tee_bin, hits)
        print(f"background p95 {floor:.0f}; arming at {level:.0f}", flush=True)
    for line in reply.splitlines():
        fields = parse_trig(line)
        if fields is not None:
            print(format_status(fields, level), flush=True)
    command = f"triggerCfg {tee_bin} {level:g} {hits}"
    reply = radar.cmd(command)
    if "Done" not in reply:
        raise SystemExit(f"triggerCfg rejected: {reply.strip()}")
    print(
        f"armed {command}. Ball on the tee should read 'ball' / watching. Ctrl+C to stop.",
        flush=True,
    )
    return level


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=None,
        help="IWR6843 CLI device. Leave unset to probe /dev/ttyUSB*. On Windows pass COMx.",
    )
    parser.add_argument("--config", default=_DEFAULT_CFG)
    parser.add_argument("--tee-m", type=float, default=DEFAULT_TEE_RANGE_M)
    parser.add_argument(
        "--level",
        type=float,
        default=None,
        help="Residual that counts as a ball. Omit to sample the empty lane for 2s.",
    )
    parser.add_argument("--hits", type=int, default=2)
    args = parser.parse_args()

    error = port_name_error(args.port, sys.platform)
    if error:
        raise SystemExit(error)
    tee_bin = tee_global_bin(args.tee_m, args.config)
    radar = IWR6843Radar(port=args.port)
    print(f"IWR6843 on {radar.port}. Stop the kiosk before swinging.", flush=True)
    try:
        level = _arm(radar, args.config, tee_bin, args.level, args.hits)
        watch(radar, tee_bin, args.tee_m, level, args.hits)
    finally:
        try:
            radar.cmd("debugCfg 0", window=0.5)
        except Exception:  # pylint: disable=broad-exception-caught
            pass
        radar.close()


if __name__ == "__main__":
    main()
