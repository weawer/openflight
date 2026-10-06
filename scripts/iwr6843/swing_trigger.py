#!/usr/bin/env python3
"""Swing the IWR6843 self-trigger and print the club track it fired on.

Stop the kiosk first. It owns this UART, so a swing there cannot show up
here, and this script cannot show up there.

The script samples the empty lane for 2s, arms ``triggerCfg`` at ``--snr``
over the firmware's floor, prints each trigger phase as it changes, and polls
``stats`` while the phase sits still. On ``Triggered`` (or ``latched=1``) it
reads ``triggerLog track`` and ``triggerLog shot``, releases the frozen ring,
and prints PASS when the club track's range-only impact is what fired. Ctrl+C
stops.

``--confirm`` turns the firmware's flight confirmation on (``trackCfg
confirm``), as the kiosk runs it: each fire is then a candidate, and the
script also prints whether the ball's flight confirmed it. A backswing should
fire a candidate the board rejects. Without it confirmation is turned off, so
a board left in confirm mode by the kiosk fires as before.

    uv run python scripts/iwr6843/swing_trigger.py --tee-m 1.575
    uv run python scripts/iwr6843/swing_trigger.py --port /dev/ttyUSB0 --tee-m 1.575
    uv run python scripts/iwr6843/swing_trigger.py --tee-m 1.575 --confirm
"""

from __future__ import annotations

import argparse
import sys
import time

from openflight.iwr6843.calibration import DEFAULT_TEE_RANGE_M
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.firmware_checks import parse_trig
from openflight.iwr6843.monitor import (
    DEFAULT_IWR6843_CONFIG,
    SELF_TRIGGER_DEFAULT_SNR,
    SelfTriggerConfig,
    measure_trigger_level,
    self_trigger_bin,
)

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


def format_status(fields: dict[str, str], threshold: float) -> str:
    """One live trigger line: the phase, the firmware's floor (``tee=``) and
    the ``floor x snr`` threshold club targets must reach."""
    parts = [f"{fields['phase']:<12} floor={fields['tee']}  threshold={threshold:.0f}"]
    for key in ("latched", "enabled"):
        if key in fields:
            parts.append(f"{key}={fields[key]}")
    return "  ".join(parts)


def _fields(line: str) -> dict[str, str]:
    return dict(token.split("=", 1) for token in line.split() if "=" in token)


def summarize_fire(track_reply: str) -> tuple[bool, str]:
    """PASS when ``triggerLog track`` says the club track's range impact fired.

    Returns ``(passed, text)``: the club track's point count and fitted speed,
    and why the range impact did or did not fire.
    """
    club = None
    ranged = None
    for line in track_reply.splitlines():
        stripped = line.strip()
        if stripped.startswith("clubtrack "):
            club = _fields(stripped)
        elif stripped.startswith("range impact "):
            ranged = _fields(stripped[len("range ") :])
    if club is None or ranged is None:
        return False, "  FAIL  no club track in the reply"
    summary = (
        f"{club.get('total', '?')} club points, speed {float(club.get('speed', 'nan')):.1f} m/s"
    )
    if ranged.get("fired") == "1":
        return True, f"  PASS  the club track's range impact fired: {summary}"
    return False, f"  FAIL  the range impact did not fire (why={ranged.get('why', '?')}): {summary}"


def summarize_confirm(track_reply: str) -> str | None:
    """The flight confirmation's verdict on this fire, or None when confirm
    mode is off (or the firmware has none)."""
    for line in track_reply.splitlines():
        stripped = line.strip()
        if not stripped.startswith("confirm "):
            continue
        fields = _fields(stripped)
        if fields.get("on") != "1":
            return None
        verdict = fields.get("verdict", "?")
        if verdict == "confirmed":
            return (
                f"  confirmed: a ball flight at {float(fields.get('speed', 'nan')):.1f} m/s, "
                f"{int(fields.get('dt', 0)) / 1000:.0f} ms after the candidate"
            )
        if verdict == "rejected":
            return f"  rejected: no ball flight (why={fields.get('why', '?')}), so no S!"
        if fields.get("unarmed", "0") != "0" and verdict == "idle":
            return "  fired at once: no ball tracker armed to confirm it"
        return f"  confirm verdict {verdict} (why={fields.get('why', '?')})"
    return None


def _validate_swing(radar: IWR6843Radar, swing: int) -> bool:
    """Read the club track that fired, release the ring, and judge the swing."""
    print(f"\nswing {swing}:", flush=True)
    try:
        track = radar.club_track()
        shot = radar.shot_status()
    except Exception as error:  # pylint: disable=broad-exception-caught
        print(f"  FAIL  {error}", flush=True)
        return False
    finally:
        # Released whatever the read did: a frozen ring stops the trigger.
        radar.release_sparse_freeze()
    for line in (track + shot).splitlines():
        if line.strip() and line.strip() != "Done" and not line.startswith("triggerLog"):
            print(f"  {line.strip()}", flush=True)
    passed, text = summarize_fire(track)
    print(text, flush=True)
    verdict = summarize_confirm(track)
    if verdict is not None:
        print(verdict, flush=True)
    for line in radar.stats().splitlines():
        fields = parse_trig(line)
        if fields is None:
            continue
        if is_latched(fields):
            print("  FAIL  ring still latched after the release", flush=True)
            return False
        break
    return passed


def _poll_status(radar: IWR6843Radar) -> dict[str, str] | None:
    found = None
    for line in radar.stats().splitlines():
        fields = parse_trig(line)
        if fields is not None:
            found = fields
    return found


def _status_key(fields: dict[str, str] | None) -> tuple | None:
    if fields is None:
        return None
    return (fields.get("phase"), fields.get("tee"), fields.get("latched"), fields.get("enabled"))


def watch(radar: IWR6843Radar, threshold: float) -> tuple[int, int]:
    """Print phase changes until Ctrl+C. Returns (passed, swings)."""
    pending = b""
    last_print = time.monotonic()
    last_key = None
    swings = 0
    passed = 0

    def swing() -> None:
        nonlocal swings, passed, last_key, last_print
        swings += 1
        if _validate_swing(radar, swings):
            passed += 1
        last_key = None
        last_print = time.monotonic()
        print("\nwatching. swing when ready.", flush=True)

    try:
        while True:
            waiting = radar.ser.in_waiting
            chunk = radar.ser.read(waiting or 1)
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
                        swing()
                        break
                    fields = parse_trig(line)
                    if is_latched(fields):
                        pending = b""
                        swing()
                        break
                    if fields is not None:
                        print(format_status(fields, threshold), flush=True)
                        last_print = time.monotonic()
                continue
            if pending or time.monotonic() - last_print < _QUIET_POLL_S:
                continue
            fields = _poll_status(radar)
            last_print = time.monotonic()
            if is_latched(fields):
                swing()
                continue
            key = _status_key(fields)
            if fields is not None and key != last_key:
                last_key = key
                print(format_status(fields, threshold), flush=True)
    except KeyboardInterrupt:
        print(f"\n{passed}/{swings} swings fired on the club track")
    return passed, swings


def _arm(
    radar: IWR6843Radar, config: str, tee_bin: int, snr: float, confirm: bool = False
) -> float:
    """Load the cfg, set the flight confirmation, sample the empty lane, arm,
    and return the threshold."""
    radar.send_config(config)
    reply = radar.cmd("debugCfg 1")
    if "Done" not in reply:
        raise SystemExit(f"debugCfg rejected: {reply.strip()}")
    # Always sent: the firmware keeps it across sensorStart.
    if not radar.set_confirm(confirm) and confirm:
        raise SystemExit("this firmware has no flight confirmation (trackCfg confirm); reflash it")
    print("Measuring the empty lane for 2s. Keep it clear.", flush=True)
    floor, threshold = measure_trigger_level(radar, tee_bin, snr=snr)
    print(f"floor p95 {floor:.0f}; threshold {threshold:.0f} at snr {snr:g}", flush=True)
    command = SelfTriggerConfig(tee_bin=tee_bin, snr=snr).command
    reply = radar.cmd(command)
    if "Done" not in reply:
        raise SystemExit(f"triggerCfg rejected: {reply.strip()}")
    print(f"armed {command}. Ctrl+C to stop.", flush=True)
    return threshold


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=None,
        help="IWR6843 CLI device. Leave unset to probe /dev/ttyUSB*. On Windows pass COMx.",
    )
    parser.add_argument("--config", default=DEFAULT_IWR6843_CONFIG)
    parser.add_argument("--tee-m", type=float, default=DEFAULT_TEE_RANGE_M)
    parser.add_argument(
        "--snr",
        type=float,
        default=SELF_TRIGGER_DEFAULT_SNR,
        help="club target threshold as a multiple of the firmware's running noise floor",
    )
    parser.add_argument(
        "--confirm",
        action="store_true",
        help="fire only once the ball's flight confirms a candidate, as the kiosk does",
    )
    args = parser.parse_args()

    error = port_name_error(args.port, sys.platform)
    if error:
        raise SystemExit(error)
    tee_bin = self_trigger_bin(args.tee_m, args.config)
    radar = IWR6843Radar(port=args.port)
    print(f"IWR6843 on {radar.port}. Stop the kiosk before swinging.", flush=True)
    try:
        threshold = _arm(radar, args.config, tee_bin, args.snr, confirm=args.confirm)
        watch(radar, threshold)
    finally:
        try:
            radar.cmd("debugCfg 0", window=0.5)
        except Exception:  # pylint: disable=broad-exception-caught
            pass
        radar.close()


if __name__ == "__main__":
    main()
