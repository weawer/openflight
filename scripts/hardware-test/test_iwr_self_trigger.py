#!/usr/bin/env python3
"""
Check the IWR6843 self-trigger and l3sparse firmware on a connected board.

The host tests pin the firmware source; this runs it. Each check prints PASS
or FAIL and the script exits non-zero if any check fails.

  1. triggerCfg accepts a valid line and rejects malformed ones.
  2. l3sparse returns every cell of a request at the size limit, with a
     positive noise floor.
  3. A request longer than L3_SPARSE_REQUEST_MAX is refused with an error,
     and none of it leaks into the CLI as a command.
  4. A request that arrives after L3_SPARSE_REQUEST_TIMEOUT_MS is refused,
     and the next l3sparse still works.
  5. (--swing) With the trigger armed, a real swing prints "Triggered" and
     the frozen ring reads back.

Usage:
    uv run python scripts/hardware-test/test_iwr_self_trigger.py
    uv run python scripts/hardware-test/test_iwr_self_trigger.py --swing --tee-m 1.575
"""

from __future__ import annotations

import argparse
import sys
import time

sys.path.insert(0, "src")

from openflight.iwr6843.calibration import DEFAULT_CAL_PATH, Calibration  # noqa: E402
from openflight.iwr6843.driver import IWR6843Radar  # noqa: E402
from openflight.iwr6843.monitor import (  # noqa: E402
    SELF_TRIGGER_OFF_COMMAND,
    SelfTriggerConfig,
    tee_global_bin,
)
from openflight.iwr6843.sparse import (  # noqa: E402
    POWER_MAGIC,
    SPARSE_REQUEST_MAX_BYTES,
    SparsePlan,
    fit_cell_request,
    format_cell_request,
    parse_power,
    power_packet_size,
)

DEFAULT_CONFIG = "config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg"
# Must match L3_SPARSE_REQUEST_TIMEOUT_MS in firmware/iwr6843/dump_format.h.
REQUEST_TIMEOUT_S = 5.0


def _report(name: str, ok: bool, detail: str = "") -> bool:
    print(f"  {'PASS' if ok else 'FAIL'}  {name}{': ' + detail if detail else ''}")
    return ok


def _every_cell(summary) -> list[tuple[int, int]]:
    geometry = summary.geometry
    return [
        (frame, local)
        for frame in range(geometry.n_frames)
        for local in range(geometry.frame_bin_count(frame))
    ]


def check_trigger_cfg(radar: IWR6843Radar) -> bool:
    """Valid lines answer Done; malformed ones answer Error."""
    ok = True
    good = radar.cmd(SelfTriggerConfig(tee_bin=34, snr=6.0, track_frames=2).command, 2.0)
    ok &= _report("triggerCfg valid line", "Done" in good and "Error" not in good, good.strip())
    for line in ("triggerCfg 10 1000", "triggerCfg x 1000 2", "triggerCfg 10 -5 2"):
        reply = radar.cmd(line, 2.0)
        ok &= _report(f"rejects {line!r}", "Error" in reply, reply.strip())
    off = radar.cmd(SELF_TRIGGER_OFF_COMMAND, 2.0)
    ok &= _report("triggerCfg off", "Done" in off, off.strip())
    return ok


def check_limit_request(radar: IWR6843Radar) -> bool:
    """A request filled to the limit comes back complete."""
    seen = {}

    def plan(summary):
        cells = _every_cell(summary)
        _request, sent = fit_cell_request(cells)
        seen["sent"] = sent
        # First cell doubles as the noise cell; the rest fill the budget.
        return SparsePlan(cells=tuple(cells[1:sent]), noise_cells=(cells[0],))

    capture = radar.read_sparse(plan)
    if capture is None:
        return _report("l3sparse at the size limit", False, "firmware has no l3sparse")
    return _report(
        "l3sparse at the size limit",
        capture.sent_cells == seen["sent"] and capture.noise_power > 0,
        f"{capture.sent_cells}/{seen['sent']} cells, noise {capture.noise_power:.1f}",
    )


def _start_sparse(radar: IWR6843Radar):
    radar.ser.reset_input_buffer()
    radar.ser.write(b"l3sparse\n")
    read = radar._read_packet(POWER_MAGIC, power_packet_size, 8.0)  # pylint: disable=protected-access
    if read is None:
        raise RuntimeError("firmware has no l3sparse")
    return parse_power(read[0])


def _cli_reply(radar: IWR6843Radar, window_s: float) -> str:
    deadline = time.monotonic() + window_s
    reply = b""
    while time.monotonic() < deadline:
        reply += radar.ser.read(512)
        if b"Error" in reply and b"\n" in reply.split(b"Error", 1)[1]:
            break
    return reply.decode(errors="replace")


def check_oversized_request(radar: IWR6843Radar) -> bool:
    """Too long: refused, and the CLI does not see the tail as commands."""
    summary = _start_sparse(radar)
    cells = _every_cell(summary) * 4
    request = format_cell_request(cells)
    ok = _report(
        "test request exceeds the limit",
        len(request) > SPARSE_REQUEST_MAX_BYTES,
        f"{len(request)} bytes",
    )
    radar.ser.write(request)
    reply = _cli_reply(radar, 3.0)
    ok &= _report("oversized request refused", "longer than" in reply, reply.strip()[-80:])
    stats = radar.stats()
    ok &= _report(
        "CLI clean afterwards",
        "Done" in stats and "not recognized" not in stats,
        stats.strip()[-80:],
    )
    return ok


def check_late_request(radar: IWR6843Radar) -> bool:
    """Too late: refused, and the next exchange works."""
    _start_sparse(radar)
    time.sleep(REQUEST_TIMEOUT_S + 0.5)
    reply = _cli_reply(radar, 2.0)
    ok = _report("late request refused", "request missing" in reply, reply.strip()[-80:])
    capture = radar.read_sparse(
        lambda summary: SparsePlan(cells=(), noise_cells=(_every_cell(summary)[0],))
    )
    ok &= _report("next l3sparse works", capture is not None and capture.noise_power > 0)
    return ok


def check_swing(radar: IWR6843Radar, config: SelfTriggerConfig, wait_s: float) -> bool:
    """Arm the trigger, wait for a real swing, read the frozen ring."""
    reply = radar.cmd(config.command, 2.0)
    if not _report("armed", "Done" in reply, config.command):
        return False
    print(f"  Swing within {wait_s:.0f}s...")
    pending = b""
    deadline = time.monotonic() + wait_s
    found = False
    while time.monotonic() < deadline and not found:
        found, pending = radar.wait_trigger_notice(pending)
    notice_at = time.monotonic()
    ok = _report("Triggered seen", found)
    if found:
        capture = radar.read_sparse(
            lambda summary: SparsePlan(cells=(), noise_cells=(_every_cell(summary)[0],))
        )
        ok &= _report(
            "frozen ring read back",
            capture is not None,
            f"{time.monotonic() - notice_at:.2f}s after the notice",
        )
    radar.cmd(SELF_TRIGGER_OFF_COMMAND, 2.0)
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 2)[1])
    parser.add_argument("--port", default=None)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--swing", action="store_true", help="also wait for a real swing")
    parser.add_argument("--cal", default=DEFAULT_CAL_PATH)
    parser.add_argument("--tee-m", type=float, default=1.575)
    parser.add_argument("--snr", type=float, default=6.0)
    parser.add_argument("--frames", type=int, default=2)
    parser.add_argument("--wait-s", type=float, default=60.0)
    args = parser.parse_args()

    radar = IWR6843Radar(args.port)
    try:
        print(f"IWR6843 on {radar.port}, config {args.config}")
        radar.send_config(args.config)
        results = [
            check_trigger_cfg(radar),
            check_limit_request(radar),
            check_oversized_request(radar),
            check_late_request(radar),
        ]
        if args.swing:
            config = SelfTriggerConfig(
                tee_bin=tee_global_bin(
                    args.tee_m, args.config, range_bias_m=Calibration.load(args.cal).range_bias_m
                ),
                snr=args.snr,
                track_frames=args.frames,
            )
            results.append(check_swing(radar, config, args.wait_s))
    finally:
        try:
            radar.stop_sensor()
        finally:
            radar.close()
    print("ALL PASS" if all(results) else "FAILURES ABOVE")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())
