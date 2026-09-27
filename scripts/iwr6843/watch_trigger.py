#!/usr/bin/env python3
"""Watch the IWR6843 self-trigger detector and print why it fires.

Stop the kiosk first. This owns the TI UART. Arms ``triggerCfg``, then polls
until Ctrl+C: prints ``triggerLog`` every ``--poll-s`` seconds, and on a fire
prints it once more (showing the frame that fired), releases the frozen
ring, and keeps watching. Ported for the detector added in
Cormac131/feat/iwr-calcs; the old per-frame ``debugCfg`` stream is gone (see
firmware/releases/l3_dump_2ms_iq16_adaptive_l3release_20260927.md) -- use
``triggerLog trace`` on the board for what the detector saw between prints.

    uv run python scripts/iwr6843/watch_trigger.py --tee-m 1.845
"""

from __future__ import annotations

import argparse
import time

from openflight.iwr6843.calibration import DEFAULT_CAL_PATH, DEFAULT_TEE_RANGE_M, Calibration
from openflight.iwr6843.driver import IWR6843Radar
from openflight.iwr6843.monitor import (
    SELF_TRIGGER_DEFAULT_SNR,
    SELF_TRIGGER_DEFAULT_TRACK_FRAMES,
    SelfTriggerConfig,
    tee_global_bin,
)

_DEFAULT_CFG = "config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None)
    parser.add_argument("--config", default=_DEFAULT_CFG)
    parser.add_argument("--cal", default=DEFAULT_CAL_PATH)
    parser.add_argument("--tee-m", type=float, default=DEFAULT_TEE_RANGE_M)
    parser.add_argument("--snr", type=float, default=SELF_TRIGGER_DEFAULT_SNR)
    parser.add_argument("--frames", type=int, default=SELF_TRIGGER_DEFAULT_TRACK_FRAMES)
    parser.add_argument("--poll-s", type=float, default=2.0)
    args = parser.parse_args()

    if args.poll_s <= 0:
        parser.error("--poll-s must be positive")
    tee_bin = tee_global_bin(
        args.tee_m, args.config, range_bias_m=Calibration.load(args.cal).range_bias_m
    )
    trigger = SelfTriggerConfig(tee_bin, args.snr, args.frames)
    radar = IWR6843Radar(port=args.port)
    configured = False
    try:
        radar.send_config(args.config)
        configured = True
        reply = radar.cmd(trigger.command)
        if "Done" not in reply or "Error" in reply:
            raise SystemExit(f"triggerCfg rejected: {reply.strip()}")
        print(
            f"watching global bin {tee_bin}, snr {args.snr:g}, "
            f"{args.frames} frames. Ctrl+C to stop.",
            flush=True,
        )
        pending = b""
        last_print = 0.0
        while True:
            fired, pending = radar.wait_trigger_notice(pending)
            if fired:
                print(radar.cmd("triggerLog"), flush=True)
                radar.release_sparse_freeze()
                print("-- fired: released the frozen ring, watching again --", flush=True)
                last_print = time.monotonic()
                continue
            if time.monotonic() - last_print >= args.poll_s:
                print(radar.cmd("triggerLog"), flush=True)
                last_print = time.monotonic()
    except KeyboardInterrupt:
        print()
    finally:
        try:
            if configured:
                radar.stop_sensor()
        finally:
            radar.close()


if __name__ == "__main__":
    main()
