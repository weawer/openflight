#!/usr/bin/env python3
"""Print the IWR6843 self-trigger as it runs.

Stop the kiosk first. This owns the TI UART, arms triggerCfg, turns on
debugCfg, and prints the board's lines until you press Ctrl+C. Each time the
club track's range-only impact fires, the club track it fired on is printed
(``triggerLog track``) before the frozen ring is released.
"""

from __future__ import annotations

import argparse
import time

from openflight.iwr6843.calibration import DEFAULT_TEE_RANGE_M
from openflight.iwr6843.driver import TRIGGER_NOTICE, IWR6843Radar
from openflight.iwr6843.monitor import (
    DEFAULT_IWR6843_CONFIG,
    SELF_TRIGGER_DEFAULT_SNR,
    SelfTriggerConfig,
    measure_trigger_level,
    self_trigger_bin,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None)
    parser.add_argument("--config", default=DEFAULT_IWR6843_CONFIG)
    parser.add_argument("--tee-m", type=float, default=DEFAULT_TEE_RANGE_M)
    parser.add_argument(
        "--snr",
        type=float,
        default=SELF_TRIGGER_DEFAULT_SNR,
        help="club target threshold as a multiple of the firmware's running noise floor",
    )
    args = parser.parse_args()

    tee_bin = self_trigger_bin(args.tee_m, args.config)
    arm = SelfTriggerConfig(tee_bin=tee_bin, snr=args.snr).command
    radar = IWR6843Radar(port=args.port)
    try:
        radar.send_config(args.config)
        # Every fire, as before: the kiosk may have left the flight
        # confirmation on, and the firmware keeps it across sensorStart.
        radar.set_confirm(False)
        # The firmware keeps its own noise floor; show it and the threshold
        # it implies so a swing's club track can be read against them.
        floor, threshold = measure_trigger_level(radar, tee_bin, snr=args.snr)
        print(f"floor p95 {floor:.0f}; threshold {threshold:.0f} at snr {args.snr:g}", flush=True)
        # Debug first so the frames right after arming are visible: a fire in
        # that window used to be swallowed by the next command's buffer reset.
        reply = radar.cmd("debugCfg 1")
        if "Done" not in reply:
            raise SystemExit(f"debugCfg rejected: {reply.strip()}")
        print(f"watching global bin {tee_bin}, snr {args.snr:g}. Ctrl+C to stop.", flush=True)
        reply = radar.cmd(arm)
        if "Done" not in reply:
            raise SystemExit(f"triggerCfg rejected: {reply.strip()}")
        print(reply.replace("Done", "").strip(), flush=True)
        # The arming reply itself may carry the notice.
        pending = reply.encode()
        while True:
            if TRIGGER_NOTICE in pending:
                pending = b""
                # Debug text shares this UART with the binary release. Silence
                # it for the exchange, then turn it back on to keep watching.
                radar.cmd("debugCfg 0", window=1.0)
                print("\n-- fired on the club track --", flush=True)
                print(radar.club_track().replace("Done", "").strip(), flush=True)
                radar.release_sparse_freeze()
                reply = radar.cmd("debugCfg 1")
                if "Done" not in reply:
                    raise SystemExit(f"debugCfg rejected: {reply.strip()}")
                print("-- released the frozen ring, watching again --", flush=True)
            else:
                pending = pending[-(len(TRIGGER_NOTICE) - 1) :]
            waiting = radar.ser.in_waiting
            chunk = radar.ser.read(waiting or 1)
            if not chunk:
                time.sleep(0.02)
                continue
            print(chunk.decode(errors="replace"), end="", flush=True)
            pending += chunk
    except KeyboardInterrupt:
        print()
    finally:
        try:
            radar.cmd("debugCfg 0", window=0.5)
        except Exception:
            pass
        radar.close()


if __name__ == "__main__":
    main()
