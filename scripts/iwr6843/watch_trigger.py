#!/usr/bin/env python3
"""Print the IWR6843 ball-leave detector as it runs.

Stop the kiosk first. This owns the TI UART, arms triggerCfg, turns on
debugCfg, and prints one line per frame until you press Ctrl+C.
"""

from __future__ import annotations

import argparse
import time

from openflight.iwr6843.calibration import DEFAULT_CAL_PATH, DEFAULT_TEE_RANGE_M, Calibration
from openflight.iwr6843.driver import TRIGGER_NOTICE, IWR6843Radar
from openflight.iwr6843.monitor import (
    SELF_TRIGGER_DEFAULT_SNR,
    measure_trigger_level,
    tee_global_bin,
)

_DEFAULT_CFG = "config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=None)
    parser.add_argument("--config", default=_DEFAULT_CFG)
    parser.add_argument("--tee-m", type=float, default=DEFAULT_TEE_RANGE_M)
    parser.add_argument("--cal", default=DEFAULT_CAL_PATH)
    parser.add_argument(
        "--snr",
        type=float,
        default=SELF_TRIGGER_DEFAULT_SNR,
        help="candidate threshold as a multiple of the firmware's running noise floor",
    )
    parser.add_argument(
        "--hits", type=int, default=2, help="tracked frames before the gate may fire"
    )
    args = parser.parse_args()

    tee_bin = tee_global_bin(
        args.tee_m, args.config, range_bias_m=Calibration.load(args.cal).range_bias_m
    )
    radar = IWR6843Radar(port=args.port)
    try:
        radar.send_config(args.config)
        # The firmware keeps its own noise floor; show it and the threshold
        # it implies so a swing's triggerLog can be read against them.
        floor, threshold = measure_trigger_level(radar, tee_bin, args.hits, snr=args.snr)
        print(f"floor p95 {floor:.0f}; threshold {threshold:.0f} at snr {args.snr:g}", flush=True)
        # Debug first so the frames right after arming are visible: a fire in
        # that window used to be swallowed by the next command's buffer reset.
        reply = radar.cmd("debugCfg 1")
        if "Done" not in reply:
            raise SystemExit(f"debugCfg rejected: {reply.strip()}")
        print(
            f"watching global bin {tee_bin}, snr {args.snr:g}, {args.hits} frames. Ctrl+C to stop.",
            flush=True,
        )
        reply = radar.cmd(f"triggerCfg {tee_bin} {args.snr} {args.hits}")
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
                radar.release_sparse_freeze()
                reply = radar.cmd("debugCfg 1")
                if "Done" not in reply:
                    raise SystemExit(f"debugCfg rejected: {reply.strip()}")
                print("\n-- fired: released the frozen ring, watching again --", flush=True)
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
