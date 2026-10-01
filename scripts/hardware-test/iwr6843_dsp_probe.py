#!/usr/bin/env python3
"""Prove the IWR6843 MSS <-> DSS detect link on a board (Phase 0).

The self-trigger's detect task is moving from the R4F (MSS) to the C674x
(DSS). Before any of it moves, this checks on the board that:

1. the DSS boots and answers over the mailbox (``trackCfg dsp ping``);
2. it reads the live L3 ring and scores a frame exactly as the MSS does,
   bit for bit (``trackCfg dsp probe``: the same l3_bin_score code on both);
3. how much faster it is, for the pre-impact scan plan's 27 bins and the
   whole window.

Usage (stop the kiosk first; it owns the port)::

    uv run python scripts/hardware-test/iwr6843_dsp_probe.py

With ``--acceptance`` it then arms the self-trigger and runs the detector
with the DSS scoring its bins, first in ``verify`` (both cores, compared)
and then in ``dss``, ``--seconds`` each while you swing, and judges what
the board reports (``dsp_link.evaluate_acceptance``): no verify mismatch or
DSS failure, no fallback or latch, no dropped or stale frame, the detect
task keeping up with the frames, no club angle dropped::

    uv run python scripts/hardware-test/iwr6843_dsp_probe.py --acceptance --seconds 60

Exit status 0 only when every ping answered, every probe matched and (with
``--acceptance``) every check passed.
"""

from __future__ import annotations

import argparse
import re
import sys
import time

sys.path.insert(0, "src")

from openflight.iwr6843.driver import IWR6843Radar  # noqa: E402
from openflight.iwr6843.dsp_link import (  # noqa: E402
    DspLinkError,
    evaluate_acceptance,
    format_timing_report,
    parse_detect_health,
    summarize_probes,
)
from openflight.iwr6843.monitor import SelfTriggerConfig  # noqa: E402

DEFAULT_CONFIG = "config/iwr6843_l3dump_wide_24f3ms_53bin_iq16.cfg"
SCAN_PLAN_BINS = 27  # the pre-impact scan plan (l3_scan.h)
# The self-trigger at the kiosk's tee (1.70 m from the enclosure front: the
# ball at bin 43, watched two bins short).
DEFAULT_TRIGGER_BIN = 41
DEFAULT_TRIGGER_SNR = 1.0


def hold(radar: IWR6843Radar, seconds: float, restart) -> tuple[int, int]:
    """Let the detector run for seconds, rearming it (l3release) after each
    swing it fires on, as the kiosk does: fired, it freezes and scores
    nothing more until released. A board found stopped and NOT latched
    scores nothing and nothing rearms it (the 2026-09-30 lock-up): it is
    restarted with restart() and counted. Returns (fired, restarts)."""
    fired = restarts = 0
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        time.sleep(1.0)
        stats = radar.stats()
        if "latched=1" in stats:
            radar.release_sparse_freeze()
            fired += 1
        elif re.search(r"active=0", stats):
            print("  board stopped and not latched: restarting it")
            restart()
            restarts += 1
    return fired, restarts


def report(radar: IWR6843Radar, label: str):
    """Print the phase's full detect timing (the firmware restarts it on
    every detectCore switch, so read it before the next) and the detect
    task's health; return (timing, stats text)."""
    timing = radar.detect_timing()
    stats = radar.stats()
    for line in format_timing_report(timing, label):
        print(f"  {line}")
    health = parse_detect_health(stats)
    print(
        "  detect: no health line in stats"
        if health is None
        else (
            f"  detect dropped={health.dropped} stale={health.stale} "
            f"stale_read={health.stale_read} shed={health.shed} "
            f"notice_dropped={health.notice_dropped}"
        )
    )
    return timing, stats


def run_acceptance(radar: IWR6843Radar, args: argparse.Namespace) -> bool:
    """Arm the self-trigger, run verify then dss while the operator swings,
    and print each check. True when every one passed."""
    trigger = SelfTriggerConfig(tee_bin=args.trigger_bin, snr=args.trigger_snr)

    def arm() -> bool:
        reply = radar.cmd(trigger.command, 2.0)
        if "Error" in reply or "Done" not in reply:
            print(f"FAIL acceptance: the self-trigger was refused: {reply.strip()}")
            return False
        return True

    def restart() -> None:
        radar.send_config(args.config)
        if args.dss_only:
            # Before arming: on a profile the MSS cannot score in time, an
            # armed MSS detector starves the CLI and the board stops answering.
            radar.detect_core("dss")
        arm()

    if args.dss_only:
        radar.detect_core("dss")
    if not arm():
        return False
    restarts = 0
    verify = None
    if not args.dss_only:
        print(f"acceptance: verify for {args.seconds:.0f} s: swing now")
        radar.detect_core("verify")
        fired, stopped = hold(radar, args.seconds, restart)
        restarts += stopped
        print(f"  fired on {fired} swings")
        verify = radar.detect_core()
        report(radar, "verify")
    print(f"acceptance: dss for {args.seconds:.0f} s: swing again")
    radar.detect_core("dss")
    fired, stopped = hold(radar, args.seconds, restart)
    restarts += stopped
    print(f"  fired on {fired} swings")
    dss = radar.detect_core()
    timing, stats = report(radar, "dss")
    checks = evaluate_acceptance(
        verify=dss if verify is None else verify,
        dss=dss,
        timing=timing,
        stats_text=stats,
        perf_text=radar.cmd("triggerLog perf", 2.0),
        recoveries=restarts,
    )
    if verify is None:
        checks = [check for check in checks if not check.name.startswith("verify")]
    for check in checks:
        print(f"  {'pass' if check.passed else 'FAIL'} {check.name}: {check.detail}")
    radar.detect_core("dss")  # the default: clears a latch from the run
    return all(check.passed for check in checks)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", maxsplit=1)[0])
    parser.add_argument("--config", default=DEFAULT_CONFIG, help="an IQ16 .cfg profile")
    parser.add_argument("--port", default=None, help="CLI serial port (default: auto-detect)")
    parser.add_argument("--repeats", type=int, default=20, help="pings and probes per size")
    parser.add_argument("--settle-s", type=float, default=1.0, help="seconds after sensorStart")
    parser.add_argument(
        "--acceptance", action="store_true", help="then run verify and dss while you swing"
    )
    parser.add_argument(
        "--dss-only",
        action="store_true",
        help="skip verify and arm on the DSS (profiles the MSS cannot score in time, e.g. 2 ms)",
    )
    parser.add_argument("--seconds", type=float, default=60.0, help="each acceptance phase")
    parser.add_argument("--trigger-bin", type=int, default=DEFAULT_TRIGGER_BIN)
    parser.add_argument("--trigger-snr", type=float, default=DEFAULT_TRIGGER_SNR)
    args = parser.parse_args()

    ok = True
    with IWR6843Radar(port=args.port) as radar:
        print(f"IWR6843 on {radar.port}")
        try:
            pings = [radar.dsp_ping() for _ in range(args.repeats)]
        except DspLinkError as exc:
            print(f"FAIL ping: {exc}")
            try:
                status = radar.dsp_status()
            except DspLinkError as status_exc:
                print(f"  no DSS status either ({status_exc}): flash the latest link image")
                return 1
            print(
                f"  DSS status: stage={status.stage}{' FAILED' if status.failed else ''} "
                f"err={status.err} beats={status.beats} served={status.served}"
                + (
                    f" exception pc={status.exc_pc:08x} efr={status.exc_efr:08x}"
                    if status.exc_pc is not None
                    else ""
                )
            )
            time.sleep(1.0)
            again = radar.dsp_status()
            print(f"  beats one second later: {again.beats} (rising means the DSS task runs)")
            try:
                hw = radar.dsp_hw()
            except DspLinkError as hw_exc:
                print(f"  no DSS hardware state ({hw_exc}): flash the latest link image")
                return 1
            print(
                f"  DSS hardware: stage in DSSGPREG0={hw.gpreg_stage} halted={hw.halted} "
                f"powered={hw.powered} (power={hw.power}) stc={hw.stc} "
                f"hs_ram_readback={'ok' if hw.hsram_ok else 'BAD'}"
            )
            print("  ESM status: " + " ".join(f"{word:08x}" for word in hw.esm))
            return 1
        print(f"ping: {len(pings)} answered, round trip {min(pings)}..{max(pings)} us")

        radar.send_config(args.config)
        time.sleep(args.settle_s)
        try:
            for bins in (SCAN_PLAN_BINS, None):
                probes = [radar.dsp_probe(bins) for _ in range(args.repeats)]
                summary = summarize_probes(probes)
                label = f"{probes[0].bins} bins" + (" (scan plan)" if bins else " (whole window)")
                prep = summary.dss_total_us_median - summary.dss_us_median
                print(
                    f"probe {label}: MSS {summary.mss_us_median:.0f} us, "
                    f"DSS {summary.dss_total_us_median:.0f} us "
                    f"(prepare {prep:.0f} + score {summary.dss_us_median:.0f}, "
                    f"gathered {summary.gathered}/{summary.count}) "
                    f"-> {summary.speedup:.1f}x faster, "
                    f"{summary.count - summary.mismatches}/{summary.count} matched"
                )
                for probe in probes:
                    if not probe.match:
                        print(
                            f"  MISMATCH slot={probe.slot} status={probe.status} "
                            f"mss_energy={probe.mss_energy} dss_energy={probe.dss_energy}"
                        )
                ok = ok and summary.mismatches == 0
            if args.acceptance and ok:
                ok = run_acceptance(radar, args)
        except DspLinkError as exc:
            print(f"FAIL probe: {exc}")
            ok = False
        finally:
            radar.stop_sensor()
    print("PASS: the DSS answers and scores the ring as the MSS does" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
