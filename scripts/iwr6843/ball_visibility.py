#!/usr/bin/env python3
"""Phase 0: was the ball visible to the live selector in a range session?

For each IWR capture in a session log, rebuilds the firmware selector input
from the saved dump, finds the ball's range-vs-time line (searched around the
OPS ball speed), and counts frames where the ball was a qualifying peak and
where it ranked among the two candidates the firmware keeps. Offline only;
it does not touch the radar.

Needs a session recorded with the fixed-window diagnostic cfg
(config/iwr6843_l3dump_diagnostic_24f2ms_53bin_iq16.cfg), whose ball-phase
windows stay put so their rise can be rebuilt once the ball is past the
golfer. Adaptive dumps rebuild only their impact frames, where the ball and
the golfer share range bins, so their counts say nothing about the ball.
Ranks only cover the retained bins; see openflight/iwr6843/ball_visibility.py.

    uv run python scripts/iwr6843/ball_visibility.py \\
        openflight_sessions/session_20260927_185647_range.jsonl --frames
"""

from __future__ import annotations

import argparse
import json
import re
from dataclasses import asdict
from pathlib import Path

from openflight.iwr6843.ball_visibility import BallLine, find_ball_line
from openflight.iwr6843.dump import parse_dump
from openflight.iwr6843.flight_track import BIN_M, FOLLOWS_BALL_RATIO, MPS_TO_MPH
from openflight.iwr6843.live_selector import gated_selector_inputs
from openflight.iwr6843.monitor import read_capture_config


def _session_start(session: Path) -> dict:
    with session.open(encoding="utf-8") as handle:
        for line in handle:
            entry = json.loads(line)
            if entry.get("type") == "session_start":
                return entry
    raise ValueError(f"{session} has no session_start entry")


def _captures(session: Path):
    with session.open(encoding="utf-8") as handle:
        for line in handle:
            entry = json.loads(line)
            if entry.get("type") == "iwr6843_capture" and entry.get("capture_path"):
                yield entry


def _phase(frame: int, pre: int, impact: int) -> str:
    if frame < pre:
        return "pre"
    return "impact" if frame < pre + impact else "flight"


def _analyse(
    path: Path, *, ops_mph, tee_bin, pre_default, impact_frames
) -> tuple[dict, BallLine | None]:
    meta, cube = parse_dump(path.read_bytes())[:2]
    frames = meta["n_frames"]
    period_s = meta["frame_period_us"] / 1e6
    starts = meta.get("range_bin_starts") or [meta["range_bin_start"]] * frames
    counts = meta.get("range_bin_counts") or [meta["n_samples"]] * frames
    offsets = meta.get("frame_time_offsets_us") or [
        i * meta["frame_period_us"] for i in range(frames)
    ]
    retention = meta.get("retention")
    pre = retention["pre_frames"] if retention else pre_default
    ops_bins = ops_mph / MPS_TO_MPH * period_s / BIN_M if ops_mph else None
    line = find_ball_line(
        gated_selector_inputs(meta, cube),
        [(start, start + count) for start, count in zip(starts, counts)],
        [offset / 1e6 for offset in offsets],
        tee_bin=tee_bin,
        impact_frames=range(max(1, pre - 2), pre + impact_frames),
        frame_period_s=period_s,
        ops_bins_per_frame=ops_bins,
        ratio_band=FOLLOWS_BALL_RATIO,
    )
    info = {
        "file": path.name,
        "retention": retention["reason"] if retention else "fixed",
        "pre_frames": pre,
        "ops_ball_mph": ops_mph,
        "frame_period_s": period_s,
    }
    return info, line


def _summary(line: BallLine | None, period_s: float) -> dict:
    if line is None:
        return {"line_frames": 0}
    return {
        "launch_frame": line.impact_frame,
        "line_mph": line.bins_per_frame * BIN_M / period_s * MPS_TO_MPH,
        "line_frames": len(line.frames),
        "visible": sum(item.visible for item in line.frames),
        "candidate": sum(item.candidate for item in line.frames),
        "outranked": sum(item.visible and not item.candidate for item in line.frames),
        "gated_out": sum(item.gated_out for item in line.frames),
        "weak": sum(not item.visible and not item.gated_out for item in line.frames),
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("session", type=Path, help="session_*_range.jsonl")
    parser.add_argument(
        "--dumps", type=Path, help="dump directory (default: <session dir>/iwr6843)"
    )
    parser.add_argument("--config", help="IWR cfg (default: the one the session logged)")
    parser.add_argument("--tee-bin", type=int, help="default: the session's triggerCfg bin")
    parser.add_argument("--frames", action="store_true", help="print every frame on the ball line")
    parser.add_argument("--json", type=Path, help="write per-capture results here")
    args = parser.parse_args()

    iwr = _session_start(args.session)["config"]["iwr6843"]
    config = read_capture_config(args.config or iwr["config"])
    tee_bin = args.tee_bin
    if tee_bin is None:
        match = re.match(r"triggerCfg (\d+)", iwr.get("self_trigger") or "")
        if match is None:
            parser.error("session has no triggerCfg; pass --tee-bin")
        tee_bin = int(match.group(1))
    dumps = args.dumps or args.session.parent / "iwr6843"

    results = []
    for capture in _captures(args.session):
        path = dumps / Path(capture["capture_path"]).name
        if not path.is_file():
            print(f"shot {capture['shot_number']:2d}: {path.name} not found, skipped")
            continue
        info, line = _analyse(
            path,
            ops_mph=capture.get("ball_speed_mph"),
            tee_bin=tee_bin,
            pre_default=config.pre_frames,
            impact_frames=config.impact_frames,
        )
        summary = _summary(line, info["frame_period_s"])
        results.append({"shot": capture["shot_number"], **info, **summary})
        if line is None:
            print(
                f"shot {capture['shot_number']:2d} {info['retention']:10s} no ball line (too few rebuilt frames)"
            )
            continue
        print(
            f"shot {capture['shot_number']:2d} {info['retention']:10s} ops {info['ops_ball_mph']:5.1f} mph"
            f"  line f{summary['launch_frame']} {summary['line_mph']:5.1f} mph"
            f"  frames {summary['line_frames']:2d}: visible {summary['visible']:2d},"
            f" candidate {summary['candidate']:2d}, outranked {summary['outranked']:2d},"
            f" gated out {summary['gated_out']:2d}, weak {summary['weak']:2d}"
        )
        if args.frames:
            for item in line.frames:
                rank = item.ball_rank if item.ball_rank is not None else "-"
                print(
                    f"    f{item.frame:2d} {_phase(item.frame, info['pre_frames'], config.impact_frames):6s}"
                    f" bins {item.window[0]:3d}-{item.window[1] - 1:3d}  ball {item.ball_bin:3d}"
                    f"  snr {item.ball_snr:5.1f}  rank {rank}  top {','.join(map(str, item.top_peaks))}"
                )
        if args.json:
            results[-1]["frames"] = [asdict(item) for item in line.frames]

    totals = {
        key: sum(r.get(key, 0) for r in results)
        for key in ("line_frames", "visible", "candidate", "outranked", "gated_out", "weak")
    }
    print(
        f"\n{len(results)} captures, {totals['line_frames']} ball-line frames: visible {totals['visible']},"
        f" candidate {totals['candidate']}, outranked {totals['outranked']},"
        f" gated out {totals['gated_out']}, weak {totals['weak']}"
        "\n(ranks cover the retained bins only; peaks outside them could outrank the ball too)"
    )
    if any(result["retention"] != "fixed" for result in results):
        print(
            "WARNING: adaptive dumps rebuild only their impact frames, where the ball is still"
            " within ~0.5 m of the tee and shares range bins with the golfer. These counts"
            " cannot be attributed to the ball; use a fixed-window (diagnostic cfg) session."
        )
    if args.json:
        args.json.write_text(json.dumps(results, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
