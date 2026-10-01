#!/usr/bin/env python3
"""How steady the reconstructed 3D trajectories are, over recorded captures.

    uv run python scripts/analysis/evaluate_trajectory_reconstruction.py \
        [--dir tests/radar/recordings] [--json out.json]

Every ``.l3dump`` in the manifest is replayed through the compiled firmware.
For each shot's ball and club track it measures the RMS perpendicular scatter
of the raw points (each frame's own angles) and of the reconstructed points
about their own best-fit 3D line, plus the ball's fitted HLA/VLA and why.
There are no angle labels, so this measures STABILITY, not accuracy: a
reconstruction that is steadily wrong scores well here.
"""

from __future__ import annotations

import argparse
import json
import statistics
from pathlib import Path

import numpy as np

from openflight.iwr6843 import firmware_replay as fr


def scatter_about_line(points) -> float | None:
    """RMS perpendicular distance of the points from their best-fit line; None under 3."""
    if len(points) < 3:
        return None
    xyz = np.asarray(points, dtype=float)
    centred = xyz - xyz.mean(axis=0)
    _, _, vt = np.linalg.svd(centred, full_matrices=False)
    along = centred @ vt[0]
    perpendicular = centred - np.outer(along, vt[0])
    return float(np.sqrt(np.mean(np.sum(perpendicular**2, axis=1))))


def _track_scatter(points) -> tuple[float | None, float | None]:
    """Scatter about a line of the raw and the reconstructed positions, over only the
    points that have a reconstruction (hence "ball compared" is small: the ball fit
    is invalid on most real shots)."""
    raw = [p.position for p in points if p.position is not None and p.filtered_position is not None]
    fitted = [p.filtered_position for p in points if p.filtered_position is not None]
    return scatter_about_line(raw), scatter_about_line(fitted)


def _median(values) -> float | None:
    values = [v for v in values if v is not None]
    return statistics.median(values) if values else None


def _summary(pairs) -> dict:
    compared = [(r, f) for r, f in pairs if r is not None and f is not None]
    return {
        "compared": len(compared),
        "raw_scatter_m_median": _median(r for r, _ in compared),
        "filtered_scatter_m_median": _median(f for _, f in compared),
    }


def evaluate(directory: Path) -> dict:
    shots = []
    for path, config in fr.recording_configs(directory):
        result = fr.replay_dump(path.read_bytes(), config)
        launch = result.launch
        ball_raw, ball_fit = _track_scatter(result.ball_points)
        club_raw, club_fit = _track_scatter(result.points)
        shots.append(
            {
                "name": path.name,
                "ball_raw_scatter_m": ball_raw,
                "ball_filtered_scatter_m": ball_fit,
                "club_raw_scatter_m": club_raw,
                "club_filtered_scatter_m": club_fit,
                "hla_deg": launch.hla_deg if launch else None,
                "vla_deg": launch.vla_deg if launch else None,
                "angle_why": launch.angle_why if launch else "no_launch",
                "angles_accepted": launch.angles_accepted if launch else 0,
                "speed_mps": launch.speed_mps if launch else None,
            }
        )
    whys: dict[str, int] = {}
    for shot in shots:
        whys[shot["angle_why"]] = whys.get(shot["angle_why"], 0) + 1
    return {
        "shots": len(shots),
        "ball": _summary((s["ball_raw_scatter_m"], s["ball_filtered_scatter_m"]) for s in shots),
        "club": _summary((s["club_raw_scatter_m"], s["club_filtered_scatter_m"]) for s in shots),
        "angle_why_counts": whys,
        "per_shot": shots,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dir", type=Path, default=Path(fr.RECORDINGS_DIR))
    parser.add_argument("--json", type=Path)
    args = parser.parse_args()
    report = evaluate(args.dir)
    text = json.dumps(report, indent=2)
    if args.json:
        args.json.write_text(text + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "per_shot"}, indent=2))


if __name__ == "__main__":
    main()
