"""Offline OV9281 club-motion measurements around impact.

The tracker intentionally reports image-plane motion only. Converting these
measurements to club path or attack angle requires camera-pose calibration and
an independent downrange velocity measurement.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np
from scipy import ndimage

BALL_DIAMETER_MM = 42.67


@dataclass(frozen=True)
class ReferenceBall:
    """Stationary ball location and apparent size before the swing."""

    x: float
    y: float
    diameter_px: float
    area_px: int


@dataclass(frozen=True)
class ImagePoint:
    """Tracked image coordinate for one camera frame."""

    frame_index: int
    x: float
    y: float


@dataclass(frozen=True)
class ImagePlaneMotion:
    """Terminal transverse motion measured in the camera image plane."""

    horizontal_px_s: float
    vertical_px_s: float
    horizontal_m_s: float
    vertical_m_s: float
    mm_per_px: float
    interval_ms: float


def detect_reference_ball(
    frames: np.ndarray,
    *,
    roi: tuple[int, int, int, int] | None = None,
    brightness_threshold: int = 210,
) -> ReferenceBall:
    """Find the stationary ball in bright- or dark-on-ground lighting."""
    if frames.ndim != 3 or frames.shape[0] < 3:
        raise ValueError("frames must have shape (n, height, width) with n >= 3")

    background = np.median(frames[: min(20, frames.shape[0])], axis=0)
    height, width = background.shape
    x0, y0, x1, y1 = roi or (0, 0, width, height)
    if not (0 <= x0 < x1 <= width and 0 <= y0 < y1 <= height):
        raise ValueError("ball ROI is outside the image")

    image_center = np.asarray((width / 2, height / 2))

    def components(  # pylint: disable=too-many-arguments
        mask: np.ndarray,
        *,
        min_area: int,
        aspect_limits: tuple[float, float],
        min_fill: float,
        diameter_from_extent: bool = False,
        contrast: np.ndarray | None = None,
    ) -> list[tuple[float, ReferenceBall]]:
        labels, count = ndimage.label(mask)
        found: list[tuple[float, ReferenceBall]] = []
        for label in range(1, count + 1):
            ys, xs = np.where(labels == label)
            area = len(xs)
            if not min_area <= area <= 600:
                continue
            component_width = int(np.ptp(xs)) + 1
            component_height = int(np.ptp(ys)) + 1
            aspect = component_width / component_height
            fill = area / (component_width * component_height)
            if not (aspect_limits[0] <= aspect <= aspect_limits[1] and fill >= min_fill):
                continue
            center = np.asarray((float(xs.mean()), float(ys.mean())))
            diameter = (
                float(max(component_width, component_height))
                if diameter_from_extent
                else math.sqrt(4 * area / math.pi)
            )
            strength = float(np.mean(contrast[ys, xs])) if contrast is not None else 0.0
            score = (
                float(np.linalg.norm(center - image_center))
                + abs(math.log(aspect)) * 4.0
                - strength * 0.05
            )
            found.append(
                (
                    score,
                    ReferenceBall(
                        x=float(center[0]),
                        y=float(center[1]),
                        diameter_px=diameter,
                        area_px=area,
                    ),
                )
            )
        return found

    # Prefer a clean white-ball component in the compact high-speed crop. The
    # dark-first order remains useful for spotlight-washed 640x400 indoor
    # scenes, but at 320x200 it can select dark foliage instead of the ball.
    bright_mask = np.zeros_like(background, dtype=bool)
    bright_mask[y0:y1, x0:x1] = background[y0:y1, x0:x1] >= brightness_threshold
    bright_candidates = components(
        bright_mask,
        min_area=20,
        aspect_limits=(0.55, 1.8),
        min_fill=0.45,
    )
    compact_capture = background.shape[0] <= 200 and background.shape[1] <= 320
    if compact_capture and bright_candidates:
        # Compact outdoor crops contain many saturated, round highlights in
        # the net and foliage. A regulation ball belongs in the central-lower
        # hitting zone and has a stable apparent diameter at tee distance.
        plausible = [
            item
            for item in bright_candidates
            if 9.0 <= item[1].diameter_px <= 30.0
            and width * 0.2 <= item[1].x <= width * 0.8
            and height * 0.45 <= item[1].y <= height * 0.9
        ]
        if plausible:
            return min(plausible, key=lambda item: item[0])[1]

    # A spotlight can wash the white face of the ball into the turf while its
    # lower silhouette remains dark. Local contrast is more stable than an
    # absolute dark threshold across indoor and outdoor exposure settings.
    dark_x0 = x0 if roi is not None else max(x0, int(width * 0.25))
    dark_x1 = x1 if roi is not None else min(x1, int(width * 0.75))
    dark_y0 = y0 if roi is not None else max(y0, int(height * 0.30))
    dark_y1 = y1 if roi is not None else min(y1, int(height * 0.70))
    blur_sigma = max(4.0, min(height, width) * 0.02)
    dark_contrast = ndimage.gaussian_filter(background, sigma=blur_sigma) - background
    dark_roi = dark_contrast[dark_y0:dark_y1, dark_x0:dark_x1]
    dark_threshold = max(20.0, float(np.percentile(dark_roi, 99.0)))
    dark_mask = np.zeros_like(background, dtype=bool)
    dark_mask[dark_y0:dark_y1, dark_x0:dark_x1] = dark_roi >= dark_threshold
    dark_candidates = components(
        dark_mask,
        min_area=8,
        aspect_limits=(0.25, 5.0),
        min_fill=0.25,
        diameter_from_extent=True,
        contrast=dark_contrast,
    )
    if dark_candidates:
        seed = min(dark_candidates, key=lambda item: item[0])[1]
        radius = max(8, int(math.ceil(seed.diameter_px * 1.5)))
        patch_x0 = max(dark_x0, int(round(seed.x)) - radius)
        patch_x1 = min(dark_x1, int(round(seed.x)) + radius + 1)
        patch_y0 = max(dark_y0, int(round(seed.y)) - radius)
        patch_y1 = min(dark_y1, int(round(seed.y)) + radius + 1)
        low_mask = dark_contrast[patch_y0:patch_y1, patch_x0:patch_x1] >= max(
            12.0, dark_threshold * 0.3
        )
        labels, _count = ndimage.label(low_mask)
        mask_ys, mask_xs = np.where(low_mask)
        if len(mask_xs):
            nearest = np.argmin(
                (mask_xs + patch_x0 - seed.x) ** 2 + (mask_ys + patch_y0 - seed.y) ** 2
            )
            selected_label = labels[mask_ys[nearest], mask_xs[nearest]]
            component_ys, component_xs = np.where(labels == selected_label)
            component_xs = component_xs + patch_x0
            component_ys = component_ys + patch_y0
            component_width = int(np.ptp(component_xs)) + 1
            component_height = int(np.ptp(component_ys)) + 1
            area = len(component_xs)
            if 8 <= area <= 600:
                return ReferenceBall(
                    x=float(component_xs.mean()),
                    y=float(component_ys.mean()),
                    diameter_px=float(max(component_width, component_height)),
                    area_px=area,
                )
        return seed

    if bright_candidates:
        return min(bright_candidates, key=lambda item: item[0])[1]
    raise ValueError("no stable reference ball found")


def image_plane_motion(
    points: Sequence[ImagePoint],
    timestamps_ns: np.ndarray,
    *,
    ball_diameter_px: float,
) -> ImagePlaneMotion:
    """Calculate terminal image-plane motion from the final adjacent points."""
    if len(points) < 2:
        raise ValueError("at least two tracked points are required")
    first, second = points[-2:]
    if second.frame_index != first.frame_index + 1:
        raise ValueError("terminal tracked points must be consecutive")
    if ball_diameter_px <= 0:
        raise ValueError("ball diameter must be positive")

    delta_s = (int(timestamps_ns[second.frame_index]) - int(timestamps_ns[first.frame_index])) / 1e9
    if delta_s <= 0:
        raise ValueError("frame timestamps must increase")
    horizontal_px_s = (second.x - first.x) / delta_s
    # Image y grows downward; physical vertical velocity uses the opposite sign.
    vertical_px_s = -(second.y - first.y) / delta_s
    mm_per_px = BALL_DIAMETER_MM / ball_diameter_px
    return ImagePlaneMotion(
        horizontal_px_s=horizontal_px_s,
        vertical_px_s=vertical_px_s,
        horizontal_m_s=horizontal_px_s * mm_per_px / 1000.0,
        vertical_m_s=vertical_px_s * mm_per_px / 1000.0,
        mm_per_px=mm_per_px,
        interval_ms=delta_s * 1000.0,
    )
