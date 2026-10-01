"""Everything the dump viewer draws for one ``.l3dump``, as plain JSON.

``scripts/iwr6843/dump_viewer`` serves a page that plots a capture as a
range x time x power surface, timelines of the trigger decisions and a
per-frame inspector. This module does the work behind it so it can be tested
without a browser:

* the per-bin maps come from :func:`firmware_replay.bin_observations`, the
  exact ``l3_verticalResidual`` observations the R4F scores, so the colours
  on the page are the numbers the trigger saw;
* the firmware verdicts come from :func:`firmware_replay.replay_dump`, the
  compiled ``l3_*.c`` trigger, club track, impact, shot and ball tracker.

A failure in the replay (no C compiler, a raw-ADC dump the firmware path
cannot take) is reported in the result instead of hiding the rest.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import asdict, dataclass, fields
from pathlib import Path

import numpy as np

from openflight.iwr6843 import firmware_replay as fr, self_trigger as st
from openflight.iwr6843.calibration import DEFAULT_PITCH_DEG, antenna_range_m
from openflight.iwr6843.dump import is_range_snapshot, parse_dump, range_data
from openflight.iwr6843.firmware_host import OBS_WAVELENGTH_M
from openflight.iwr6843.tracking import RANGE_SPAN_M, same_tx_loop_period_s

_TRIGGER_CFG = re.compile(r"triggerCfg\s+(\d+)\s+([0-9.]+)\s+(\d+)")
# Floor for log power so empty bins plot instead of turning into -inf.
_POWER_FLOOR = 1.0


@dataclass(frozen=True)
class ViewerOptions:  # pylint: disable=too-many-instance-attributes
    """What the page lets you change before a run. None means "use the default"."""

    tee_bin: int | None = st.FIRMWARE_TRIGGER_DEFAULT_BIN  # None: from tee_range_m
    dest_bin: int | None = st.FIRMWARE_TRIGGER_DEFAULT_BIN
    snr: float = st.FIRMWARE_TRIGGER_DEFAULT_SNR  # the trigger's (triggerCfg)
    stat: str = "peak"
    subbin: str = "parabolic"
    post_from_frame: int | None = None
    stop_at_fire: bool = False
    pitch_deg: float = DEFAULT_PITCH_DEG
    tee_range_m: float = st.DEFAULT_TEE_RANGE_M
    ball_hypotheses: bool | None = None  # the ball search; None: the firmware default
    # The tee band's width, placed automatically; 0 turns it off.
    band_bins: float | None = st.TEE_BAND_DEFAULT_BINS
    ball_snr: float | None = st.FIRMWARE_BALL_DEFAULT_SNR  # the ball tracker's (trackCfg ballSnr)

    @classmethod
    def from_mapping(cls, raw: dict) -> ViewerOptions:
        """Build from query/JSON values; blank strings are "not set", unknown keys an error."""
        return cls(**cls._parse(raw))

    @classmethod
    def for_recording(cls, raw: dict) -> ViewerOptions:
        """Options to replay a recorded capture as the board ran it: what the
        session log says (``raw``, as ``from_mapping``) over the settings the
        recordings were made with (triggerCfg snr 6, ball snr 3, the tee bin
        from the slant range, no tee band), not today's defaults. The range
        gate that froze them is gone; the replay fires on the club track."""
        recorded = {
            "tee_bin": None,
            "dest_bin": None,
            "snr": fr.DEFAULT_SNR,
            "ball_snr": fr.DEFAULT_BALL_SNR,
            "band_bins": 0.0,
        }
        return cls(**{**recorded, **cls._parse(raw)})

    @classmethod
    def _parse(cls, raw: dict) -> dict:
        """The set values of ``raw`` converted to their field types."""
        known = {f.name: f for f in fields(cls)}
        unknown = sorted(set(raw) - set(known))
        if unknown:
            raise ValueError(f"unknown options: {', '.join(unknown)}")
        values: dict = {}
        for name, value in raw.items():
            if value is None or (isinstance(value, str) and not value.strip()):
                continue
            kind = str(known[name].type)
            if "bool" in kind:
                values[name] = (
                    value
                    if isinstance(value, bool)
                    else str(value).lower()
                    in (
                        "1",
                        "true",
                        "yes",
                        "on",
                    )
                )
            elif "int" in kind:
                values[name] = int(value)
            elif "float" in kind:
                values[name] = float(value)
            else:
                values[name] = str(value)
        return values


def bin_width_m(fft_size: int = fr.DEFAULT_FFT_SIZE) -> float:
    """Metres per range-FFT bin: every cfg keeps a 6 m span."""
    return RANGE_SPAN_M / fft_size


def tee_bin_for(options: ViewerOptions, fft_size: int = fr.DEFAULT_FFT_SIZE) -> int:
    """The firmware tee bin: the explicit one, else the tee's range from the array
    (``tee_range_m`` is measured from the enclosure front, as the session log
    holds it) rounded to a bin."""
    if options.tee_bin is not None:
        return options.tee_bin
    return int(round(antenna_range_m(options.tee_range_m) / bin_width_m(fft_size)))


def _finite(value: float | None) -> float | None:
    """JSON has no NaN or infinity; both become null."""
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _db(values: np.ndarray) -> np.ndarray:
    """Power in dB; at or below the floor (an empty bin, the MTI-cancelled zero
    Doppler bin) is NaN, so it plots as a gap rather than a pit."""
    values = np.asarray(values, dtype=float)
    with np.errstate(divide="ignore", invalid="ignore"):
        return np.where(values > _POWER_FLOOR, 10.0 * np.log10(values), np.nan)


def _grid(rows: list[np.ndarray], starts: list[int], bin0: int, width: int) -> list[list]:
    """Per-frame local rows on the global bins ``bin0 .. width - 1``, null where not stored."""
    out = []
    for row, start in zip(rows, starts, strict=True):
        line: list = [None] * (width - bin0)
        for local, value in enumerate(row):
            if bin0 <= start + local < width:
                line[start + local - bin0] = _finite(value)
        out.append(line)
    return out


def frame_maps(meta: dict, cube: np.ndarray) -> dict:
    """Range x time maps on a global-bin axis, plus each frame's range-Doppler map.

    ``mti_db``: the burst-MTI residual energy the trigger scores
    (``l3_verticalResidual`` energy). ``static_db``: the loop-mean return, the
    stationary scene (ball, tee, hands). ``velocity``: the lag-1 Doppler
    radial velocity per bin, aliased as the firmware reads it.
    ``range_doppler_db``: per frame, an FFT over the loops of the MTI
    residual, summed over the vertical channels.
    """
    ranged = range_data(meta, cube)
    n_frames, chirps, n_rx, _ = ranged.shape
    n_tx = int(meta["n_tx"])
    loops = chirps // n_tx
    loop_period_s = same_tx_loop_period_s(n_tx)
    windows = [fr.frame_window(meta, frame) for frame in range(n_frames)]
    if not is_range_snapshot(meta):
        # A raw-ADC dump range-FFTs to every bin of every frame.
        windows = [(0, ranged.shape[-1] // 2) for _ in range(n_frames)]
    bin0 = min(start for start, _ in windows)
    width = max(start + count for start, count in windows)
    tx = list(fr.vertical_tx_indices(n_tx))
    mti, static, velocity, doppler = [], [], [], []
    for frame, (start, count) in enumerate(windows):
        table = fr.bin_observation_table(ranged, frame, 0, count, n_tx)
        mti.append(_db(table["energy"].astype(float)))
        phase = np.arctan2(table["r1Im"], table["r1Re"]).astype(float)
        velocity.append(phase * OBS_WAVELENGTH_M / (4.0 * math.pi * loop_period_s))
        data = ranged[frame, :, :, :count].reshape(loops, n_tx, n_rx, count)[:, tx]
        static.append(_db((np.abs(data.mean(axis=0)) ** 2).sum(axis=(0, 1))))
        residual = data - data.mean(axis=0, keepdims=True)
        spectrum = np.fft.fftshift(np.fft.fft(residual, axis=0), axes=0)
        doppler.append(_db((np.abs(spectrum) ** 2).sum(axis=(1, 2))))
    starts = [start for start, _ in windows]
    velocity_span = OBS_WAVELENGTH_M / (4.0 * loop_period_s)
    return {
        "bin0": bin0,  # the global bin of every grid row's first column
        "width": width,  # one past the last global bin
        "starts": starts,
        "counts": [count for _, count in windows],
        "mti_db": _grid(mti, starts, bin0, width),
        "static_db": _grid(static, starts, bin0, width),
        "velocity_mps": _grid(velocity, starts, bin0, width),
        "range_doppler_db": [
            [[_finite(v) for v in row] for row in frame_map.T] for frame_map in doppler
        ],  # [frame][local bin][doppler bin]
        "doppler_axis_mps": [
            _finite(v) for v in np.linspace(-velocity_span, velocity_span, loops, endpoint=False)
        ],
        "velocity_span_mps": velocity_span,
    }


def _jsonable(value):
    """Dataclasses, tuples and floats from the replay made JSON-safe."""
    if hasattr(value, "__dataclass_fields__"):
        return {
            f.name: _jsonable(getattr(value, f.name))
            for f in fields(value)
            if f.repr  # the ctypes state is repr=False
        }
    if isinstance(value, dict):
        return {k: _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, float):
        return _finite(value)
    return value


# The drawn segment of a fitted track: from this long before its crossing.
FIT_LINE_SPAN_US = 12_000.0  # the K = 4 points at 3 ms frames


def _impact_fit_json(result: fr.ReplayResult) -> dict | None:
    """The impact fit with, per kept track, the fitted line as two (t_us, range_m) points.

    Every number is run through ``_finite`` before it reaches JSON: a track
    with a non-finite speed or time drops its line instead of poisoning the
    whole payload.
    """
    if result.impact_fit is None:
        return None
    ball_m = result.config.destination * bin_width_m(result.config.fft_size)
    out = _jsonable(result.impact_fit)
    for name, track in result.impact_fit.tracks.items():
        line = None
        if track.why == "ok" and track.time_us is not None:
            t0 = track.time_us - FIT_LINE_SPAN_US
            r0 = ball_m + track.speed_mps * (t0 - track.time_us) * 1e-6
            points = [_finite(t0), _finite(r0), _finite(track.time_us), _finite(ball_m)]
            if all(v is not None for v in points):
                t0, r0, t1, r1 = points
                line = [[t0, r0], [t1, r1]]
        out["tracks"][name]["line"] = line
    return out


def firmware_section(raw: bytes, meta: dict, cube: np.ndarray, options: ViewerOptions) -> dict:
    """The compiled firmware's replay, with each frame's watched-stat peak against its threshold."""
    config = fr.ReplayConfig(
        tee_bin=tee_bin_for(options),
        dest_bin=options.dest_bin,
        snr=options.snr,
        stat=options.stat,
        subbin=options.subbin,
        post_from_frame=options.post_from_frame,
        stop_at_fire=options.stop_at_fire,
        pitch_deg=options.pitch_deg,
        ball_hypotheses=options.ball_hypotheses,
        band_bins=options.band_bins,
        ball_snr=options.ball_snr,
    )
    result = fr.replay_dump(raw, config)
    n_tx = int(meta["n_tx"])
    watched = []
    for frame in result.frames:
        start, _ = fr.frame_window(meta, frame.frame)
        if frame.count <= 0:
            watched.append(None)
            continue
        table = fr.bin_observation_table(
            cube, frame.frame, frame.first_bin - start, frame.count, n_tx
        )
        values = table[options.stat].astype(float)
        best = int(np.argmax(values))
        watched.append({"bin": frame.first_bin + best, "stat": _finite(values[best])})
    return {
        "config": asdict(config),
        "frames": _jsonable(result.frames),
        "watched_peak": watched,
        "points": _jsonable(result.points),
        "ball_points": _jsonable(result.ball_points),
        "fired_frame": result.fired_frame,
        "impact_timestamp_us": result.impact_timestamp_us,
        "delivery": _jsonable(result.delivery),
        "launch": _jsonable(result.launch),
        "ball_angle": _jsonable(result.ball_angle),
        "speed_mps": _finite(result.speed_mps),
        "longest_run": result.longest_run,
        "acquisitions": result.acquisitions,
        "track_counters": result.track_counters,
        "trigger_summary": result.trigger_summary,
        "report": fr.format_report(result, points=True),
        "band": list(result.band) if result.band is not None else None,
        "range_frame": result.range_frame,
        "ball_range_m": _finite(result.config.destination * bin_width_m(result.config.fft_size)),
        "impact_fit": _impact_fit_json(result),
    }


def _guarded(builder, *args) -> dict:
    try:
        return {"ok": True, **builder(*args)}
    except Exception as exc:  # pylint: disable=broad-exception-caught
        # One broken replay must not blank the page: report it where it would draw.
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}"}


def analyze_dump(raw: bytes, options: ViewerOptions | None = None) -> dict:
    """The whole page's data for one capture."""
    options = options or ViewerOptions()
    meta, cube = parse_dump(raw)
    timestamps = fr.frame_timestamps_us(meta)
    width_m = bin_width_m()
    header = {
        k: v
        for k, v in meta.items()
        if k not in ("range_bin_starts", "range_bin_counts", "frame_time_offsets_us", "iq8_scales")
    }
    return {
        "meta": _jsonable(header),
        "n_frames": int(meta["n_frames"]),
        "timestamps_ms": [t / 1000.0 for t in timestamps],
        "bin_width_m": width_m,
        # A saved capture's pre/post boundary: the frame the board froze on.
        "freeze_frame": fr.freeze_frame(meta),
        "tee_bin": tee_bin_for(options),
        "options": asdict(options),
        "maps": frame_maps(meta, cube),
        "firmware": _guarded(firmware_section, raw, meta, cube, options),
    }


def parse_trigger_cfg(text: str | None) -> dict:
    """``triggerCfg <bin> <snr> <on>`` -> viewer options, {} when absent or off."""
    match = _TRIGGER_CFG.search(text or "")
    if not match or int(match.group(3)) == 0:
        return {}
    return {"tee_bin": int(match.group(1)), "snr": float(match.group(2))}


def _read_jsonl(path: Path) -> list[dict]:
    rows = []
    with path.open(encoding="utf-8") as handle:
        for line in handle:
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue  # a torn last line from a killed session
    return rows


def manifest_options(dump_path: Path) -> dict:
    """The options the folder's ``manifest.json`` sets for this dump, as the tests replay it.

    Only keys the manifest actually gives (default merged with the file's own
    entry) that are also viewer options; ``null`` means "not set". Empty when
    there is no manifest or it yields no tee bin.
    """
    try:
        entry = fr.recording_entry(dump_path)
    except ValueError:
        return {}
    known = {f.name for f in fields(ViewerOptions)}
    return {k: v for k, v in entry.items() if k in known and v is not None}


def session_context(dump_path: Path, search_dirs: list[Path] | None = None) -> dict | None:
    """The session log entry that saved this capture, and the options it implies.

    Looks through ``*.jsonl`` in the dump's folder, its parent and the
    parent's ``session_logs`` (or ``search_dirs``) for an ``iwr6843_capture``
    whose ``capture_path`` names this file. The folder's manifest options
    (``manifest_options``) override the session's for the keys it sets, and
    alone make a context (``session_file`` None) when no session mentions the
    dump. Returns None when neither exists.
    """
    context = _session_entry(dump_path, search_dirs)
    manifest = manifest_options(dump_path)
    if not manifest:
        return context
    if context is None:
        context = {"session_file": None, "defaults": {}}
    context["defaults"] = {**context["defaults"], **manifest}
    return context


def _session_entry(dump_path: Path, search_dirs: list[Path] | None) -> dict | None:
    """The session log's view of this capture (``session_context`` without the manifest)."""
    name = dump_path.name
    dirs = search_dirs or [
        dump_path.parent,
        dump_path.parent.parent,
        dump_path.parent.parent / "session_logs",  # the trackman export layout
    ]
    for directory in dirs:
        if not directory.is_dir():
            continue
        for log in sorted(directory.glob("*.jsonl")):
            if name not in log.read_text(encoding="utf-8", errors="replace"):
                continue
            rows = _read_jsonl(log)
            capture = next(
                (
                    r
                    for r in rows
                    if r.get("type") == "iwr6843_capture"
                    and Path(str(r.get("capture_path", ""))).name == name
                ),
                None,
            )
            if capture is None:
                continue
            start = next((r for r in rows if r.get("type") == "session_start"), {})
            iwr = (start.get("config") or {}).get("iwr6843") or {}
            defaults = parse_trigger_cfg(iwr.get("self_trigger"))
            if iwr.get("tee_slant_range_m") is not None:
                defaults["tee_range_m"] = float(iwr["tee_slant_range_m"])
            if iwr.get("tilt_deg") is not None:
                defaults["pitch_deg"] = float(iwr["tilt_deg"])
            shot = next(
                (
                    r
                    for r in rows
                    if r.get("type") == "shot_detected"
                    and r.get("shot_number") == capture.get("shot_number")
                ),
                None,
            )
            return {
                "session_file": log.name,
                "trigger_type": start.get("trigger_type"),
                "cfg": iwr.get("config"),
                "self_trigger": iwr.get("self_trigger"),
                "shot_number": capture.get("shot_number"),
                "ball_speed_mph": capture.get("ball_speed_mph"),
                "trigger_delta_ms": capture.get("trigger_delta_ms"),
                "capture_error": capture.get("capture_error"),
                "measurement": capture.get("measurement"),
                "shot": _jsonable(shot) if shot else None,
                "defaults": defaults,
            }
    return None


__all__ = [
    "ViewerOptions",
    "analyze_dump",
    "bin_width_m",
    "firmware_section",
    "frame_maps",
    "parse_trigger_cfg",
    "session_context",
    "tee_bin_for",
]
