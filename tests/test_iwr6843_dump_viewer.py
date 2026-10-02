"""Tests for the dump viewer's data layer and its Flask front end.

``openflight.iwr6843.dump_viewer`` turns one ``.l3dump`` into the JSON the
page plots; ``scripts/iwr6843/dump_viewer.py`` serves it. The maps must be
the firmware's own observations on a global-bin axis, the replays must
survive each other's failure, the session log must fill the options in, and
the server must only read captures under its folder.
"""

from __future__ import annotations

import dataclasses
import importlib.util
import io
import json
import math
import re
from html.parser import HTMLParser
from pathlib import Path

import numpy as np
import pytest
from iwr6843_synth import synth_shot_dump

from openflight.iwr6843 import dump_viewer as dv, firmware_host as fw, firmware_replay as fr
from openflight.iwr6843.dump import (
    SAMPLE_INT16_IQ,
    SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
    pack_dump,
    parse_dump,
    synth_target,
)

SCRIPT = Path(__file__).parents[1] / "scripts" / "iwr6843" / "dump_viewer.py"
TEE_RANGE_M = 1.372
TEE_BIN = int(TEE_RANGE_M / (6.0 / 128))  # 29, as the firmware replay tests use

needs_compiler = pytest.mark.skipif(
    fw.host_compiler() is None, reason="no C compiler for the firmware modules"
)


def _variable_dump(
    *,
    starts=(20, 20, 22, 22),
    counts=(16, 16, 12, 12),
    n_tx=2,
    loops=4,
    n_rx=4,
    width=16,
    moving_local=5,
    static_local=9,
) -> bytes:
    """A timed variable-width IQ16 snapshot: a mover and a static return."""
    frames = len(starts)
    cube = np.zeros((frames, loops * n_tx, n_rx, width), dtype=complex)
    for loop in range(loops):
        # A whole cycle over the 4 loops: burst-MTI mean removal leaves it unbiased.
        doppler = np.exp(1j * (math.pi / 2) * loop)
        cube[:, loop * n_tx : (loop + 1) * n_tx, :, moving_local] = 2000.0 * doppler
    cube[..., static_local] += 3000.0
    for frame, count in enumerate(counts):
        cube[frame, ..., count:] = 0.0
    return pack_dump(
        cube,
        n_tx=n_tx,
        version=6,
        sample_fmt=SAMPLE_RANGE_FFT_IQ16_VARIABLE_TIMED,
        range_bin_starts=starts,
        range_bin_counts=counts,
        frame_time_offsets_us=[2000 * f for f in range(frames)],
    )


# --- options ---------------------------------------------------------------


def test_default_pitch_is_ten_degrees():
    """The page's /api/defaults fill pitch from ViewerOptions; 0 silently zeros VLA."""
    assert dv.ViewerOptions().pitch_deg == pytest.approx(10.0)


def test_default_tee_and_dest_bins_are_thirty_two():
    options = dv.ViewerOptions()
    assert options.tee_bin == 32
    assert options.dest_bin == 32


def test_options_coerce_form_strings_and_skip_blanks():
    options = dv.ViewerOptions.from_mapping(
        {
            "tee_bin": "41",
            "dest_bin": "",
            "snr": "6.5",
            "stat": "energy",
            "stop_at_fire": False,
            "post_from_frame": None,
            "tee_range_m": " ",
        }
    )
    assert options.tee_bin == 41
    assert options.dest_bin == 32  # blank means unset → form default
    assert options.snr == 6.5
    assert options.stat == "energy"
    assert options.stop_at_fire is False
    assert options.post_from_frame is None
    assert options.tee_range_m == dv.ViewerOptions().tee_range_m


@pytest.mark.parametrize("text", ["off", "0", "no", ""])
def test_options_read_falsy_checkbox_strings_as_false(text):
    assert dv.ViewerOptions.from_mapping({"stop_at_fire": text}).stop_at_fire is False


def test_the_gate_options_are_gone():
    """The range gate, its track-frame count and the geometric detector were
    removed (2026-09-30)."""
    for gone in ("track_frames", "fire_mode", "impact_armed", "geometry_armed"):
        with pytest.raises(ValueError, match=f"unknown options: {gone}"):
            dv.ViewerOptions.from_mapping({gone: "1"})


def test_options_reject_unknown_keys_and_bad_numbers():
    with pytest.raises(ValueError, match="unknown options: bogus"):
        dv.ViewerOptions.from_mapping({"bogus": 1})
    with pytest.raises(ValueError):
        dv.ViewerOptions.from_mapping({"tee_bin": "forty"})


def test_tee_bin_is_explicit_or_the_rounded_slant_range_when_cleared():
    assert dv.tee_bin_for(dv.ViewerOptions(tee_bin=12)) == 12
    # The tee distance is what was measured from the enclosure face (and logged):
    # the array sits ARRAY_DEPTH_M behind it, and the bin is range from the array.
    assert dv.tee_bin_for(dv.ViewerOptions(tee_bin=None, tee_range_m=1.845)) == 40
    assert dv.tee_bin_for(dv.ViewerOptions(tee_bin=None, tee_range_m=1.524)) == 33
    assert dv.bin_width_m() == pytest.approx(6.0 / 128)


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("triggerCfg 41 6.0 2", {"tee_bin": 41, "snr": 6.0}),
        ("  triggerCfg 7 3 1 ", {"tee_bin": 7, "snr": 3.0}),
        ("triggerCfg 0 0 0", {}),  # SELF_TRIGGER_OFF_COMMAND
        (None, {}),
        ("", {}),
        ("sensorStop", {}),
    ],
)
def test_parse_trigger_cfg(text, expected):
    assert dv.parse_trigger_cfg(text) == expected


# --- maps ------------------------------------------------------------------


def test_maps_sit_on_the_global_bins_the_frames_stored():
    raw = _variable_dump()
    meta, cube = parse_dump(raw)
    maps = dv.frame_maps(meta, cube)

    assert maps["bin0"] == 20
    assert maps["width"] == 36  # frames 0-1 store 20..35
    assert maps["starts"] == [20, 20, 22, 22]
    assert maps["counts"] == [16, 16, 12, 12]
    for key in ("mti_db", "static_db", "velocity_mps"):
        assert len(maps[key]) == 4
        assert all(len(row) == 16 for row in maps[key])
    # Frames 2-3 start at 22: columns 0-1 (bins 20-21) were not stored.
    assert maps["static_db"][2][:2] == [None, None]
    assert maps["static_db"][2][2] is None or isinstance(maps["static_db"][2][2], float)
    # ... and neither were bins 34-35.
    assert maps["static_db"][2][14:] == [None, None]


def test_the_mover_is_in_the_mti_map_and_the_static_return_is_not():
    raw = _variable_dump()
    meta, cube = parse_dump(raw)
    maps = dv.frame_maps(meta, cube)
    mover, static = 20 + 5 - maps["bin0"], 20 + 9 - maps["bin0"]
    row = maps["mti_db"][0]
    assert row[mover] is not None and row[mover] > 60
    # Burst MTI removes a constant return exactly: a gap, not a -inf pit.
    assert row[static] is None
    # ... and the loop mean keeps the static return, which a whole-cycle mover averages out of.
    assert maps["static_db"][0][static] > 60
    assert maps["static_db"][0][mover] is None


def test_mti_energy_is_the_firmware_observation():
    raw = _variable_dump()
    meta, cube = parse_dump(raw)
    maps = dv.frame_maps(meta, cube)
    table = fr.bin_observation_table(cube, 0, 0, 16, 2)
    expected = 10 * math.log10(float(table["energy"][5]))
    assert maps["mti_db"][0][5] == pytest.approx(expected, abs=1e-3)


def test_lag1_velocity_reads_the_per_loop_phase_step():
    raw = _variable_dump()
    meta, cube = parse_dump(raw)
    maps = dv.frame_maps(meta, cube)
    loop_period_s = fr.same_tx_loop_period_s(2)
    expected = (math.pi / 2) * fw.OBS_WAVELENGTH_M / (4 * math.pi * loop_period_s)
    assert maps["velocity_mps"][0][5] == pytest.approx(expected, rel=1e-3)
    assert maps["velocity_span_mps"] == pytest.approx(fw.OBS_WAVELENGTH_M / (4 * loop_period_s))


def test_range_doppler_peaks_in_the_movers_doppler_bin():
    raw = _variable_dump()
    meta, cube = parse_dump(raw)
    maps = dv.frame_maps(meta, cube)
    rd = maps["range_doppler_db"][0]  # [local bin][doppler bin]
    assert len(rd) == 16 and len(rd[5]) == 4
    axis = maps["doppler_axis_mps"]
    assert len(axis) == 4 and axis[0] < 0 < axis[-1]
    values = [v if v is not None else -1 for v in rd[5]]
    assert axis[int(np.argmax(values))] > 0  # +pi/2 per loop is a positive velocity


def test_raw_adc_dumps_map_every_positive_range_bin():
    cube = synth_target(0.0, 10, n_frames=3, n_samples=32)
    raw = pack_dump(cube, n_tx=2, version=3, frame_period_us=3000, sample_fmt=SAMPLE_INT16_IQ)
    meta, parsed = parse_dump(raw)
    maps = dv.frame_maps(meta, parsed)
    assert maps["bin0"] == 0 and maps["width"] == 16
    assert maps["static_db"][0][10] == max(v for v in maps["static_db"][0] if v is not None)


# --- analyze ----------------------------------------------------------------


def test_analyze_is_strict_json_and_reports_a_failed_replay_in_place(monkeypatch):
    def broken(*_args, **_kwargs):
        raise RuntimeError("no host C compiler")

    monkeypatch.setattr(dv.fr, "replay_dump", broken)
    data = dv.analyze_dump(_variable_dump(), dv.ViewerOptions(tee_bin=25, tee_range_m=1.2))
    json.dumps(data, allow_nan=False)
    assert data["firmware"] == {"ok": False, "error": "RuntimeError: no host C compiler"}
    assert "python_trigger" not in data, "the host ball-leave detector was removed"
    assert data["n_frames"] == 4
    assert data["timestamps_ms"] == [0.0, 2.0, 4.0, 6.0]
    assert data["tee_bin"] == 25
    # No retention report before v8: the plan's window switch (frame 2) is the freeze.
    assert data["freeze_frame"] == 2


def test_analyze_reports_the_retention_boundary_as_the_freeze(monkeypatch):
    monkeypatch.setattr(
        dv,
        "parse_dump",
        lambda raw: (
            {**parse_dump(raw)[0], "retention": {"reason": "complete", "pre_frames": 3}},
            parse_dump(raw)[1],
        ),
    )
    data = dv.analyze_dump(_variable_dump())
    assert data["freeze_frame"] == 3


def test_freeze_frame_without_retention_is_where_the_plan_switches_windows():
    """Older captures (2026-08-24) carry no retention report; the capture
    plan's first post-impact window is the frame the sound trigger froze on,
    so "post = freeze frame" must still work on them."""
    data = dv.analyze_dump(
        _variable_dump(starts=(20, 20, 20, 32, 32, 47), counts=(16,) * 6), dv.ViewerOptions()
    )
    assert data["freeze_frame"] == 3


def test_freeze_frame_is_unknown_for_a_single_window_capture():
    data = dv.analyze_dump(_variable_dump(starts=(20,) * 4, counts=(16,) * 4), dv.ViewerOptions())
    assert data["freeze_frame"] is None


def test_raw_adc_dump_keeps_the_maps_when_the_firmware_path_refuses_it():
    cube = synth_target(0.0, 10, n_frames=3, n_samples=32)
    raw = pack_dump(cube, n_tx=2, version=3, frame_period_us=3000, sample_fmt=SAMPLE_INT16_IQ)
    data = dv.analyze_dump(raw, dv.ViewerOptions(tee_bin=10))
    json.dumps(data, allow_nan=False)
    assert data["firmware"]["ok"] is False
    assert "range-FFT snapshot" in data["firmware"]["error"]
    assert data["maps"]["width"] == 16


@needs_compiler
def test_a_whole_shot_carries_the_gate_the_tracks_and_their_3d_points():
    raw = synth_shot_dump(
        path_deg=3.0, hla_deg=2.0, vla_deg=12.0, ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M
    )
    data = dv.analyze_dump(raw, dv.ViewerOptions(tee_bin=TEE_BIN, tee_range_m=TEE_RANGE_M))
    json.dumps(data, allow_nan=False)
    firmware = data["firmware"]
    assert firmware["ok"], firmware.get("error")
    assert firmware["fired_frame"] is not None
    assert firmware["points"] and firmware["ball_points"]
    assert firmware["launch"] is not None
    assert len(firmware["watched_peak"]) == len(firmware["frames"])
    fired = firmware["frames"][firmware["fired_frame"]]
    assert fired["fired"] is True
    # The frame that fired still reports what the trigger watched. The removed
    # range gate fired on that peak (above snr x floor); the club track fires
    # from its own span, and the scan plan clips the trigger's region to short
    # of the band (l3_scan.h), where it is the floor's source only.
    assert firmware["watched_peak"][firmware["fired_frame"]] is not None
    for point in firmware["points"] + firmware["ball_points"]:
        assert len(point["position"]) == 3
    assert "clubtrack" in firmware["report"]


@needs_compiler
def test_a_whole_shot_carries_the_band_and_the_impact_fit():
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M)
    data = dv.analyze_dump(
        raw,
        dv.ViewerOptions(tee_bin=TEE_BIN, dest_bin=TEE_BIN, tee_range_m=TEE_RANGE_M, band_bins=6.0),
    )
    json.dumps(data, allow_nan=False)
    firmware = data["firmware"]
    assert firmware["ok"], firmware.get("error")
    lo, hi = firmware["band"]
    assert lo < TEE_BIN < hi
    fit = firmware["impact_fit"]
    assert fit is not None and fit["verdict"] in fw.FIT_VERDICT_NAMES
    ball_m = firmware["ball_range_m"]
    assert ball_m == pytest.approx(TEE_BIN * 6.0 / 128)
    drawn = 0
    for track in fit["tracks"].values():
        if track["why"] != "ok":
            assert track["line"] is None
            continue
        (t0, r0), (t1, r1) = track["line"]
        assert t1 == pytest.approx(track["time_us"])
        assert r1 == pytest.approx(ball_m)
        assert r0 == pytest.approx(ball_m + track["speed_mps"] * (t0 - t1) * 1e-6)
        drawn += 1
    assert drawn >= 1


def test_default_options_are_the_boards():
    """Tee and dest bin 32, trigger and ball snr 1 on the peak statistic, a 6-bin band."""
    options = dv.ViewerOptions()
    assert options.tee_bin == 32
    assert options.dest_bin == 32
    assert options.snr == 1.0
    assert options.stat == "peak"
    assert options.band_bins == 6.0
    assert options.ball_snr == 1.0
    assert dv.tee_bin_for(options) == 32


@needs_compiler
def test_default_options_place_the_band():
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M)
    data = dv.analyze_dump(raw, dv.ViewerOptions(tee_bin=TEE_BIN, tee_range_m=TEE_RANGE_M))
    assert data["firmware"]["band"] is not None


@needs_compiler
def test_band_zero_turns_the_band_off():
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M)
    options = dv.ViewerOptions(tee_bin=TEE_BIN, tee_range_m=TEE_RANGE_M, band_bins=0.0)
    assert dv.analyze_dump(raw, options)["firmware"]["band"] is None


def test_ball_snr_reaches_the_replay(monkeypatch):
    seen = {}

    def capture(_raw, config):
        seen["config"] = config
        raise RuntimeError("stop here")

    monkeypatch.setattr(dv.fr, "replay_dump", capture)
    dv.analyze_dump(_variable_dump(), dv.ViewerOptions.from_mapping({"ball_snr": "4.5"}))
    assert seen["config"].ball_snr == 4.5
    assert seen["config"].snr == 1.0


@needs_compiler
def test_impact_fit_json_stays_json_safe_when_a_track_speed_is_non_finite():
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M)
    config = fr.ReplayConfig(tee_bin=TEE_BIN, dest_bin=TEE_BIN, band_bins=6.0)
    result = fr.replay_dump(raw, config)
    ok_name = next(name for name, t in result.impact_fit.tracks.items() if t.why == "ok")
    broken = dataclasses.replace(result.impact_fit.tracks[ok_name], speed_mps=float("nan"))
    tracks = dict(result.impact_fit.tracks)
    tracks[ok_name] = broken
    result = dataclasses.replace(
        result, impact_fit=dataclasses.replace(result.impact_fit, tracks=tracks)
    )

    out = dv._impact_fit_json(result)
    json.dumps(out, allow_nan=False)
    assert out["tracks"][ok_name]["line"] is None


# --- session context ---------------------------------------------------------


def _write_session(path: Path, capture_name: str, *, torn: bool = False) -> None:
    rows = [
        {
            "type": "session_start",
            "trigger_type": "sound",
            "config": {
                "iwr6843": {
                    "config": "config/x.cfg",
                    "self_trigger": "triggerCfg 41 6.0 2",
                    "tee_slant_range_m": 1.845,
                    "tilt_deg": 10.4,
                }
            },
        },
        {
            "type": "iwr6843_capture",
            "shot_number": 2,
            "capture_path": f"/home/pi/openflight_sessions/iwr6843/{capture_name}",
            "ball_speed_mph": 53.1,
            "trigger_delta_ms": -462.8,
            "capture_error": "adaptive retention stopped: ambiguous",
        },
        {"type": "shot_detected", "shot_number": 1, "club_speed_mph": 10.0},
        {"type": "shot_detected", "shot_number": 2, "club_speed_mph": 43.0},
    ]
    text = "\n".join(json.dumps(r) for r in rows) + "\n"
    if torn:
        text += '{"type": "session_end", "ts'
    path.write_text(text, encoding="utf-8")


def test_session_context_finds_the_log_in_the_parent_folder(tmp_path):
    dumps = tmp_path / "iwr6843"
    dumps.mkdir()
    dump = dumps / "iwr6843_x_002.l3dump"
    dump.write_bytes(b"")
    _write_session(tmp_path / "session_a.jsonl", dump.name, torn=True)
    (tmp_path / "unrelated.jsonl").write_text('{"type": "session_start"}\n', encoding="utf-8")

    context = dv.session_context(dump)

    assert context["session_file"] == "session_a.jsonl"
    assert context["trigger_type"] == "sound"
    assert context["ball_speed_mph"] == 53.1
    assert context["shot"]["club_speed_mph"] == 43.0  # shot 2, not shot 1
    assert context["defaults"] == {
        "tee_bin": 41,
        "snr": 6.0,
        "tee_range_m": 1.845,
        "pitch_deg": 10.4,
    }
    dv.ViewerOptions.from_mapping(context["defaults"])  # the page can post them back


def test_session_context_is_none_without_a_matching_capture(tmp_path):
    dump = tmp_path / "iwr6843_y_001.l3dump"
    dump.write_bytes(b"")
    # Mentions the name only as a substring of another capture.
    _write_session(tmp_path / "session_b.jsonl", "iwr6843_y_001.l3dump.bak")
    assert dv.session_context(dump) is None


# --- server ------------------------------------------------------------------


@pytest.fixture(scope="module")
def viewer_module():
    spec = importlib.util.spec_from_file_location("dump_viewer_script", SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture
def client(viewer_module, tmp_path):
    (tmp_path / "sub").mkdir()
    (tmp_path / "sub" / "a.l3dump").write_bytes(_variable_dump())
    (tmp_path / "notes.txt").write_text("not a capture", encoding="utf-8")
    (tmp_path.parent / f"{tmp_path.name}-outside.l3dump").write_bytes(_variable_dump())
    return viewer_module.create_app(tmp_path).test_client()


def test_server_lists_captures_recursively(client):
    body = client.get("/api/files").get_json()
    assert [f["path"] for f in body["files"]] == ["sub/a.l3dump"]
    assert body["files"][0]["bytes"] > 0


def test_server_serves_the_default_options(client):
    """The page fills its form from these, so the defaults live in one place."""
    body = client.get("/api/defaults").get_json()
    assert body == dataclasses.asdict(dv.ViewerOptions())
    assert (body["tee_bin"], body["dest_bin"], body["snr"], body["stat"], body["band_bins"]) == (
        38,
        38,
        1.0,
        "peak",
        6.0,
    )
    assert body["pitch_deg"] == pytest.approx(10.0)


def test_page_has_a_box_for_every_option(client):
    page = client.get("/").data.decode()
    for name in dataclasses.asdict(dv.ViewerOptions()):
        assert f'id="{name}"' in page, name


def test_server_serves_the_page(client):
    response = client.get("/")
    assert response.status_code == 200
    assert b"L3 Dump Viewer" in response.data


def test_server_analyzes_a_listed_capture(client, monkeypatch):
    monkeypatch.setattr(dv.fr, "replay_dump", _raise)
    response = client.post(
        "/api/analyze", json={"path": "sub/a.l3dump", "options": {"tee_bin": "25"}}
    )
    assert response.status_code == 200, response.get_json()
    body = json.loads(response.data)
    assert body["name"] == "a.l3dump"
    assert body["context"] is None
    assert body["tee_bin"] == 25


@pytest.mark.parametrize(
    "path",
    ["../outside.l3dump", "notes.txt", "sub/missing.l3dump", "", "/etc/passwd"],
)
def test_server_refuses_paths_outside_the_folder_or_not_captures(client, tmp_path, path):
    if path == "../outside.l3dump":
        path = f"../{tmp_path.name}-outside.l3dump"
    response = client.post("/api/analyze", json={"path": path})
    assert response.status_code == 400
    assert "not a capture" in response.get_json()["error"]
    assert client.get("/api/context", query_string={"path": path}).status_code == 400


def test_server_analyzes_an_upload(client, monkeypatch):
    monkeypatch.setattr(dv.fr, "replay_dump", _raise)
    response = client.post(
        "/api/analyze",
        data={
            "file": (io.BytesIO(_variable_dump()), "dropped.l3dump"),
            "options": json.dumps({"tee_bin": 25}),
        },
        content_type="multipart/form-data",
    )
    assert response.status_code == 200, response.get_json()
    assert response.get_json()["name"] == "dropped.l3dump"


def test_server_reports_bad_options_and_bad_dumps_as_400(client):
    response = client.post("/api/analyze", json={"path": "sub/a.l3dump", "options": {"x": 1}})
    assert response.status_code == 400
    assert "unknown options" in response.get_json()["error"]
    response = client.post(
        "/api/analyze",
        data={"file": (io.BytesIO(b"NOPE" + bytes(40)), "bad.l3dump")},
        content_type="multipart/form-data",
    )
    assert response.status_code == 400
    assert "bad magic" in response.get_json()["error"]


def _raise(*_args, **_kwargs):
    raise RuntimeError("firmware replay disabled in this test")


def test_session_context_finds_a_log_in_a_sibling_session_logs_folder(tmp_path):
    """The trackman export layout: <day>/openflight/{iwr6843,session_logs}/."""
    dumps = tmp_path / "iwr6843"
    logs = tmp_path / "session_logs"
    dumps.mkdir()
    logs.mkdir()
    dump = dumps / "iwr6843_z_001.l3dump"
    dump.write_bytes(b"")
    _write_session(logs / "session_c_trackman.jsonl", dump.name)
    context = dv.session_context(dump)
    assert context is not None and context["session_file"] == "session_c_trackman.jsonl"


@needs_compiler
def test_the_viewer_can_switch_the_ball_search_and_shows_the_hypotheses():
    raw = synth_shot_dump(
        path_deg=3.0, hla_deg=2.0, vla_deg=12.0, ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M
    )
    options = dv.ViewerOptions.from_mapping({"tee_bin": TEE_BIN, "ball_hypotheses": "true"})
    assert options.ball_hypotheses is True
    data = dv.analyze_dump(raw, options)
    json.dumps(data, allow_nan=False)
    frames = data["firmware"]["frames"]
    assert any(frame["ball_hypotheses"] for frame in frames)
    first = next(frame["ball_hypotheses"] for frame in frames if frame["ball_hypotheses"])
    assert {"id", "points"} <= set(first[0])


# --- CSS colour tokens ------


def _css_tokens() -> dict[str, str]:
    html = (Path(__file__).parents[1] / "scripts" / "iwr6843" / "dump_viewer.html").read_text(
        encoding="utf-8"
    )
    return dict(re.findall(r"--([a-z-]+):\s*(#[0-9a-fA-F]{6})", html))


def test_the_ball_is_blue_and_nothing_else_uses_that_blue():
    tokens = _css_tokens()
    assert tokens["ball"].lower() == "#339af0"
    others = {name: value.lower() for name, value in tokens.items() if name != "ball"}
    assert "#339af0" not in others.values(), others


def test_the_page_has_no_host_ball_leave_panel():
    """The host ball-leave detector was removed (2026-09-30)."""
    html = (Path(__file__).parents[1] / "scripts" / "iwr6843" / "dump_viewer.html").read_text(
        encoding="utf-8"
    )
    assert "py" not in _css_tokens()
    for gone in ("python_trigger", "py_level", "py_hits", "Host ball-leave"):
        assert gone not in html, gone


# --- labels ------------------------------------------------------------------


def _label_payload(**over):
    body = {
        "version": 1,
        "reviewed": True,
        "ball": {"points": [{"frame": 2, "range_bin": 41.5, "doppler_mps": 30.0}]},
        "club": {"points": []},
        "tolerances": {"range_bins": 1.0, "min_coverage": 0.8},
        "notes": "n",
    }
    body.update(over)
    return body


def test_get_labels_without_a_file_is_an_empty_unreviewed_template(client):
    body = client.get("/api/labels", query_string={"path": "sub/a.l3dump"}).get_json()
    assert body["dump"] == "a.l3dump"
    assert body["reviewed"] is False
    assert body["ball"] == {"points": []} and body["club"] == {"points": []}
    assert len(body["dump_sha256"]) == 64


def test_put_labels_saves_a_sidecar_next_to_the_dump_and_get_returns_it(client, tmp_path):
    response = client.put(
        "/api/labels", query_string={"path": "sub/a.l3dump"}, json=_label_payload()
    )
    assert response.status_code == 200, response.get_json()
    assert (tmp_path / "sub" / "a.l3dump.labels.json").is_file()
    body = client.get("/api/labels", query_string={"path": "sub/a.l3dump"}).get_json()
    assert body["ball"]["points"] == [{"frame": 2, "range_bin": 41.5, "doppler_mps": 30.0}]
    assert body["reviewed"] is True
    # The sidecar is not itself listed as a capture.
    assert [f["path"] for f in client.get("/api/files").get_json()["files"]] == ["sub/a.l3dump"]


def test_put_labels_ignores_a_stale_client_hash(client):
    body = _label_payload(dump="x.l3dump", dump_sha256="stale")
    response = client.put("/api/labels", query_string={"path": "sub/a.l3dump"}, json=body)
    assert response.status_code == 200
    assert response.get_json()["dump"] == "a.l3dump"


def test_put_labels_refuses_a_frame_beyond_the_dump(client, tmp_path):
    body = _label_payload(ball={"points": [{"frame": 4, "range_bin": 41.0}]})  # 4 frames: 0..3
    response = client.put("/api/labels", query_string={"path": "sub/a.l3dump"}, json=body)
    assert response.status_code == 400
    assert "frame 4" in response.get_json()["error"]
    assert not (tmp_path / "sub" / "a.l3dump.labels.json").exists()


@pytest.mark.parametrize(
    "body",
    [
        _label_payload(version=9),
        _label_payload(ball={"points": [{"frame": 1, "range_bin": float("nan")}]}),
        _label_payload(bogus=1),
        _label_payload(ball={"points": 5}),
        _label_payload(club={"points": None}),
        [],
    ],
)
def test_put_labels_reports_invalid_labels_as_400(client, body):
    response = client.put("/api/labels", query_string={"path": "sub/a.l3dump"}, json=body)
    assert response.status_code == 400


@pytest.mark.parametrize("method", ["get", "put"])
@pytest.mark.parametrize(
    "path", ["notes.txt", "sub/missing.l3dump", "", "/etc/passwd", "../x.l3dump"]
)
def test_labels_endpoints_refuse_paths_outside_the_folder_or_not_captures(client, method, path):
    kwargs = {"json": _label_payload()} if method == "put" else {}
    response = getattr(client, method)("/api/labels", query_string={"path": path}, **kwargs)
    assert response.status_code == 400


def test_get_labels_reports_a_changed_dump_as_400(client, tmp_path):
    client.put("/api/labels", query_string={"path": "sub/a.l3dump"}, json=_label_payload())
    dump = tmp_path / "sub" / "a.l3dump"
    dump.write_bytes(dump.read_bytes() + b"\0")
    response = client.get("/api/labels", query_string={"path": "sub/a.l3dump"})
    assert response.status_code == 400
    assert "changed since it was labelled" in response.get_json()["error"]


ANNOTATE_IDS = [
    "annotate",
    "ann-ball",
    "ann-club",
    "ann-seed",
    "ann-clear",
    "ann-reviewed",
    "ann-range-tol",
    "ann-min-cov",
    "ann-notes",
    "ann-save",
    "ann-status",
]


def test_page_has_the_annotate_controls(client):
    page = client.get("/").data.decode()
    for name in ANNOTATE_IDS:
        assert f'id="{name}"' in page, name
    assert "/api/labels" in page


def test_annotate_marks_are_not_the_ball_blue_reused_for_something_else(client):
    """The ball keeps its colour: labels use the object colours, no new blue."""
    page = client.get("/").data.decode()
    assert "ball label" in page and "club label" in page


def test_upload_drops_annotate_and_map_keeps_zoom(client):
    page = client.get("/").data.decode()
    upload = page.split("function useUpload", 1)[1].split("loadFiles();", 1)[0]
    assert "annOff()" in upload and "ANN_NEEDS_CAPTURE" in upload
    assert 'uirevision: "map"' in page
    assert "!current" in page.split("async function annSave", 1)[1][:200]


class _AncestorIds(HTMLParser):
    """The ids of every open element around each element that has an id."""

    def __init__(self):
        super().__init__()
        self.stack: list[str | None] = []
        self.ancestors: dict[str, list[str]] = {}

    def handle_starttag(self, tag, attrs):
        if tag in ("input", "br", "img", "meta", "link", "hr"):
            element_id = dict(attrs).get("id")
            if element_id:
                self.ancestors[element_id] = [i for i in self.stack if i]
            return
        element_id = dict(attrs).get("id")
        if element_id:
            self.ancestors[element_id] = [i for i in self.stack if i]
        self.stack.append(element_id)

    def handle_endtag(self, tag):
        if self.stack:
            self.stack.pop()


def test_the_annotate_status_is_visible_when_the_annotate_body_is_hidden(client):
    """Load errors and the upload hint are written before #annBody is ever shown."""
    parser = _AncestorIds()
    parser.feed(client.get("/").data.decode())
    assert "annBody" in parser.ancestors and "ann-status" in parser.ancestors
    assert "annBody" not in parser.ancestors["ann-status"]
    assert "annPanel" in parser.ancestors["ann-status"]


def test_seeding_without_a_firmware_track_says_so_and_drops_the_aliased_doppler(client):
    page = client.get("/").data.decode()
    seed = page.split('$("#ann-seed").addEventListener', 1)[1].split("});", 1)[0]
    assert "no firmware track to seed from" in seed
    assert "doppler_mps" not in seed


def test_the_reviewed_checkbox_says_it_covers_both_objects(client):
    page = client.get("/").data.decode()
    label = re.search(r'<input type="checkbox" id="ann-reviewed">([^<]*)<', page).group(1)
    assert "BOTH ball and club" in label


# --- a manifest beside the dump configures the page like it configures the tests ---


def _write_manifest(folder, entry=None, default=None):
    manifest = {
        "default": default
        or {"tee_bin": 34, "dest_bin": 49, "post_from_frame": 9, "notes": "range session"},
        "a.l3dump": entry
        or {"pitch_deg": 10.4, "band_bins": 3.0, "expect": {"ball_speed_mps": [28, 45]}},
    }
    (folder / "manifest.json").write_text(json.dumps(manifest), encoding="utf-8")


def test_session_context_offers_the_manifest_options_when_no_session_mentions_the_dump(tmp_path):
    dump = tmp_path / "a.l3dump"
    dump.write_bytes(b"")
    _write_manifest(tmp_path)

    context = dv.session_context(dump)

    assert context["session_file"] is None
    assert context["defaults"] == {
        "tee_bin": 34,
        "dest_bin": 49,
        "post_from_frame": 9,
        "pitch_deg": 10.4,
        "band_bins": 3.0,
    }
    dv.ViewerOptions.from_mapping(context["defaults"])  # the page can post them back


def test_the_manifest_wins_over_the_session_log_for_the_keys_it_sets(tmp_path):
    dump = tmp_path / "a.l3dump"
    dump.write_bytes(b"")
    _write_session(tmp_path / "session_a.jsonl", dump.name)
    _write_manifest(tmp_path, entry={"tee_bin": 39, "dest_bin": None})

    context = dv.session_context(dump)

    assert context["session_file"] == "session_a.jsonl"
    defaults = context["defaults"]
    assert defaults["tee_bin"] == 39  # the session log said 41
    assert defaults["post_from_frame"] == 9
    assert defaults["snr"] == 6.0  # the manifest is silent on it: the session still applies
    assert defaults["pitch_deg"] == 10.4
    assert "dest_bin" not in defaults  # a null entry is "not set", never the string "None"


def test_a_manifest_that_gives_the_dump_no_tee_bin_is_no_manifest_options(tmp_path):
    dump = tmp_path / "a.l3dump"
    dump.write_bytes(b"")
    (tmp_path / "manifest.json").write_text(json.dumps({"a.l3dump": {"notes": "x"}}))
    assert dv.session_context(dump) is None


def test_a_dump_without_a_manifest_entry_gets_the_manifest_default(tmp_path):
    dump = tmp_path / "other.l3dump"
    dump.write_bytes(b"")
    _write_manifest(tmp_path)
    assert dv.session_context(dump)["defaults"]["tee_bin"] == 34


def test_context_endpoint_returns_the_manifest_options(client, tmp_path):
    _write_manifest(tmp_path / "sub")
    context = client.get("/api/context?path=sub/a.l3dump").get_json()
    assert context["defaults"]["dest_bin"] == 49
    assert context["defaults"]["post_from_frame"] == 9


def test_the_analysed_firmware_config_uses_the_manifest_options(client, tmp_path, monkeypatch):
    _write_manifest(tmp_path / "sub")
    seen = []

    def record(_raw, config):
        seen.append(config)
        raise ValueError("stop after the config")

    monkeypatch.setattr(dv.fr, "replay_dump", record)
    context = client.get("/api/context?path=sub/a.l3dump").get_json()
    response = client.post(
        "/api/analyze", json={"path": "sub/a.l3dump", "options": context["defaults"]}
    )
    assert response.status_code == 200, response.get_json()
    (config,) = seen
    assert (config.tee_bin, config.dest_bin, config.post_from_frame) == (34, 49, 9)
    assert config.pitch_deg == 10.4


def test_the_page_applies_checkbox_and_select_defaults(client):
    page = client.get("/").data.decode()
    apply = page.split("function applyDefaults", 1)[1].split("function readForm", 1)[0]
    assert "checkbox" in apply


def test_the_page_says_when_no_session_log_was_found_even_with_manifest_options(client):
    page = client.get("/").data.decode()
    ctx = page.split("function renderCtx", 1)[1].split("function chip", 1)[0]
    assert "!c || !c.session_file" in ctx


def test_hide_removes_the_open_dump_from_the_list_and_remembers_it(client):
    """Hiding is a list filter for this served folder, remembered in the browser."""
    page = client.get("/").data.decode()
    assert 'id="hide"' in page and 'id="show-hidden"' in page
    assert "openflight.dumpViewer.hidden" in page
    render = page.split("function renderFiles()", 1)[1].split("async function selectFile", 1)[0]
    assert "hidden.has(f.path)" in render
    hide = page.split("async function hideCurrent()", 1)[1].split("async function run", 1)[0]
    assert "setHidden" in hide and "selectFile" in hide and "filesRoot" in page
    opening = page.split("function openFromHash()", 1)[1].split("function renderFiles", 1)[0]
    assert "hidden.has" in opening


def test_switching_dumps_keeps_the_firmware_form(client):
    """Session defaults fill an empty form once; later dumps keep what is on screen."""
    page = client.get("/").data.decode()
    select = page.split("async function selectFile", 1)[1].split("function resetForm", 1)[0]
    assert "if (!keepSettings) resetForm();" in select
    run = page.split("async function run(fresh)", 1)[1].split("function ", 1)[0]
    assert "if (ctx && !keepSettings)" in run
    assert "saveForm()" in run
    assert "openflight.dumpViewer.settings:" in page
    load = page.split("async function loadFiles()", 1)[1].split("function openFromHash", 1)[0]
    assert "restoreForm()" in load
    reset = page.split('$("#reset").onclick', 1)[1].split('$("#useFreeze")', 1)[0]
    assert "saveForm()" in reset
    assert 'OPTS.forEach((k) => $("#" + k).addEventListener("change", () => { saveForm();' in page


def _page() -> str:
    return (Path(__file__).parents[1] / "scripts" / "iwr6843" / "dump_viewer.html").read_text(
        encoding="utf-8"
    )


def test_the_trajectory_view_has_a_raw_fitted_both_toggle():
    html = _page()
    assert 'id="tabsTraj"' in html
    for value in ("both", "raw", "fitted"):
        assert f'data-t="{value}"' in html
    assert 'let trajShow = "both"' in html


def test_the_trajectory_view_draws_the_reconstruction_and_tolerates_its_absence():
    """Review Focus 5: an older payload (no filtered_position) still draws the raw track."""
    html = _page()
    body = html[html.index("function renderTraj(") : html.index("// ---------- annotate")]
    assert "filtered_position" in body
    assert ".filter((p) => p.filtered_position)" in body, "points with no reconstruction are skipped"
    assert "filter_hypothesis" in body and "angle_confidence" in body


@needs_compiler
def test_shot_points_carry_their_reconstruction_to_the_page():
    raw = synth_shot_dump(
        path_deg=3.0, hla_deg=2.0, vla_deg=12.0, ball_speed_ms=60.0, tee_range_m=TEE_RANGE_M
    )
    data = dv.analyze_dump(raw, dv.ViewerOptions(tee_bin=TEE_BIN, tee_range_m=TEE_RANGE_M))
    json.dumps(data, allow_nan=False)
    firmware = data["firmware"]
    for point in firmware["points"] + firmware["ball_points"]:
        assert {"filtered_position", "filter_accepted", "filter_hypothesis", "angle_confidence"} <= set(point)
        assert point["filter_hypothesis"] in fw.FILTER_HYP_NAMES
    assert "angle_why" in firmware["launch"]


def test_the_hover_only_reports_a_fit_for_reconstructed_points():
    html = _page()
    body = html[html.index("function renderTraj(") : html.index("// ---------- annotate")]
    assert "p.filter_accepted === false" in body
    assert 'p.filter_hypothesis ?? "unfiltered"' not in body


def test_a_track_without_a_reconstruction_is_still_drawn_raw():
    page = _page()
    assert 'const alone = trajShow === "raw" || !list.some((p) => p.filtered_position);' in page
    assert 'if (trajShow !== "fitted" || !(list || []).some((p) => p.filtered_position)) raw(' in page


def test_the_launch_chip_says_why_the_angles_are_missing():
    page = _page()
    assert "F.launch.angle_why" in page and "F.launch.angles_accepted" in page
    assert "angles: " in page
