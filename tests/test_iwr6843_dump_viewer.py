"""Tests for the dump viewer's data layer and its Flask front end.

``openflight.iwr6843.dump_viewer`` turns one ``.l3dump`` into the JSON the
page plots; ``scripts/iwr6843/dump_viewer.py`` serves it. The maps must be
the firmware's own observations on a global-bin axis, the replays must
survive each other's failure, the session log must fill the options in, and
the server must only read captures under its folder.
"""

from __future__ import annotations

import importlib.util
import io
import json
import math
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


def test_options_coerce_form_strings_and_skip_blanks():
    options = dv.ViewerOptions.from_mapping(
        {
            "tee_bin": "41",
            "dest_bin": "",
            "snr": "6.5",
            "stat": "energy",
            "impact_armed": "true",
            "stop_at_fire": False,
            "post_from_frame": None,
            "tee_range_m": " ",
        }
    )
    assert options.tee_bin == 41
    assert options.dest_bin is None
    assert options.snr == 6.5
    assert options.stat == "energy"
    assert options.impact_armed is True
    assert options.stop_at_fire is False
    assert options.post_from_frame is None
    assert options.tee_range_m == dv.ViewerOptions().tee_range_m


@pytest.mark.parametrize("text", ["off", "0", "no", ""])
def test_options_read_falsy_checkbox_strings_as_false(text):
    assert dv.ViewerOptions.from_mapping({"impact_armed": text}).impact_armed is False


def test_options_reject_unknown_keys_and_bad_numbers():
    with pytest.raises(ValueError, match="unknown options: bogus"):
        dv.ViewerOptions.from_mapping({"bogus": 1})
    with pytest.raises(ValueError):
        dv.ViewerOptions.from_mapping({"tee_bin": "forty"})


def test_tee_bin_is_explicit_or_the_rounded_slant_range():
    assert dv.tee_bin_for(dv.ViewerOptions(tee_bin=12)) == 12
    assert dv.tee_bin_for(dv.ViewerOptions(tee_range_m=1.845)) == 39  # 1.845 / 0.046875
    assert dv.bin_width_m() == pytest.approx(6.0 / 128)


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("triggerCfg 41 6.0 2", {"tee_bin": 41, "snr": 6.0, "track_frames": 2}),
        ("  triggerCfg 7 3 1 ", {"tee_bin": 7, "snr": 3.0, "track_frames": 1}),
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
    assert data["python_trigger"]["ok"] is True
    assert data["n_frames"] == 4
    assert data["timestamps_ms"] == [0.0, 2.0, 4.0, 6.0]
    assert data["tee_bin"] == 25
    assert data["freeze_frame"] is None  # no retention report before v8


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
    # The frame the gate fired on watched a peak above snr x floor.
    peak = firmware["watched_peak"][firmware["fired_frame"]]
    assert peak["stat"] >= firmware["config"]["snr"] * fired["floor"] * 0.5
    for point in firmware["points"] + firmware["ball_points"]:
        assert len(point["position"]) == 3
    assert "clubtrack" in firmware["report"]


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
        "track_frames": 2,
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
