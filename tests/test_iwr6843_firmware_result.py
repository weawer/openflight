"""Tests for the IWR6843 shot result, firmware/iwr6843/l3_result.c, and its
host parser, openflight.iwr6843.shot_result.

The result is built from explicit shot, ball-track and launch structures so
each validation rule and each flag is exercised on its own; the packet the C
serialises is parsed by the host and every field compared.
"""

from __future__ import annotations

import ctypes
import json
import math
import struct

import pytest

from openflight.iwr6843 import firmware_host as fw, shot_result

DEG = math.pi / 180.0
M = {name: index for index, name in enumerate(fw.RESULT_METRIC_NAMES)}


@pytest.fixture(scope="module")
def lib(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


def make_shot(lib, *, state="result", club_points=7, source=fw.SHOT_IMPACT_RANGE, **delivery):
    shot = fw.Shot()
    cfg = fw.ShotCfg()
    lib.l3_shot_cfg_defaults(ctypes.byref(cfg))
    lib.l3_shot_init(ctypes.byref(shot), ctypes.byref(cfg))
    shot.state = fw.SHOT_STATE_NAMES.index(state)
    shot.impactTimestampUs = 23218
    shot.impactSource = source
    shot.clubPoints = club_points
    shot.ballOrigin = fw.Vec3(1.36, 0.0, 0.0)
    d = shot.delivery
    d.points = delivery.get("points", club_points)
    d.speedMps = delivery.get("speed", 40.0)
    # The approach's range rate; 0 (unset) is "no approach to compare with".
    d.radialSpeedMps = delivery.get("radial", 0.0)
    d.pathRad = delivery.get("path_deg", 2.0) * DEG
    d.attackRad = delivery.get("attack_deg", -3.0) * DEG
    d.residualM = delivery.get("residual", 0.01)
    d.confidence = delivery.get("confidence", 0.8)
    d.speedValid = 1 if delivery.get("valid", True) else 0
    d.pathValid = 1 if delivery.get("angles", True) else 0
    d.attackValid = 1 if delivery.get("angles", True) else 0
    return shot


def make_ball(lib, *, confirmed=True, points=6, coasted=0):
    ball = fw.BallTrack()
    cfg = fw.BallTrackCfg()
    lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
    lib.l3_ball_track_init(ctypes.byref(ball), ctypes.byref(cfg))
    ball.confirmed = 1 if confirmed else 0
    ball.core.count = points
    ball.counters[fw.BALL_TRACK_WHY_NAMES.index("coasted")] = coasted
    return ball


def make_launch(
    *,
    speed=60.0,
    hla_deg=1.0,
    vla_deg=12.0,
    points=6,
    residual=0.005,
    confidence=0.9,
    angles=True,
    valid=True,
):
    launch = fw.Launch()
    launch.points = points
    launch.speedMps = speed
    launch.radialSpeedMps = speed
    launch.hlaRad = hla_deg * DEG
    launch.vlaRad = vla_deg * DEG
    launch.residualM = residual
    launch.confidence = confidence
    launch.speedValid = 1 if valid else 0
    launch.hlaValid = 1 if (angles and valid) else 0
    launch.vlaValid = 1 if (angles and valid) else 0
    return launch


def build(lib, shot, ball, launch, *, shot_id=3, locked=True) -> fw.ShotResult:
    out = fw.ShotResult()
    lib.l3_result_build(
        ctypes.byref(shot),
        ctypes.byref(ball),
        ctypes.byref(launch),
        None,
        shot_id,
        1 if locked else 0,
        ctypes.byref(out),
    )
    return out


def quality(result) -> set[str]:
    return {name for name, bit in fw.QUALITY_FLAGS.items() if result.qualityFlags & bit}


# Every flag a clean shot earns; the warnings are never earned, and
# geometric_impact is reserved: the geometric detector was removed (2026-09-30).
WARNINGS = {"impact_uncertain", "ball_slower_than_club"}
RESERVED = {"geometric_impact"}
GOOD_QUALITY = set(fw.QUALITY_FLAGS) - WARNINGS - RESERVED


def test_a_complete_shot_is_valid_with_every_core_metric_measured(lib):
    result = build(lib, make_shot(lib), make_ball(lib), make_launch())
    assert result.version == 2 and result.shotId == 3
    assert fw.RESULT_VERDICT_NAMES[result.verdict] == "valid"
    for name in (
        "ball_speed",
        "vertical_launch",
        "horizontal_launch",
        "club_speed",
        "club_path",
        "angle_of_attack",
        "impact_range",
    ):
        m = result.metric[M[name]]
        assert m.flags & fw.MEAS_VALID and m.flags & fw.MEAS_MEASURED, name
        assert not (m.flags & fw.MEAS_IMPLAUSIBLE), name
        assert result.validFlags & (1 << M[name])
    for name in ("spin_rate", "spin_axis"):
        assert not (result.metric[M[name]].flags & fw.MEAS_VALID)
        assert not (result.validFlags & (1 << M[name]))
    assert result.metric[M["ball_speed"]].value == pytest.approx(60.0)
    assert result.metric[M["vertical_launch"]].value == pytest.approx(12.0 * DEG)
    assert result.metric[M["club_path"]].value == pytest.approx(2.0 * DEG)
    assert result.metric[M["impact_range"]].value == pytest.approx(1.36)
    assert result.smash == pytest.approx(1.5)
    assert result.impactTimestampUs == 23218 and result.impactSource == fw.SHOT_IMPACT_RANGE
    assert result.clubPoints == 7 and result.ballPoints == 6
    assert quality(result) == GOOD_QUALITY


def test_ball_flight_without_a_club_is_partial_and_club_without_flight_too(lib):
    no_club = build(lib, make_shot(lib, valid=False), make_ball(lib), make_launch())
    assert fw.RESULT_VERDICT_NAMES[no_club.verdict] == "partial"
    assert not (no_club.validFlags & (1 << M["club_speed"]))
    assert no_club.smash == 0.0
    no_ball = build(
        lib, make_shot(lib), make_ball(lib, confirmed=False, points=0), make_launch(valid=False)
    )
    assert fw.RESULT_VERDICT_NAMES[no_ball.verdict] == "partial"
    assert not (no_ball.validFlags & (1 << M["ball_speed"]))
    assert "ball_from_origin" not in quality(no_ball)


def test_nothing_measured_is_invalid(lib):
    result = build(
        lib,
        make_shot(lib, state="ready", valid=False),
        make_ball(lib, confirmed=False, points=0),
        make_launch(valid=False),
    )
    assert fw.RESULT_VERDICT_NAMES[result.verdict] == "invalid"
    assert result.validFlags == 0 and "impact_identified" not in quality(result)


def test_impossible_smash_doubts_both_speeds(lib):
    result = build(lib, make_shot(lib, speed=20.0), make_ball(lib), make_launch(speed=60.0))
    assert result.smash == pytest.approx(3.0)
    assert result.metric[M["ball_speed"]].flags & fw.MEAS_IMPLAUSIBLE
    assert result.metric[M["club_speed"]].flags & fw.MEAS_IMPLAUSIBLE
    assert "smash_plausible" not in quality(result) and "speeds_plausible" not in quality(result)
    assert fw.RESULT_VERDICT_NAMES[result.verdict] == "invalid"


@pytest.mark.parametrize(
    "kwargs,metric",
    [
        ({"speed": 130.0}, "ball_speed"),
        ({"vla_deg": 70.0}, "vertical_launch"),
        ({"hla_deg": -50.0}, "horizontal_launch"),
    ],
)
def test_out_of_bounds_launch_values_are_flagged_not_clipped(lib, kwargs, metric):
    result = build(lib, make_shot(lib), make_ball(lib), make_launch(**kwargs))
    m = result.metric[M[metric]]
    assert m.flags & fw.MEAS_IMPLAUSIBLE
    assert m.value == pytest.approx(
        list(kwargs.values())[0] * (DEG if "deg" in list(kwargs)[0] else 1.0)
    )
    assert fw.RESULT_VERDICT_NAMES[result.verdict] != "valid"


def test_out_of_bounds_club_values_are_flagged(lib):
    result = build(
        lib, make_shot(lib, path_deg=40.0, attack_deg=25.0), make_ball(lib), make_launch()
    )
    assert result.metric[M["club_path"]].flags & fw.MEAS_IMPLAUSIBLE
    assert result.metric[M["angle_of_attack"]].flags & fw.MEAS_IMPLAUSIBLE
    assert "angles_plausible" not in quality(result)


def test_range_only_speeds_are_marked_radial(lib):
    result = build(lib, make_shot(lib, angles=False), make_ball(lib), make_launch(angles=False))
    assert result.metric[M["ball_speed"]].flags & fw.MEAS_RADIAL_ONLY
    assert result.metric[M["club_speed"]].flags & fw.MEAS_RADIAL_ONLY
    assert not (result.validFlags & (1 << M["club_path"]))
    assert not (result.validFlags & (1 << M["vertical_launch"]))
    assert fw.RESULT_VERDICT_NAMES[result.verdict] == "partial", "no launch angles"


def test_the_tee_standing_in_for_the_ball_is_recorded(lib):
    result = build(lib, make_shot(lib), make_ball(lib), make_launch(), locked=False)
    assert result.metric[M["ball_speed"]].flags & fw.MEAS_FALLBACK
    assert result.metric[M["impact_range"]].flags & fw.MEAS_FALLBACK
    assert not (result.metric[M["impact_range"]].flags & fw.MEAS_MEASURED)
    assert result.metric[M["impact_range"]].confidence == pytest.approx(0.5)
    assert "ball_locked" not in quality(result)
    assert fw.RESULT_VERDICT_NAMES[result.verdict] == "valid", "a fallback is still a shot"


def test_residuals_coasts_and_short_tracks_show_in_the_quality_flags(lib):
    scattered = build(lib, make_shot(lib, residual=0.1), make_ball(lib), make_launch())
    assert "residuals_ok" not in quality(scattered)
    assert fw.RESULT_VERDICT_NAMES[scattered.verdict] == "partial"
    coasted = build(lib, make_shot(lib), make_ball(lib, coasted=1), make_launch())
    assert "ball_continuous" not in quality(coasted) and "ball_from_origin" in quality(coasted)
    short = build(lib, make_shot(lib, club_points=2, points=2), make_ball(lib), make_launch())
    assert not (short.validFlags & (1 << M["club_speed"]))
    assert "club_track" not in quality(short)
    # Older sources (the removed gate and geometry) earn no extra flag.
    for source in (fw.SHOT_IMPACT_GATE, fw.SHOT_IMPACT_GEOMETRY):
        old = build(lib, make_shot(lib, source=source), make_ball(lib), make_launch())
        assert "geometric_impact" not in quality(old) and "impact_identified" in quality(old)


def test_names_and_text_formats(lib):
    for index, name in enumerate(fw.RESULT_METRIC_NAMES):
        assert lib.l3_result_metric_name(index).decode() == name
    assert lib.l3_result_metric_name(9).decode() == "?"
    for index, name in enumerate(fw.RESULT_VERDICT_NAMES):
        assert lib.l3_result_verdict_name(index).decode() == name
    result = build(lib, make_shot(lib), make_ball(lib), make_launch())
    text = fw.c_text(lib.l3_result_format, ctypes.byref(result), cap=240)
    assert text.startswith("result v2 shot=3 verdict=valid valid=0x")
    assert " impact=23218 source=range club=7 ball=6 smash=1.50" in text
    speed = fw.c_text(lib.l3_result_format_metric, ctypes.byref(result), M["ball_speed"])
    assert speed == "  ball_speed=60.00 conf=0.90 flags=measured"
    vla = fw.c_text(lib.l3_result_format_metric, ctypes.byref(result), M["vertical_launch"])
    assert vla == "  vertical_launch=12.00deg conf=0.90 flags=measured"
    spin = fw.c_text(lib.l3_result_format_metric, ctypes.byref(result), M["spin_rate"])
    assert spin == "  spin_rate=- conf=0.00 flags=none"
    fallback = build(lib, make_shot(lib, angles=False), make_ball(lib), make_launch(), locked=False)
    club = fw.c_text(lib.l3_result_format_metric, ctypes.byref(fallback), M["club_speed"])
    assert club.endswith("flags=measured,radial")
    rng = fw.c_text(lib.l3_result_format_metric, ctypes.byref(fallback), M["impact_range"])
    assert rng.endswith("flags=inferred,tee")


def test_packet_is_164_little_endian_bytes_the_host_parses_back(lib):
    result = build(lib, make_shot(lib), make_ball(lib), make_launch(hla_deg=-1.5), shot_id=7)
    buffer = ctypes.create_string_buffer(fw.RESULT_PACKET_BYTES)
    assert lib.l3_result_serialize(ctypes.byref(result), buffer, fw.RESULT_PACKET_BYTES) == 164
    assert lib.l3_result_serialize(ctypes.byref(result), buffer, 163) == 0
    packet = shot_result.parse_packet(buffer.raw)
    assert packet.version == 2 and packet.shot_id == 7 and packet.verdict == "valid"
    assert packet.impact_timestamp_us == 23218 and packet.impact_source == "range"
    assert packet.club_points == 7 and packet.ball_points == 6
    assert packet.smash == pytest.approx(1.5)
    assert packet["ball_speed"].value == pytest.approx(60.0)
    assert packet["horizontal_launch"].value == pytest.approx(-1.5, abs=1e-4), "degrees on the host"
    assert packet["club_path"].value == pytest.approx(2.0, abs=1e-4)
    assert packet["impact_range"].value == pytest.approx(1.36)
    assert packet["spin_rate"].value is None and packet["spin_rate"].label == "-"
    assert packet["ball_speed"].label == "MEASURED" and packet["ball_speed"].usable
    assert packet.quality == GOOD_QUALITY
    assert packet["ball_speed"].confidence == pytest.approx(0.9)
    assert packet.impact_fit["verdict"] == "none"
    # The hex line the CLI prints round-trips through the same parser.
    hex_text = fw.c_text(lib.l3_result_format_hex, ctypes.byref(result), cap=340)
    assert len(hex_text) == 328
    assert shot_result.parse_hex(f"packet {hex_text}") == packet
    reply = (
        f"result v2 shot=7 ...\n  ball_speed=60.00 ...\npacket {hex_text[:164]}\n"
        f"packet+ {hex_text[164:]}\nDone\n"
    )
    assert shot_result.parse_result_reply(reply) == packet
    assert shot_result.parse_result_reply("Done\n") is None


def test_host_parser_reads_implausible_radial_and_fallback_from_the_packet(lib):
    result = build(
        lib,
        make_shot(lib, speed=20.0, angles=False),
        make_ball(lib),
        make_launch(speed=60.0),
        locked=False,
    )
    buffer = ctypes.create_string_buffer(fw.RESULT_PACKET_BYTES)
    lib.l3_result_serialize(ctypes.byref(result), buffer, fw.RESULT_PACKET_BYTES)
    packet = shot_result.parse_packet(buffer.raw)
    assert packet.verdict == "invalid"
    assert packet["ball_speed"].implausible and packet["club_speed"].implausible
    assert not packet["ball_speed"].usable
    assert packet["club_speed"].radial_only and not packet["ball_speed"].radial_only
    assert packet["ball_speed"].fallback and "ball_locked" not in packet.quality


def test_packet_to_dict_keeps_provenance_beside_every_metric(lib):
    result = build(lib, make_shot(lib), make_ball(lib), make_launch(hla_deg=-1.5), shot_id=7)
    buffer = ctypes.create_string_buffer(fw.RESULT_PACKET_BYTES)
    lib.l3_result_serialize(ctypes.byref(result), buffer, fw.RESULT_PACKET_BYTES)
    packet = shot_result.parse_packet(buffer.raw)

    payload = packet.to_dict()

    assert payload["version"] == 2 and payload["shot_id"] == 7 and payload["verdict"] == "valid"
    assert payload["impact_fit"]["verdict"] == "none"
    assert payload["impact_source"] == "range" and payload["impact_timestamp_us"] == 23218
    assert payload["club_points"] == 7 and payload["ball_points"] == 6
    assert payload["smash"] == pytest.approx(1.5)
    assert payload["quality"] == sorted(GOOD_QUALITY)
    assert set(payload["metrics"]) == set(fw.RESULT_METRIC_NAMES)
    ball = payload["metrics"]["ball_speed"]
    assert ball["value"] == pytest.approx(60.0) and ball["label"] == "MEASURED"
    assert ball["measured"] and ball["usable"] and not ball["radial_only"]
    spin = payload["metrics"]["spin_rate"]
    assert spin["value"] is None and spin["label"] == "-" and not spin["usable"]
    assert json.dumps(payload), "the record is JSON-serialisable as it stands"
    # Per-domain confidence: the weakest usable metric of each domain; spin has none.
    domains = payload["domains"]
    assert set(domains) == {"club", "ball", "angle", "spin"}
    assert domains["ball"] == pytest.approx(0.9)
    assert domains["spin"] == 0.0
    club_metrics = [payload["metrics"][n] for n in ("club_speed", "club_path", "angle_of_attack")]
    assert domains["club"] == pytest.approx(
        min(m["confidence"] for m in club_metrics if m["usable"]), abs=1e-3
    )
    assert domains["angle"] == pytest.approx(
        min(payload["metrics"][n]["confidence"] for n in ("vertical_launch", "horizontal_launch")),
        abs=1e-3,
    )


def test_host_parser_rejects_wrong_sizes_versions_and_bad_hex():
    with pytest.raises(ValueError, match="bytes"):
        shot_result.parse_packet(b"\x00" * 2)
    with pytest.raises(ValueError, match="version"):
        shot_result.parse_packet(struct.pack("<I", 9) + bytes(160))
    bad_size = bytearray(100)
    bad_size[0] = 2
    with pytest.raises(ValueError, match="bytes"):
        shot_result.parse_packet(bytes(bad_size))
    with pytest.raises(ValueError, match="hex"):
        shot_result.parse_hex("packet zz")


def test_packet_v2_carries_the_impact_fit(lib):
    fit = fw.ImpactFit()
    lib.l3_impact_fit_reset(ctypes.byref(fit))
    fit.verdict = fw.FIT_VERDICT_NAMES.index("consistent")
    fit.impactUs, fit.spreadUs, fit.refinedMinusTriggerUs = 30_000.0, 210.0, -2_500.0
    fit.track[2].why, fit.track[2].points = 0, 4
    fit.track[2].timeUs, fit.track[2].sigmaUs, fit.track[2].speedMps = 29_990.0, 60.0, 61.2
    shot = fw.Shot()
    ball = fw.BallTrack()
    launch = fw.Launch()
    result = fw.ShotResult()
    lib.l3_result_build(
        ctypes.byref(shot),
        ctypes.byref(ball),
        ctypes.byref(launch),
        ctypes.byref(fit),
        7,
        1,
        ctypes.byref(result),
    )
    buffer = ctypes.create_string_buffer(fw.RESULT_PACKET_BYTES)
    assert lib.l3_result_serialize(ctypes.byref(result), buffer, fw.RESULT_PACKET_BYTES) == 164
    raw = buffer.raw

    parsed = shot_result.parse_packet(raw)

    assert parsed.version == 2
    assert parsed.impact_fit["verdict"] == "consistent"
    assert parsed.impact_fit["impact_us"] == pytest.approx(30_000.0)
    assert parsed.impact_fit["refined_minus_trigger_us"] == pytest.approx(-2_500.0)
    assert parsed.impact_fit["dropped"] is None
    ball_out = parsed.impact_fit["tracks"]["ball_out"]
    assert (ball_out["why"], ball_out["points"]) == ("ok", 4)
    assert ball_out["speed_mps"] == pytest.approx(61.2)
    assert parsed.impact_fit["tracks"]["club_in"]["why"] == "missing"
    assert parsed.to_dict()["impact_fit"]["verdict"] == "consistent"


def test_null_fit_serialises_as_verdict_none(lib):
    result = fw.ShotResult()
    lib.l3_result_build(
        ctypes.byref(fw.Shot()),
        ctypes.byref(fw.BallTrack()),
        ctypes.byref(fw.Launch()),
        None,
        1,
        0,
        ctypes.byref(result),
    )
    buffer = ctypes.create_string_buffer(fw.RESULT_PACKET_BYTES)
    lib.l3_result_serialize(ctypes.byref(result), buffer, fw.RESULT_PACKET_BYTES)
    assert shot_result.parse_packet(buffer.raw).impact_fit["verdict"] == "none"


def test_v1_packet_still_parses_without_an_impact_fit():
    v1 = struct.pack(
        "<II9fII9fIBBBBf", 1, 3, *([0.0] * 9), 0, 0, *([0.0] * 9), 12345, 2, 1, 5, 4, 1.4
    )
    parsed = shot_result.parse_packet(v1)
    assert parsed.version == 1
    assert parsed.impact_fit is None
    assert parsed.impact_timestamp_us == 12345
    assert parsed.to_dict()["impact_fit"] is None


def test_wrong_size_or_version_is_refused():
    with pytest.raises(ValueError, match="bytes"):
        shot_result.parse_packet(b"\x02\x00\x00\x00" + bytes(96))
    with pytest.raises(ValueError, match="version"):
        shot_result.parse_packet(struct.pack("<I", 9) + bytes(160))


def fit_with(lib, verdict: str) -> fw.ImpactFit:
    fit = fw.ImpactFit()
    lib.l3_impact_fit_reset(ctypes.byref(fit))
    fit.verdict = fw.FIT_VERDICT_NAMES.index(verdict)
    return fit


def build_with_fit(lib, fit) -> fw.ShotResult:
    out = fw.ShotResult()
    lib.l3_result_build(
        ctypes.byref(make_shot(lib)),
        ctypes.byref(make_ball(lib)),
        ctypes.byref(make_launch()),
        ctypes.byref(fit),
        3,
        1,
        ctypes.byref(out),
    )
    return out


def test_impact_uncertain_is_the_next_bit_after_geometric_impact():
    assert fw.QUALITY_FLAGS["impact_uncertain"] == fw.QUALITY_FLAGS["geometric_impact"] << 1


def test_an_inconsistent_impact_fit_marks_the_shot_impact_uncertain(lib):
    result = build_with_fit(lib, fit_with(lib, "inconsistent"))
    assert "impact_uncertain" in quality(result)
    assert quality(result) == GOOD_QUALITY | {"impact_uncertain"}  # nothing else is lost


@pytest.mark.parametrize("verdict", ["none", "single_track", "consistent"])
def test_other_impact_fit_verdicts_leave_impact_uncertain_clear(lib, verdict):
    assert "impact_uncertain" not in quality(build_with_fit(lib, fit_with(lib, verdict)))


def test_no_fit_leaves_impact_uncertain_clear(lib):
    result = build(lib, make_shot(lib), make_ball(lib), make_launch())
    assert "impact_uncertain" not in quality(result)


def test_the_host_parser_surfaces_impact_uncertain(lib):
    result = build_with_fit(lib, fit_with(lib, "inconsistent"))
    buffer = ctypes.create_string_buffer(fw.RESULT_PACKET_BYTES)
    lib.l3_result_serialize(ctypes.byref(result), buffer, fw.RESULT_PACKET_BYTES)
    packet = shot_result.parse_packet(buffer.raw)
    assert "impact_uncertain" in packet.quality
    assert "impact_uncertain" in packet.to_dict()["quality"]


@pytest.mark.parametrize("mask", range(8))
def test_impact_source_names_match_the_firmware(lib, mask):
    buffer = ctypes.create_string_buffer(24)
    assert shot_result.IMPACT_SOURCES[mask] == lib.l3_shot_source_name(mask, buffer, 24).decode()


def test_impact_sources_cover_every_bit_combination():
    bits = fw.SHOT_IMPACT_GATE | fw.SHOT_IMPACT_GEOMETRY | fw.SHOT_IMPACT_RANGE
    assert set(shot_result.IMPACT_SOURCES) == set(range(bits + 1))
    assert "SHOT_IMPACT_RANGE" in fw.__all__


@pytest.mark.parametrize(
    ("why", "timed"),
    [(name, name in ("ok", "dropped", "uncertain")) for name in fw.FIT_WHY_NAMES] + [("?", False)],
)
def test_fit_track_timed_is_the_whys_the_c_keeps_a_time_for(why, timed):
    assert fw.fit_track_timed(why) is timed


@pytest.mark.parametrize(
    ("verdict", "decided"),
    [("none", False), ("single_track", True), ("consistent", True), ("inconsistent", True)]
    + [("?", False)],
)
def test_fit_verdict_decided_is_any_known_verdict_but_none(verdict, decided):
    assert fw.fit_verdict_decided(verdict) is decided


def test_fit_track_indices_are_named_in_track_order():
    assert (fw.FIT_CLUB_IN, fw.FIT_CLUB_OUT, fw.FIT_BALL_OUT) == tuple(
        fw.FIT_TRACK_NAMES.index(n) for n in ("club_in", "club_out", "ball_out")
    )


# --- the ball slower than the club's approach ---------------------------------
#
# 51 ball-visible captures (2026-09-29): every good ball track left at 1.02-1.86x
# the club's approach range rate (launch speed as the board fits it, range-only
# on every one of them); 14 of the 24 wrong ones with an approach were under 1.0x. The flag doubts the ball's speed
# and launch angles, never the club's.


def test_ball_slower_than_club_is_the_next_bit_after_impact_uncertain():
    assert fw.QUALITY_FLAGS["ball_slower_than_club"] == fw.QUALITY_FLAGS["impact_uncertain"] << 1


@pytest.mark.parametrize(
    ("launch_mps", "flagged"), [(29.0, True), (29.9, True), (30.0, False), (45.0, False)]
)
def test_a_ball_slower_than_the_clubs_approach_is_flagged(lib, launch_mps, flagged):
    result = build(lib, make_shot(lib, radial=30.0), make_ball(lib), make_launch(speed=launch_mps))
    assert ("ball_slower_than_club" in quality(result)) is flagged


def test_a_flagged_ball_is_never_a_valid_shot_but_the_club_stays_trusted(lib):
    """Smash 34/36 sits inside the 0.8-1.6 window, so only the new flag acts."""
    result = build(
        lib, make_shot(lib, speed=36.0, radial=35.0), make_ball(lib), make_launch(speed=34.0)
    )
    assert fw.RESULT_VERDICT_NAMES[result.verdict] == "partial"
    assert {"angles_plausible", "speeds_plausible", "club_track"} <= quality(result)


def test_a_range_only_ball_is_compared_too(lib):
    """The 51 captures' launches were all range-only (the angle fit rejected
    its angles), and the evidence is on that speed: a 3D speed only reads higher."""
    launch = make_launch(speed=20.0, angles=False)
    result = build(lib, make_shot(lib, speed=36.0, radial=35.0), make_ball(lib), launch)
    assert "ball_slower_than_club" in quality(result)


@pytest.mark.parametrize("delivery", [{"radial": 0.0}, {"radial": 30.0, "valid": False}])
def test_without_an_approach_nothing_is_compared(lib, delivery):
    result = build(lib, make_shot(lib, **delivery), make_ball(lib), make_launch(speed=5.0 + 15.0))
    assert "ball_slower_than_club" not in quality(result)


def _parsed(lib, result):
    buffer = ctypes.create_string_buffer(fw.RESULT_PACKET_BYTES)
    lib.l3_result_serialize(ctypes.byref(result), buffer, fw.RESULT_PACKET_BYTES)
    return shot_result.parse_packet(buffer.raw)


def test_the_host_does_not_use_a_flagged_balls_speed_or_launch_angles(lib):
    packet = _parsed(
        lib,
        build(
            lib, make_shot(lib, speed=36.0, radial=35.0), make_ball(lib), make_launch(speed=34.0)
        ),
    )
    assert "ball_slower_than_club" in packet.quality
    for name in ("ball_speed", "vertical_launch", "horizontal_launch"):
        assert packet[name].value is not None and not packet[name].usable, name
    for name in ("club_speed", "club_path", "angle_of_attack"):
        assert packet[name].usable, name


def test_the_host_uses_an_unflagged_balls_launch_angles(lib):
    packet = _parsed(
        lib, build(lib, make_shot(lib, radial=30.0), make_ball(lib), make_launch(speed=45.0))
    )
    assert "ball_slower_than_club" not in packet.quality
    assert packet["vertical_launch"].usable and packet["horizontal_launch"].usable


def test_doubting_onboard_angles_leaves_the_speeds_usable(lib):
    packet = _parsed(lib, build(lib, make_shot(lib), make_ball(lib), make_launch()))
    doubted = packet.with_onboard_angles_doubted()
    angles = ("vertical_launch", "horizontal_launch", "club_path", "angle_of_attack")
    for name in angles:
        assert not doubted[name].usable, name
    for name in packet.metrics:
        if name not in angles:
            assert doubted[name].usable == packet[name].usable, name
    assert doubted["ball_speed"].usable and doubted["club_speed"].usable


# --- ball_flight: what the no-ball veto reads (2026-10-01) ---------------------
#
# Raking a ball onto the tee or a waggle fired the self-trigger 8 times in 10 on
# 2026-10-01 with no ball leaving. A packet saying the board measured no ball
# speed is what lets the host skip the 7 s readback.


def _packet(lib, shot, ball, launch) -> shot_result.ShotResultPacket:
    result = build(lib, shot, ball, launch)
    buffer = ctypes.create_string_buffer(fw.RESULT_PACKET_BYTES)
    lib.l3_result_serialize(ctypes.byref(result), buffer, fw.RESULT_PACKET_BYTES)
    return shot_result.parse_packet(buffer.raw)


def test_a_measured_ball_speed_is_a_ball_flight(lib):
    assert _packet(lib, make_shot(lib), make_ball(lib), make_launch()).ball_flight


def test_a_club_without_a_ball_is_no_ball_flight(lib):
    packet = _packet(
        lib, make_shot(lib), make_ball(lib, confirmed=False, points=0), make_launch(valid=False)
    )
    assert packet.verdict == "partial"
    assert not packet.ball_flight


def test_an_implausible_ball_speed_still_counts_as_a_flight(lib):
    """The veto drops captures; a doubted ball is still a ball, so it keeps the dump."""
    packet = _packet(lib, make_shot(lib, speed=20.0), make_ball(lib), make_launch(speed=60.0))
    assert packet["ball_speed"].implausible
    assert packet.ball_flight


def test_ball_points_alone_are_not_a_flight(lib):
    """Replays of the raked-ball captures hold ball points on a stationary return."""
    packet = _packet(
        lib, make_shot(lib), make_ball(lib, confirmed=False, points=4), make_launch(valid=False)
    )
    assert packet.ball_points == 4
    assert not packet.ball_flight
