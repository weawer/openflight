"""scripts/iwr6843/swing_trigger.py: swing the club-track self-trigger on the board.

The range gate and the host ball-leave replay this tool once judged swings
with are gone (2026-09-30). A swing now passes when the club track's
range-only impact is what fired, read back from ``triggerLog track``.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "iwr6843" / "swing_trigger.py"
spec = importlib.util.spec_from_file_location("swing_trigger", SCRIPT)
swing_trigger = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = swing_trigger
spec.loader.exec_module(swing_trigger)

TEE_BIN = 14

# ``triggerLog track`` as the board prints it right after a club-track fire.
FIRED_TRACK = (
    "triggerLog track\n"
    "clubtrack active=1 why=associated count=7 total=7 misses=0 bin=37.62 dest=38 dist=0.38 "
    "vel=31.20 speed=31.40 fit=1 residual=0.08 acq=1 assoc=6 coast=0 drop=0\n"
    "delivery points=7 speed=31.40 valid=1\n"
    " angle az=+0.0 el=+2.1 estimates=6\n"
    "impact fired=0 why=pending closestcm=4.00 armed=0 source=4\n"
    "range impact fired=1 why=fired closestcm=0.00 offsetms=0.90 t=24900\n"
    "impactfit verdict=pending\n"
    "p frame=7 t=21.0ms bin=37.62 dist=0.4\n"
    "Done\n"
)


def test_windows_port_name_is_rejected_on_the_pi():
    message = swing_trigger.port_name_error("COM5", "linux")
    assert message is not None and "/dev/ttyUSB0" in message
    assert swing_trigger.port_name_error("COM5", "win32") is None
    assert swing_trigger.port_name_error("/dev/ttyUSB0", "linux") is None
    assert swing_trigger.port_name_error(None, "linux") is None


def test_parse_trig_reads_stats_and_debug_lines():
    stats = swing_trigger.parse_trig("trig phase=watching tee=1800 latched=0 enabled=1")
    assert stats == {"phase": "watching", "tee": "1800", "latched": "0", "enabled": "1"}
    assert swing_trigger.parse_trig("frames=1 active=1") is None


def test_released_fired_phase_is_not_a_new_swing():
    released = swing_trigger.parse_trig("trig phase=fired tee=1000 latched=0 enabled=1")
    held = swing_trigger.parse_trig("trig phase=fired tee=1000 latched=1 enabled=1")
    legacy = swing_trigger.parse_trig("trig phase=fired tee=1000")

    assert not swing_trigger.is_latched(released)
    assert swing_trigger.is_latched(held)
    assert swing_trigger.is_latched(legacy)


def test_format_status_reads_tee_as_the_floor_against_the_threshold():
    fields = swing_trigger.parse_trig("trig phase=toward tee=1800 latched=0 enabled=1")
    text = swing_trigger.format_status(fields, 10800.0)
    assert text.startswith("toward")
    assert "floor=1800" in text and "threshold=10800" in text


def test_a_club_track_fire_passes_with_its_track_summarised():
    passed, text = swing_trigger.summarize_fire(FIRED_TRACK)
    assert passed
    assert "PASS" in text
    assert "7 club points" in text and "31.4 m/s" in text


def test_a_fire_the_range_impact_did_not_make_fails():
    """The capture froze but not on the club track (or it lost the track)."""
    reply = FIRED_TRACK.replace(
        "range impact fired=1 why=fired", "range impact fired=0 why=nodelivery"
    )
    passed, text = swing_trigger.summarize_fire(reply)
    assert not passed
    assert "FAIL" in text and "nodelivery" in text


def test_an_unreadable_track_fails():
    passed, text = swing_trigger.summarize_fire("Error: triggerLog\n")
    assert not passed and "FAIL" in text


class _ArmRadar:
    def __init__(self):
        self.commands: list[str] = []

    def send_config(self, path: str, lines=None) -> None:
        self.commands.append(path)

    def cmd(self, line: str, window: float = 1.5) -> str:
        del window
        self.commands.append(line)
        return "Done\n"

    def set_confirm(self, enabled: bool) -> bool:
        self.commands.append(f"confirm {int(enabled)}")
        return self.confirm_supported

    confirm_supported = True


def test_arming_samples_the_lane_then_arms_the_club_track(monkeypatch):
    def measure(_radar, tee_bin, *, snr):
        assert (tee_bin, snr) == (TEE_BIN, 1.0)
        return 200000.0, 200000.0

    monkeypatch.setattr(swing_trigger, "measure_trigger_level", measure)
    radar = _ArmRadar()

    threshold = swing_trigger._arm(radar, "cfg", TEE_BIN, 1.0)

    assert threshold == 200000.0
    assert radar.commands[0] == "cfg"
    assert "confirm 0" in radar.commands, "a board left in confirm mode fires as before"
    assert radar.commands[-1] == "triggerCfg 14 1.0 1"


def test_arming_with_confirm_turns_the_flight_confirmation_on(monkeypatch):
    monkeypatch.setattr(swing_trigger, "measure_trigger_level", lambda *_a, **_k: (1.0, 1.0))
    radar = _ArmRadar()

    swing_trigger._arm(radar, "cfg", TEE_BIN, 1.0, confirm=True)

    assert radar.commands.index("confirm 1") < radar.commands.index("triggerCfg 14 1.0 1")


def test_arming_with_confirm_on_firmware_without_it_stops(monkeypatch):
    monkeypatch.setattr(swing_trigger, "measure_trigger_level", lambda *_a, **_k: (1.0, 1.0))
    radar = _ArmRadar()
    radar.confirm_supported = False

    with pytest.raises(SystemExit, match="no flight confirmation"):
        swing_trigger._arm(radar, "cfg", TEE_BIN, 1.0, confirm=True)


def test_arming_without_confirm_on_old_firmware_is_fine(monkeypatch):
    monkeypatch.setattr(swing_trigger, "measure_trigger_level", lambda *_a, **_k: (1.0, 1.0))
    radar = _ArmRadar()
    radar.confirm_supported = False

    assert swing_trigger._arm(radar, "cfg", TEE_BIN, 1.0) == 1.0


@pytest.mark.parametrize(
    ("line", "expected"),
    [
        (
            "confirm on=1 verdict=confirmed why=flight speed=48.20 dt=6000 confirmed_n=1 "
            "rejected_n=0 unarmed=0",
            "confirmed: a ball flight at 48.2 m/s, 6 ms after the candidate",
        ),
        (
            "confirm on=1 verdict=rejected why=timeout speed=0.00 dt=26000 confirmed_n=0 "
            "rejected_n=1 unarmed=0",
            "rejected: no ball flight (why=timeout), so no S!",
        ),
        (
            "confirm on=1 verdict=idle why=none speed=0.00 dt=0 confirmed_n=0 rejected_n=0 "
            "unarmed=1",
            "fired at once: no ball tracker armed",
        ),
        (
            "confirm on=1 verdict=pending why=few speed=0.00 dt=0 confirmed_n=0 rejected_n=0 "
            "unarmed=0",
            "confirm verdict pending (why=few)",
        ),
    ],
)
def test_the_confirm_verdict_is_summarised(line, expected):
    assert expected in swing_trigger.summarize_confirm(FIRED_TRACK + line + "\n")


@pytest.mark.parametrize(
    "track",
    [
        FIRED_TRACK,
        FIRED_TRACK + "confirm on=0 verdict=idle why=none speed=0.00 dt=0 confirmed_n=0 "
        "rejected_n=0 unarmed=0\n",
    ],
)
def test_no_confirm_verdict_without_confirm_mode(track):
    assert swing_trigger.summarize_confirm(track) is None


def test_a_swing_prints_the_confirm_verdict(capsys):
    track = FIRED_TRACK + (
        "confirm on=1 verdict=rejected why=ended speed=0.00 dt=30000 confirmed_n=0 "
        "rejected_n=1 unarmed=0\n"
    )

    swing_trigger._validate_swing(_SwingRadar(track), 1)

    assert "rejected: no ball flight (why=ended)" in capsys.readouterr().out


class _SwingRadar:
    def __init__(self, track: str, stats: str = "trig phase=watching tee=10 latched=0\nDone\n"):
        self.track = track
        self.stats_reply = stats
        self.calls: list[str] = []

    def club_track(self) -> str:
        self.calls.append("track")
        return self.track

    def shot_status(self) -> str:
        self.calls.append("shot")
        return "shot state=impact since=1 impact=24900 source=range\nDone\n"

    def release_sparse_freeze(self) -> None:
        self.calls.append("release")

    def stats(self) -> str:
        self.calls.append("stats")
        return self.stats_reply


def test_a_swing_reads_the_track_before_releasing_the_ring(capsys):
    radar = _SwingRadar(FIRED_TRACK)

    assert swing_trigger._validate_swing(radar, 1) is True

    assert radar.calls[:3] == ["track", "shot", "release"]
    out = capsys.readouterr().out
    assert "clubtrack active=1" in out and "source=range" in out and "PASS" in out


def test_a_ring_still_latched_after_the_release_fails():
    radar = _SwingRadar(FIRED_TRACK, stats="trig phase=fired tee=10 latched=1\nDone\n")
    assert swing_trigger._validate_swing(radar, 1) is False


@pytest.mark.parametrize("gone", ["--hits", "--level"])
def test_the_range_gate_options_are_gone(monkeypatch, gone):
    monkeypatch.setattr(sys, "argv", ["swing_trigger.py", gone, "2"])
    with pytest.raises(SystemExit):
        swing_trigger.main()
