"""Tests for the versioned, transport-neutral HTTP API surface."""

import json

from openflight import server as server_module
from openflight.launch_monitor import ClubType
from openflight.shot_stream import HEARTBEAT_FRAME, ShotStreamBroker


class _Monitor:
    def __init__(self):
        self.clubs = []

    def set_club(self, club):
        self.clubs.append(club)


def _shot_data():
    return {
        "timestamp": "2026-07-29T19:42:10",
        "club": "driver",
        "ball_speed_mph": 151.4,
        "club_speed_mph": None,
        "smash_factor": None,
        "estimated_carry_yards": 264,
        "launch_angle_vertical": None,
        "launch_angle_horizontal": None,
        "spin_rpm": None,
        "club_path_deg": None,
        "spin_axis_deg": None,
    }


def test_capabilities_describe_the_general_api(monkeypatch):
    monkeypatch.setattr(server_module, "monitor", _Monitor())
    monkeypatch.setattr(server_module, "iwr6843_runtime", None)
    monkeypatch.setattr(server_module, "ble_publisher", None)

    response = server_module.app.test_client().get("/api/v1/capabilities")

    assert response.status_code == 200
    assert response.get_json() == {
        "api_version": 1,
        "event_schema_versions": [1],
        "capabilities": ["club.read", "club.write", "events.stream"],
    }


def test_state_bootstraps_authoritative_resources(monkeypatch):
    monkeypatch.setattr(server_module, "active_club", ClubType.IRON_7)

    response = server_module.app.test_client().get("/api/v1/state")

    assert response.status_code == 200
    assert response.get_json() == {
        "api_version": 1,
        "club": {"value": "7-iron"},
    }


def test_versioned_club_resource_uses_existing_domain_operation(monkeypatch):
    monitor = _Monitor()
    monkeypatch.setattr(server_module, "monitor", monitor)

    response = server_module.app.test_client().put(
        "/api/v1/club",
        json={"club": "pw"},
    )

    assert response.status_code == 200
    assert response.get_json() == {"status": "applied", "club": "pw"}
    assert monitor.clubs == [ClubType.PW]


def test_versioned_event_stream_reuses_current_wire_contract(monkeypatch):
    broker = ShotStreamBroker(heartbeat_interval_s=0.01)
    broker.publish(_shot_data())
    monkeypatch.setattr(server_module, "shot_stream", broker)

    response = server_module.app.test_client().get("/api/v1/events")
    try:
        frames = response.response
        assert next(frames).decode("utf-8") == HEARTBEAT_FRAME
        event_frame = next(frames).decode("utf-8")
        payload = json.loads(event_frame.split("data: ", 1)[1])
        assert event_frame.startswith("event: shot\n")
        assert payload["schema_version"] == 1
        assert payload["ball_speed_mph"] == 151.4
    finally:
        response.close()


def test_versioned_calibration_resource_reports_unavailable_hardware(monkeypatch):
    monkeypatch.setattr(server_module, "iwr6843_runtime", None)

    response = server_module.app.test_client().get("/api/v1/calibrations/iwr6843/orientation")

    assert response.status_code == 409
    assert response.get_json() == {"error": "TI IWR6843 radar is not enabled"}


def test_legacy_routes_remain_available(monkeypatch):
    monkeypatch.setattr(server_module, "active_club", ClubType.WOOD_3)

    response = server_module.app.test_client().get("/api/club")

    assert response.status_code == 200
    assert response.get_json() == {"status": "current", "club": "3-wood"}


def test_contract_encoder_has_transport_neutral_ownership():
    from openflight.api.contracts import build_shot_event

    event = build_shot_event(_shot_data(), event_id="shot-1")

    assert event["event_id"] == "shot-1"
    assert event["schema_version"] == 1
