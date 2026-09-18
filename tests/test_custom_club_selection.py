"""Custom club selection, catalog mutations, and shot identity."""

import pytest

from openflight import server
from openflight.clubs import ClubType
from openflight.clubs.store import ClubStore
from openflight.profiles import ProfileStore


@pytest.fixture
def catalog(tmp_path, monkeypatch):
    store = ClubStore(tmp_path / "clubs.json")
    monkeypatch.setattr(server, "club_store", store)
    monkeypatch.setattr(server, "profile_store", ProfileStore(tmp_path / "profiles.json"))
    monkeypatch.setattr(server, "monitor", server.MockLaunchMonitor())
    monkeypatch.setattr(server.socketio, "emit", lambda *args, **kwargs: None)
    monkeypatch.setattr(server, "_register_shot_for_finalization", lambda *args, **kwargs: None)
    monkeypatch.setattr(server, "_finish_shot_detected", lambda *args, **kwargs: None)
    monkeypatch.setattr(server, "_has_slow_shot_enrichment", lambda shot: False)
    return store


def add_club():
    return server.handle_save_custom_club(
        {"name": "My iron", "base_type": "7-iron", "loft_deg": 30}
    )


def test_custom_selection_records_identity_through_rename_and_delete(catalog):
    assert add_club() == {"ok": True}
    custom = catalog.list()[0]
    server.monitor.start(server.on_shot_detected)
    server.handle_set_club({"club": custom.id})
    assert server._session_state_payload()["club"] == custom.id
    assert server.monitor._current_club == ClubType.IRON_7
    shot = server.monitor.simulate_shot(100)
    assert shot.club == ClubType.IRON_7
    assert shot.to_dict()["custom_club_id"] == custom.id
    assert shot.to_dict()["custom_club_name"] == "My iron"
    server.handle_save_custom_club(
        {"id": custom.id, "name": "Renamed", "base_type": "5-iron", "loft_deg": 25}
    )
    assert server.monitor._current_club == ClubType.IRON_5
    assert shot.to_dict()["custom_club_name"] == "My iron"
    assert server.handle_remove_custom_club({"id": custom.id}) == {"ok": True}
    assert server._current_club_id() == "5-iron"
    assert ClubStore(catalog._path).list() == []
    assert shot.to_dict()["custom_club_id"] == custom.id


def test_builtin_selection_clears_custom_identity(catalog):
    add_club()
    server.handle_set_club({"club": catalog.list()[0].id})
    server.handle_set_club({"club": "7-iron"})
    assert server._selected_custom_club() is None
    assert server._current_club_id() == "7-iron"


@pytest.mark.parametrize(
    "changes",
    [
        {"name": " "},
        {"base_type": "unknown"},
        {"base_type": "bad"},
        {"loft_deg": 0},
        {"loft_deg": float("nan")},
    ],
)
def test_invalid_input_does_not_change_catalog(catalog, changes):
    response = server.handle_save_custom_club(
        {"name": "Iron", "base_type": "7-iron", "loft_deg": 30, **changes}
    )
    assert "error" in response
    assert catalog.list() == []


def test_failed_write_keeps_selection_and_catalog(catalog, monkeypatch):
    add_club()
    club = catalog.list()[0]
    server.handle_set_club({"club": club.id})
    monkeypatch.setattr(catalog, "_save_locked", lambda: False)
    assert "error" in server.handle_remove_custom_club({"id": club.id})
    assert server._current_club_id() == club.id
    assert catalog.get(club.id) == club
    server.handle_set_club({"club": "missing"})
    assert server._current_club_id() == club.id
