import json
import threading

import pytest

import openflight.club_store as club_store_module
from openflight.club_store import ClubStore, CustomClub
from openflight.clubs import ClubType


def test_creates_empty_catalog_when_file_is_missing(tmp_path):
    path = tmp_path / "config" / "clubs.json"

    store = ClubStore(path)

    assert store.list() == []
    assert json.loads(path.read_text(encoding="utf-8")) == {
        "version": 1,
        "clubs": [],
    }


def test_adds_and_reloads_custom_club(tmp_path):
    path = tmp_path / "clubs.json"
    store = ClubStore(path)

    added = store.add(
        name="P790 7 Iron",
        label="7i",
        group="Irons",
        base_type=ClubType.IRON_7,
        loft_deg=30.5,
        sort_order=7,
    )

    assert added is not None
    assert ClubStore(path).get(added.id) == added
    payload = json.loads(path.read_text(encoding="utf-8"))
    assert payload["version"] == 1
    assert payload["clubs"][0]["base_type"] == "7-iron"
    assert payload["clubs"][0]["loft_deg"] == 30.5


def test_rejects_invalid_base_type_and_loft(tmp_path):
    store = ClubStore(tmp_path / "clubs.json")

    assert (
        store.add(
            name="Invalid",
            label="X",
            group="Irons",
            base_type="not-a-type",
            loft_deg=34,
        )
        is None
    )
    assert (
        store.add(
            name="Invalid",
            label="X",
            group="Irons",
            base_type=ClubType.IRON_7,
            loft_deg=0,
        )
        is None
    )
    assert store.list() == []


def test_updates_metadata_without_changing_id(tmp_path):
    store = ClubStore(tmp_path / "clubs.json")
    added = store.add(
        name="Old Name",
        label="7i",
        group="Irons",
        base_type=ClubType.IRON_7,
        loft_deg=34,
    )
    assert added is not None

    updated = store.update(added.id, name="New Name", loft_deg=30.5)

    assert updated == CustomClub(
        id=added.id,
        name="New Name",
        label="7i",
        group="Irons",
        base_type=ClubType.IRON_7,
        loft_deg=30.5,
    )
    assert ClubStore(tmp_path / "clubs.json").get(added.id) == updated


def test_removes_custom_club(tmp_path):
    path = tmp_path / "clubs.json"
    store = ClubStore(path)
    added = store.add(
        name="Utility Iron",
        label="UI",
        group="Irons",
        base_type=ClubType.IRON_2,
        loft_deg=17,
    )
    assert added is not None

    assert store.remove(added.id)
    assert ClubStore(path).list() == []


def test_skips_invalid_and_duplicate_records_on_load(tmp_path):
    path = tmp_path / "clubs.json"
    valid = {
        "id": "stable-id",
        "name": "Valid",
        "label": "7i",
        "group": "Irons",
        "base_type": "7-iron",
        "loft_deg": 34,
        "enabled": True,
        "sort_order": 1,
    }
    path.write_text(
        json.dumps({"version": 1, "clubs": [valid, valid, {"id": "bad"}]}),
        encoding="utf-8",
    )

    assert ClubStore(path).list() == [CustomClub.from_dict(valid)]


def test_rejects_future_schema_without_rewriting_file(tmp_path, caplog):
    path = tmp_path / "clubs.json"
    payload = {"version": 2, "clubs": []}
    original = json.dumps(payload)
    path.write_text(original, encoding="utf-8")

    store = ClubStore(path)

    assert store.list() == []
    assert path.read_text(encoding="utf-8") == original
    assert "unsupported schema version" in caplog.text


@pytest.mark.parametrize("enabled", [None, 0, 1, "false", "true"])
def test_rejects_non_boolean_enabled_values(enabled):
    raw = {
        "id": "stable-id",
        "name": "Valid",
        "label": "7i",
        "group": "Irons",
        "base_type": "7-iron",
        "loft_deg": 34,
        "enabled": enabled,
    }

    assert CustomClub.from_dict(raw) is None


def test_add_rolls_back_when_persistence_fails(tmp_path, monkeypatch):
    path = tmp_path / "clubs.json"
    store = ClubStore(path)
    original = path.read_text(encoding="utf-8")
    monkeypatch.setattr(
        "openflight.club_store.os.replace",
        lambda *_args: (_ for _ in ()).throw(OSError("disk full")),
    )

    added = store.add(
        name="P790 7 Iron",
        label="7i",
        group="Irons",
        base_type=ClubType.IRON_7,
        loft_deg=30.5,
    )

    assert added is None
    assert store.list() == []
    assert path.read_text(encoding="utf-8") == original


def test_update_and_remove_roll_back_when_persistence_fails(tmp_path, monkeypatch):
    path = tmp_path / "clubs.json"
    store = ClubStore(path)
    added = store.add(
        name="Original",
        label="7i",
        group="Irons",
        base_type=ClubType.IRON_7,
        loft_deg=34,
    )
    assert added is not None
    original = path.read_text(encoding="utf-8")
    monkeypatch.setattr(
        "openflight.club_store.os.replace",
        lambda *_args: (_ for _ in ()).throw(OSError("disk full")),
    )

    assert store.update(added.id, name="Changed") is None
    assert store.get(added.id) == added
    assert store.remove(added.id) is False
    assert store.get(added.id) == added
    assert path.read_text(encoding="utf-8") == original


def test_save_reports_failure(tmp_path, monkeypatch):
    store = ClubStore(tmp_path / "clubs.json")
    monkeypatch.setattr(
        "openflight.club_store.os.replace",
        lambda *_args: (_ for _ in ()).throw(OSError("disk full")),
    )

    assert store.save() is False


@pytest.mark.parametrize("failure_stage", ["write", "fsync", "replace"])
def test_add_rolls_back_at_each_persistence_stage(tmp_path, monkeypatch, failure_stage):
    path = tmp_path / "clubs.json"
    store = ClubStore(path)
    original = path.read_text(encoding="utf-8")

    def fail(*_args, **_kwargs):
        raise OSError("disk failure")

    if failure_stage == "write":
        monkeypatch.setattr(club_store_module.json, "dump", fail)
    elif failure_stage == "fsync":
        monkeypatch.setattr(club_store_module.os, "fsync", fail)
    else:
        monkeypatch.setattr(club_store_module.os, "replace", fail)

    added = store.add(
        name="P790 7 Iron",
        label="7i",
        group="Irons",
        base_type=ClubType.IRON_7,
        loft_deg=30.5,
    )

    assert added is None
    assert store.list() == []
    assert path.read_text(encoding="utf-8") == original
    assert list(tmp_path.glob("*.tmp")) == []


def test_save_holds_lock_through_atomic_replace(tmp_path, monkeypatch):
    store = ClubStore(tmp_path / "clubs.json")
    replace_started = threading.Event()
    release_replace = threading.Event()
    mutation_finished = threading.Event()
    real_replace = __import__("os").replace
    added_clubs = []
    save_results = []
    calls = 0

    def blocking_replace(source, destination):
        nonlocal calls
        calls += 1
        if calls == 1:
            replace_started.set()
            assert release_replace.wait(timeout=2)
        real_replace(source, destination)

    monkeypatch.setattr("openflight.club_store.os.replace", blocking_replace)
    save_thread = threading.Thread(target=lambda: save_results.append(store.save()))
    save_thread.start()
    assert replace_started.wait(timeout=2)

    def add_club():
        added_clubs.append(
            store.add(
                name="Blocked",
                label="B",
                group="Irons",
                base_type=ClubType.IRON_7,
                loft_deg=34,
            )
        )
        mutation_finished.set()

    mutation_thread = threading.Thread(target=add_club)
    mutation_thread.start()
    assert not mutation_finished.wait(timeout=0.1)
    release_replace.set()
    save_thread.join(timeout=2)
    mutation_thread.join(timeout=2)

    assert not save_thread.is_alive()
    assert not mutation_thread.is_alive()
    assert mutation_finished.is_set()
    assert save_results == [True]
    assert added_clubs[0] is not None
    assert store.list() == [added_clubs[0]]
    assert ClubStore(tmp_path / "clubs.json").list() == [added_clubs[0]]
