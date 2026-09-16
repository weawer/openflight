import json

from openflight.club_store import ClubStore, CustomClub
from openflight.launch_monitor import ClubType


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
