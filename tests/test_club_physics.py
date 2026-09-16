from dataclasses import FrozenInstanceError

import pytest

from openflight.club_physics import CLUB_PHYSICS, get_club_physics
from openflight.launch_monitor import ClubType


def test_physics_registry_covers_every_club_type():
    assert set(CLUB_PHYSICS) == set(ClubType)


def test_seven_iron_has_canonical_defaults():
    physics = get_club_physics(ClubType.IRON_7)

    assert physics.nominal_loft_deg == 34.0
    assert physics.optimal_launch_deg == 20.5
    assert physics.average_ball_speed_mph == 100
    assert physics.launch_deg_per_mph == 0.30
    assert physics.typical_spin_rpm == 6500
    assert physics.optimal_smash == 1.34


def test_physics_records_are_immutable():
    physics = get_club_physics(ClubType.DRIVER)

    with pytest.raises(FrozenInstanceError):
        physics.nominal_loft_deg = 12.0
