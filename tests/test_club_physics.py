from dataclasses import FrozenInstanceError

import pytest

from openflight.club_physics import (
    CLUB_PHYSICS,
    CLUB_SIMULATION_PROFILES,
    SHOT_SIMULATION_DEFAULTS,
    ClubType,
    get_club_physics,
    get_club_simulation_profile,
)


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


def test_simulation_registry_covers_every_club_type():
    assert set(CLUB_SIMULATION_PROFILES) == set(ClubType)


def test_seven_iron_has_canonical_simulation_profile():
    profile = get_club_simulation_profile(ClubType.IRON_7)

    assert profile.ball_speed_std_dev_mph == 7
    assert profile.average_smash == 1.27
    assert profile.spin_std_dev_rpm == 600
    assert profile.launch_std_dev_deg == 2.5


def test_simulation_profiles_are_immutable():
    profile = get_club_simulation_profile(ClubType.DRIVER)

    with pytest.raises(FrozenInstanceError):
        profile.average_smash = 1.5


def test_shot_simulation_defaults_are_immutable():
    with pytest.raises(FrozenInstanceError):
        SHOT_SIMULATION_DEFAULTS.min_ball_speed_mph = 40
