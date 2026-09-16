"""Immutable built-in club categories and their canonical physics defaults."""

from __future__ import annotations

from dataclasses import dataclass

from .launch_monitor import ClubType


@dataclass(frozen=True)
class ClubPhysics:
    """Version-controlled defaults for one built-in club category."""

    nominal_loft_deg: float
    optimal_launch_deg: float
    average_ball_speed_mph: float
    launch_deg_per_mph: float
    typical_spin_rpm: float
    optimal_smash: float


@dataclass(frozen=True)
class ClubSimulationProfile:
    """Variability inputs used to generate realistic development shots."""

    ball_speed_std_dev_mph: float
    average_smash: float
    spin_std_dev_rpm: float
    launch_std_dev_deg: float


@dataclass(frozen=True)
class ShotSimulationDefaults:
    """Shared bounds and distributions for development shot generation."""

    min_ball_speed_mph: float = 50
    max_ball_speed_mph: float = 200
    smash_variation: float = 0.03
    min_spin_rpm: float = 1000
    min_launch_deg: float = 5.0
    horizontal_launch_std_dev_deg: float = 2.0
    confidence_min: float = 0.5
    confidence_max: float = 0.95
    angle_of_attack_mean_deg: float = -4.0
    angle_of_attack_std_dev_deg: float = 2.5
    spin_confidence_choices: tuple[float, ...] = (0.3, 0.6, 0.7, 0.9)
    club_path_max_abs_deg: float = 5.0
    spin_axis_error_max_abs_deg: float = 5.0


CLUB_PHYSICS: dict[ClubType, ClubPhysics] = {
    #                           loft launch speed sensitivity spin  smash
    ClubType.DRIVER: ClubPhysics(10.5, 11.0, 143, 0.15, 2700, 1.48),
    ClubType.WOOD_3: ClubPhysics(15.0, 12.5, 135, 0.18, 3500, 1.44),
    ClubType.WOOD_5: ClubPhysics(18.0, 14.0, 128, 0.20, 4200, 1.42),
    ClubType.WOOD_7: ClubPhysics(21.0, 15.5, 122, 0.20, 4800, 1.42),
    ClubType.HYBRID_3: ClubPhysics(19.0, 13.5, 123, 0.22, 4400, 1.39),
    ClubType.HYBRID_5: ClubPhysics(22.0, 15.0, 118, 0.22, 4900, 1.38),
    ClubType.HYBRID_7: ClubPhysics(25.0, 16.5, 112, 0.25, 5300, 1.37),
    ClubType.HYBRID_9: ClubPhysics(28.0, 18.0, 106, 0.25, 5800, 1.36),
    ClubType.IRON_2: ClubPhysics(18.0, 13.0, 120, 0.25, 4000, 1.37),
    ClubType.IRON_3: ClubPhysics(21.0, 14.5, 118, 0.25, 4500, 1.36),
    ClubType.IRON_4: ClubPhysics(24.0, 16.0, 114, 0.28, 5000, 1.35),
    ClubType.IRON_5: ClubPhysics(27.0, 17.5, 110, 0.28, 5400, 1.35),
    ClubType.IRON_6: ClubPhysics(30.5, 19.0, 105, 0.30, 6000, 1.34),
    ClubType.IRON_7: ClubPhysics(34.0, 20.5, 100, 0.30, 6500, 1.34),
    ClubType.IRON_8: ClubPhysics(38.0, 23.0, 94, 0.30, 7500, 1.33),
    ClubType.IRON_9: ClubPhysics(42.0, 25.5, 88, 0.30, 8500, 1.33),
    ClubType.PW: ClubPhysics(46.0, 28.0, 82, 0.30, 9000, 1.25),
    ClubType.GW: ClubPhysics(50.0, 30.0, 76, 0.30, 9500, 1.23),
    ClubType.SW: ClubPhysics(54.0, 32.0, 73, 0.30, 10000, 1.22),
    ClubType.LW: ClubPhysics(58.0, 35.0, 70, 0.30, 10500, 1.20),
    ClubType.UNKNOWN: ClubPhysics(34.0, 18.0, 120, 0.25, 5000, 1.35),
}


CLUB_SIMULATION_PROFILES: dict[ClubType, ClubSimulationProfile] = {
    #                           speed sd  smash  spin sd  launch sd
    ClubType.DRIVER: ClubSimulationProfile(12, 1.45, 400, 2.0),
    ClubType.WOOD_3: ClubSimulationProfile(10, 1.42, 400, 2.0),
    ClubType.WOOD_5: ClubSimulationProfile(10, 1.40, 400, 2.0),
    ClubType.WOOD_7: ClubSimulationProfile(9, 1.40, 500, 2.0),
    ClubType.HYBRID_3: ClubSimulationProfile(9, 1.39, 400, 2.0),
    ClubType.HYBRID_5: ClubSimulationProfile(9, 1.37, 500, 2.0),
    ClubType.HYBRID_7: ClubSimulationProfile(8, 1.35, 500, 2.0),
    ClubType.HYBRID_9: ClubSimulationProfile(8, 1.33, 500, 2.5),
    ClubType.IRON_2: ClubSimulationProfile(9, 1.35, 400, 2.0),
    ClubType.IRON_3: ClubSimulationProfile(9, 1.35, 400, 2.0),
    ClubType.IRON_4: ClubSimulationProfile(8, 1.33, 500, 2.0),
    ClubType.IRON_5: ClubSimulationProfile(8, 1.31, 500, 2.0),
    ClubType.IRON_6: ClubSimulationProfile(7, 1.29, 600, 2.5),
    ClubType.IRON_7: ClubSimulationProfile(7, 1.27, 600, 2.5),
    ClubType.IRON_8: ClubSimulationProfile(6, 1.25, 700, 3.0),
    ClubType.IRON_9: ClubSimulationProfile(6, 1.23, 800, 3.0),
    ClubType.PW: ClubSimulationProfile(5, 1.21, 800, 3.0),
    ClubType.GW: ClubSimulationProfile(5, 1.20, 900, 3.5),
    ClubType.SW: ClubSimulationProfile(5, 1.19, 1000, 4.0),
    ClubType.LW: ClubSimulationProfile(5, 1.18, 1000, 4.0),
    ClubType.UNKNOWN: ClubSimulationProfile(15, 1.35, 800, 3.0),
}


SHOT_SIMULATION_DEFAULTS = ShotSimulationDefaults()


def get_club_physics(club_type: ClubType) -> ClubPhysics:
    """Return canonical defaults, falling back to the unknown category."""
    return CLUB_PHYSICS.get(club_type, CLUB_PHYSICS[ClubType.UNKNOWN])


def get_club_simulation_profile(club_type: ClubType) -> ClubSimulationProfile:
    """Return mock generation inputs, falling back to the unknown category."""
    return CLUB_SIMULATION_PROFILES.get(club_type, CLUB_SIMULATION_PROFILES[ClubType.UNKNOWN])
