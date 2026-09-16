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


def get_club_physics(club_type: ClubType) -> ClubPhysics:
    """Return canonical defaults, falling back to the unknown category."""
    return CLUB_PHYSICS.get(club_type, CLUB_PHYSICS[ClubType.UNKNOWN])
