"""The IQ16 scratch must live outside L3 so capture owns the whole arena.

Map-file coverage in this file comes in three tiers, because `*.map` is
gitignored (it is a fresh build artifact) and CI never runs a firmware build:

* `MAP` (`firmware/iwr6843/l3_dump_mss.map`) is the live map from whatever
  build last ran on this machine. Tests keyed off it (`test_data_ram_keeps_a_
  working_margin`, `test_l3_is_fully_claimed_by_the_capture_ring`,
  `test_derived_arena_matches_the_linker_region`) `pytest.skip` when it is
  absent, which is always true in CI. That skip is deliberate, not a gap.
* `BASELINE_MAP` (`baseline/l3_dump_mss.map.baseline`) is tracked in git and
  captures the geometry from *before* the scratch relocation. It always
  runs, but only proves the relocation had room to happen.
* `CURRENT_MAP` (`baseline/l3_dump_mss.map.current`) is also tracked and
  captures the geometry from *after* the relocation (and after the
  CONFIGURABLE_CAPTURE removal / designated-initializer change in this same
  fix wave, which the build confirmed move nothing). This is the one that
  actually guards the ~20,947 B DATA_RAM margin in CI:
  `test_tracked_reference_map_keeps_the_data_ram_margin` always runs against
  it, and `test_tracked_reference_map_agrees_with_the_live_map` cross-checks
  it against `MAP` whenever a live map happens to be present, so the tracked
  reference cannot silently drift from reality on a machine that does build.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[1]
FIRMWARE = ROOT / "firmware" / "iwr6843" / "l3_dump.c"
MAP = ROOT / "firmware" / "iwr6843" / "l3_dump_mss.map"
BASELINE_MAP = ROOT / "firmware" / "iwr6843" / "baseline" / "l3_dump_mss.map.baseline"
CURRENT_MAP = ROOT / "firmware" / "iwr6843" / "baseline" / "l3_dump_mss.map.current"
MIN_DATA_RAM_FREE_BYTES = 16 * 1024


def _function_source(source: str, name: str, next_name: str) -> str:
    start = source.rindex(name)
    end = source.index(next_name, start)
    return source[start:end]


def test_scratch_is_not_carved_out_of_the_capture_arena():
    """L3_IQ8_CAPTURE_BYTES was deleted outright (its only value was always
    L3_TOTAL_BYTES once the scratch left L3), so the equivalent surviving
    construct is l3_captureCapacityBytes() returning L3_TOTAL_BYTES
    unconditionally, with no offset-cast into g_ring and no IQ8/IQ16 branch
    left over from the old capacity split."""
    source = FIRMWARE.read_text(encoding="utf-8")
    assert "L3_IQ8_CAPTURE_BYTES" not in source
    assert "&g_ring[L3_IQ8_CAPTURE_BYTES]" not in source

    capacity = _function_source(
        source,
        "static uint32_t l3_captureCapacityBytes",
        "static uint32_t l3_captureBytesPerComplex",
    )
    assert "return L3_TOTAL_BYTES;" in capacity
    assert "l3_captureUsesIq8" not in capacity
    assert "L3_RING_IQ8" not in capacity


def test_scratch_is_a_real_array_in_its_own_section():
    source = FIRMWARE.read_text(encoding="utf-8")
    assert 'DATA_SECTION(g_iq16FrameScratch, ".dataScratch")' in source
    assert "static int16_t g_iq16FrameScratch[2][L3_IQ16_SCRATCH_WORDS];" in source


def _memory_rows(path: Path = MAP) -> dict[str, tuple[int, int]]:
    if not path.exists():
        pytest.skip(
            f"no linker map at {path}; this check needs a local firmware build "
            f"(see the plan's Global Constraints for the docker command). The "
            f"_Static_assert in l3_dump.c enforces the scratch size at build "
            f"time regardless, and test_baseline_map_geometry_is_intact below "
            f"always runs."
        )
    rows: dict[str, tuple[int, int]] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = re.match(
            r"\s+(\w+)\s+([0-9a-f]{8})\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)",
            line,
        )
        if match:
            rows.setdefault(match.group(1), (int(match.group(4), 16), int(match.group(5), 16)))
    if not rows:
        pytest.fail(f"could not parse MEMORY CONFIGURATION from {MAP}")
    return rows


def test_data_ram_keeps_a_working_margin():
    _used, unused = _memory_rows()["DATA_RAM"]
    assert unused >= MIN_DATA_RAM_FREE_BYTES, (
        f"DATA_RAM free margin fell to {unused} B, below {MIN_DATA_RAM_FREE_BYTES} B"
    )


def test_l3_is_fully_claimed_by_the_capture_ring():
    used, unused = _memory_rows()["L3_RAM"]
    assert unused == 0
    assert used == 786_432


def test_baseline_map_geometry_is_intact():
    """Always runs: the baseline map IS tracked, unlike the build output.

    Guards the parser itself and catches a corrupted or truncated baseline,
    so CI keeps real coverage even with no toolchain present.
    """
    rows = _memory_rows(BASELINE_MAP)
    assert rows["L3_RAM"] == (786_432, 0)
    baseline_data_ram_free = rows["DATA_RAM"][1]
    assert baseline_data_ram_free >= 98_304 + MIN_DATA_RAM_FREE_BYTES, (
        "the pre-relocation baseline no longer has room for a 98,304 B scratch "
        "plus the required margin; the relocation premise is broken"
    )


def test_tracked_reference_map_keeps_the_data_ram_margin():
    """Always runs: this is the real DATA_RAM margin guard in CI.

    The live-map tests above are the ones a developer's local build actually
    exercises, but they skip in CI because `*.map` is gitignored. This test
    reads the tracked post-relocation reference map instead, so the ~20,947 B
    margin the relocation was meant to protect is checked on every run, not
    only on a machine that happens to have the TI toolchain.
    """
    rows = _memory_rows(CURRENT_MAP)
    used, unused = rows["DATA_RAM"]
    assert unused >= MIN_DATA_RAM_FREE_BYTES, (
        f"tracked reference DATA_RAM free margin is {unused} B, below "
        f"{MIN_DATA_RAM_FREE_BYTES} B"
    )
    assert used + unused == 0x30000

    l3_used, l3_unused = rows["L3_RAM"]
    assert l3_unused == 0
    assert l3_used == 786_432


def test_tracked_reference_map_agrees_with_the_live_map():
    """Guards against the tracked reference map going stale.

    Skips when there is no live map to compare against (the normal CI case);
    on a machine that just built, it fails loudly if `l3_dump_mss.map.current`
    no longer matches reality, so nobody has to notice a silent drift by hand.
    """
    if not MAP.exists():
        pytest.skip(
            f"no live linker map at {MAP} to compare against the tracked "
            f"reference; see the plan's Global Constraints for the docker "
            f"build command."
        )
    live_rows = _memory_rows(MAP)
    reference_rows = _memory_rows(CURRENT_MAP)
    assert live_rows["L3_RAM"] == reference_rows["L3_RAM"], (
        "live L3_RAM geometry no longer matches the tracked reference map; "
        "regenerate baseline/l3_dump_mss.map.current from a fresh build"
    )
    assert live_rows["DATA_RAM"] == reference_rows["DATA_RAM"], (
        "live DATA_RAM geometry no longer matches the tracked reference map; "
        "regenerate baseline/l3_dump_mss.map.current from a fresh build"
    )


def test_the_l3track_workspace_lives_in_hs_ram_and_is_cleared_before_use():
    """DATA_RAM was down to 926 B (2026-10-01 build of the reconstruction).
    The 17.4 KB l3track workspace is only used on a frozen ring, so it sits in
    HS-RAM with the other MSS diagnostics. HS-RAM is not zeroed at boot: the
    selector must clear what it reads before it scans."""
    source = FIRMWARE.read_text(encoding="utf-8")
    assert "static L3TrackWorkspace gTrackWorkspace L3_HSRAM_DIAG;" in source

    selector = (FIRMWARE.parent / "track_select.c").read_text(encoding="utf-8")
    select = selector[selector.index("int32_t l3track_select(") :]
    scan = select.index("l3track_scan(layout")
    for cleared in ("memset(ws->detCount, 0,", "memset(ws->cellMask, 0,"):
        assert 0 <= select.find(cleared) < scan, f"{cleared} must precede the scan"
    assert "ws->nOrder = 0U;" in selector
