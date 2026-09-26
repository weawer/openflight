"""Native tests for lossless IQ16 range-window compaction."""

from __future__ import annotations

import ctypes
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[1]
SOURCE = ROOT / "firmware" / "iwr6843" / "compact_iq16.c"


@pytest.fixture(scope="module")
def compact(tmp_path_factory: pytest.TempPathFactory):
    library = tmp_path_factory.mktemp("compact-iq16") / "compact_iq16.so"
    subprocess.run(
        [
            "cc",
            "-shared",
            "-fPIC",
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-o",
            str(library),
            str(SOURCE),
        ],
        check=True,
    )
    function = ctypes.CDLL(str(library)).l3_compact_iq16
    function.argtypes = [
        ctypes.POINTER(ctypes.c_int16),
        ctypes.POINTER(ctypes.c_int16),
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.c_uint16,
        ctypes.c_uint16,
    ]
    function.restype = ctypes.c_int32
    return function


def test_compacts_every_chirp_and_rx_without_changing_iq16_values(compact):
    chirps, receivers, full_bins = 6, 4, 128
    bin_start, bin_count = 20, 32
    values = [
        ((chirp * 4000 + rx * 500 + bin_index * 2 + component) % 65536) - 32768
        for chirp in range(chirps)
        for rx in range(receivers)
        for bin_index in range(full_bins)
        for component in range(2)
    ]
    source = (ctypes.c_int16 * len(values))(*values)
    output_words = chirps * receivers * bin_count * 2
    output = (ctypes.c_int16 * output_words)()

    assert compact(
        source, output, chirps, receivers, full_bins, bin_start, bin_count
    ) == 0

    expected = []
    for chirp in range(chirps):
        for rx in range(receivers):
            base = ((chirp * receivers + rx) * full_bins + bin_start) * 2
            expected.extend(values[base : base + bin_count * 2])
    assert list(output) == expected


@pytest.mark.parametrize(
    ("bin_start", "bin_count"),
    [(128, 1), (120, 9), (0, 0)],
)
def test_rejects_invalid_windows_without_touching_destination(
    compact, bin_start, bin_count
):
    source = (ctypes.c_int16 * (4 * 128 * 2))()
    output = (ctypes.c_int16 * 16)(*([1234] * 16))

    assert compact(source, output, 1, 4, 128, bin_start, bin_count) == -1
    assert list(output) == [1234] * 16
