"""The board image's feature switches (L3_FEATURE_DEFS in firmware/iwr6843/makefile).

DATA_RAM on the R4F is shared by the IQ16 scratch, the heap, .data, .bss and
the stacks, and the tracker state outgrew it. The board image therefore
compiles out what is not needed on the board right now: the ball-hypothesis
search (off at run time anyway) and most of the trigger's raw-input trace.
The code stays in the tree and the default host build keeps all of it.

These tests pin the switch list to the makefile, build the modules with that
exact list, and check the board variant is smaller, still compiles warning
free, and tracks the ball the way the default build does with the search off.
Structs whose layout the switches change are only touched through the C.
"""

from __future__ import annotations

import ctypes
import re
from pathlib import Path

import pytest
from iwr6843_twotrack import BIN_M, TwoTracks
from test_iwr6843_firmware_trigger import CLUB, FrontEnd, make_cfg

from openflight.iwr6843 import firmware_host as fw

FIRMWARE_DIR = Path(__file__).parents[1] / "firmware" / "iwr6843"
APP_MAKEFILE = FIRMWARE_DIR / "makefile"
TOP_MAKEFILE = FIRMWARE_DIR.parent / "Makefile"
BOARD_TRACE_DEPTH = 24


def board_defines() -> tuple[str, ...]:
    """The ``NAME=VALUE`` pairs of the makefile's default L3_FEATURE_DEFS."""
    makefile = APP_MAKEFILE.read_text(encoding="utf-8").replace("\\\n", " ")
    match = re.search(r"^L3_FEATURE_DEFS\s*\?=(.*)$", makefile, re.MULTILINE)
    assert match, "L3_FEATURE_DEFS ?= ... must stay in the app makefile"
    return tuple(re.findall(r"--define=(\S+)", match.group(1)))


@pytest.fixture(scope="module")
def host(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_host"))


@pytest.fixture(scope="module")
def board(tmp_path_factory):
    if fw.host_compiler() is None:
        pytest.skip("no C compiler for the firmware modules")
    return fw.build_firmware_library(tmp_path_factory.mktemp("l3_board"), defines=board_defines())


# --- the makefile ------------------------------------------------------------


def test_the_board_image_leaves_out_the_search_and_most_of_the_trace():
    assert dict(define.split("=", 1) for define in board_defines()) == {
        "L3_BALL_HYPOTHESES": "0",
        "L3_BALL_RECOVER": "0",
        "L3_TRIG_TRACE_DEPTH": f"{BOARD_TRACE_DEPTH}U",
    }


def test_every_board_build_gets_the_switches():
    makefile = APP_MAKEFILE.read_text(encoding="utf-8")
    assert re.search(r"^R4F_CFLAGS \+= \$\(L3_FEATURE_DEFS\)$", makefile, re.MULTILINE)
    # The production target passes geometry only; overriding the switch list
    # there would silently put the hypotheses back into the release image.
    assert "L3_FEATURE_DEFS" not in TOP_MAKEFILE.read_text(encoding="utf-8")


def test_the_switched_out_code_stays_in_the_tree_and_the_build():
    sources = re.search(
        r"^SOURCES\s*=(.*)$", APP_MAKEFILE.read_text(encoding="utf-8"), re.MULTILINE
    ).group(1)
    assert "l3_ball_hyp.c" in sources.split(), "kept compiling so it cannot rot"
    assert "l3_ball_hyp.c" in fw.HOST_SOURCES


def test_the_host_defaults_keep_everything():
    hyp = (FIRMWARE_DIR / "l3_ball_hyp.h").read_text(encoding="utf-8")
    trigger = (FIRMWARE_DIR / "l3_trigger.h").read_text(encoding="utf-8")
    assert "#ifndef L3_BALL_HYPOTHESES\n#define L3_BALL_HYPOTHESES 1\n#endif" in hyp
    assert "L3_TRIG_LOG_DEPTH" not in trigger, "the gate's flight recorder went with the gate"
    assert "#ifndef L3_TRIG_TRACE_DEPTH\n#define L3_TRIG_TRACE_DEPTH       64U\n#endif" in trigger
    assert fw.TRIG_TRACE_DEPTH == 64


def test_the_hypothesis_angle_loop_is_compiled_out_with_the_search():
    source = (FIRMWARE_DIR / "l3_dump.c").read_text(encoding="utf-8")
    start = source.index("#if L3_BALL_HYPOTHESES")
    end = source.index("#endif /* L3_BALL_HYPOTHESES */", start)
    guarded = source[start:end]
    assert "gBallTrack.hyps" in guarded
    assert "gBallTrack.hyps" not in source[:start] + source[end:]


# --- the board variant, built ------------------------------------------------


def test_the_board_ball_track_drops_exactly_the_hypotheses(host, board):
    host_bytes = host.l3_ball_track_struct_bytes()
    assert host_bytes == ctypes.sizeof(fw.BallTrack)
    saved = ctypes.sizeof(fw.BallHyps) + ctypes.sizeof(fw.BallHypVerdict)
    saved += ctypes.sizeof(fw.BallHypsCfg)  # the track's copy of its cfg
    saved += ctypes.sizeof(fw.BallHistory)  # L3_BALL_RECOVER
    saved += ctypes.sizeof(fw.BallRecoverCfg) + 8  # recover, historySnr
    assert board.l3_ball_track_struct_bytes() == host_bytes - saved


class OpaqueBallTrack:
    """A ball track in a buffer sized by the library that owns its layout."""

    def __init__(self, lib, use_hypotheses: int = 0):
        self.lib = lib
        cfg = fw.BallTrackCfg()  # the board cfg is a prefix of this one
        lib.l3_ball_track_cfg_defaults(ctypes.byref(cfg))
        cfg.useHypotheses = use_hypotheses
        self.buffer = ctypes.create_string_buffer(lib.l3_ball_track_struct_bytes())
        self.ptr = ctypes.cast(self.buffer, ctypes.POINTER(fw.BallTrack))
        lib.l3_ball_track_init(self.ptr, ctypes.byref(cfg))

    def run(self, scene: TwoTracks) -> list[tuple[int, str]]:
        origin = fw.Vec3(scene.origin_bin * BIN_M, 0.0, 0.0)
        anchor = fw.BallAnchor(
            anchorBin=scene.origin_bin,
            acceptFromBin=scene.origin_bin,
            gateUs=scene.gate_us,
            anchorUs=scene.gate_us,
            anchorTolUs=15_000,
        )
        self.lib.l3_ball_track_arm(self.ptr, ctypes.byref(anchor), ctypes.byref(origin))
        seen = []
        for f in scene.build():
            arr = (fw.TargetObs * max(1, len(f.targets)))(*f.targets)
            appended = self.lib.l3_ball_track_update_joint(
                self.ptr, arr, len(f.targets), f.frame, f.timestamp_us, f.club_index
            )
            seen.append((appended, fw.c_text(self.lib.l3_ball_track_format_status, self.ptr)))
        return seen

    def launch(self) -> tuple[int, bytes]:
        out = fw.Launch()
        used = self.lib.l3_ball_track_launch(self.ptr, ctypes.byref(out))
        return used, bytes(out)


@pytest.mark.parametrize(
    "scene",
    [
        TwoTracks(),
        TwoTracks(club_visible=False),
        TwoTracks(missing_ball=(2, 3)),
        TwoTracks(merged=(1, 2)),
        TwoTracks(ball_mps=65.0, frames=10),
        TwoTracks(missing_ball=tuple(range(1, 9))),  # no ball at all
    ],
    ids=["two-tracks", "lone-ball", "ball-gaps", "merged", "fast-ball", "no-ball"],
)
def test_the_board_ball_track_is_the_default_one_with_the_search_off(host, board, scene):
    expected = OpaqueBallTrack(host)
    got = OpaqueBallTrack(board)
    assert got.run(scene) == expected.run(scene)
    assert got.launch() == expected.launch()


def test_asking_the_board_for_the_search_gets_the_plain_track(host, board):
    scene = TwoTracks()
    expected = OpaqueBallTrack(host, use_hypotheses=0)
    got = OpaqueBallTrack(board, use_hypotheses=1)
    assert got.run(scene) == expected.run(scene)
    assert got.launch() == expected.launch()


def test_the_board_trace_keeps_the_newest_24_frames(board):
    det = FrontEnd(board, make_cfg(board))
    for _ in range(BOARD_TRACE_DEPTH + 10):
        det.feed({12: CLUB})
    traces = det.traces()
    assert len(traces) == BOARD_TRACE_DEPTH
    assert traces[0].frame == 11
    assert traces[-1].frame == BOARD_TRACE_DEPTH + 10
