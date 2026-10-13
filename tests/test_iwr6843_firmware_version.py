"""Firmware release versioning and the CLI ``version`` command, end to end."""

from __future__ import annotations

import logging
import re
import shutil
import subprocess
from pathlib import Path

import pytest

import openflight.iwr6843.driver as driver_module
from openflight.iwr6843 import firmware_cli as cli, firmware_version as fw
from openflight.iwr6843.firmware_version import (
    FirmwareVersion,
    SemVer,
    bump_release_version,
    find_version_line,
    parse_version_reply,
    read_release_version,
)
from openflight.iwr6843.monitor import IWR6843CaptureMonitor
from tests.test_iwr6843_driver import _command_radar
from tests.test_iwr6843_monitor import FakeButton, FakeRadar, _raw_dump

REPO = Path(__file__).parents[1]
FIRMWARE_DIR = REPO / "firmware"
FIRMWARE_SOURCE = FIRMWARE_DIR / "iwr6843" / "l3_dump.c"
VERSION_HEADER = FIRMWARE_DIR / "iwr6843" / "fw_version.h"
TOP_MAKEFILE = FIRMWARE_DIR / "Makefile"
APP_MAKEFILE = FIRMWARE_DIR / "iwr6843" / "makefile"

REPLY = (
    "stats version\r\n"
    "version=1.4.2 git=b6f4c36d2286-dirty variant=hybrid-cadence "
    "built=2026-09-30T15:37:51Z\r\n"
    "Done\r\n"
    "l3dump:/>"
)


# --- SemVer ------------------------------------------------------------------


@pytest.mark.parametrize(
    "text,expected",
    [
        ("0.0.0", SemVer(0, 0, 0)),
        ("1.2.3", SemVer(1, 2, 3)),
        ("10.20.30", SemVer(10, 20, 30)),
        ("  1.0.0\n", SemVer(1, 0, 0)),
    ],
)
def test_semver_parses_release_versions(text, expected):
    assert SemVer.parse(text) == expected


@pytest.mark.parametrize(
    "text",
    ["", "1", "1.0", "1.0.0.0", "v1.0.0", "01.0.0", "1.00.0", "1.0.0-rc1", "1.0.x", "-1.0.0"],
)
def test_semver_rejects_anything_but_major_minor_patch(text):
    with pytest.raises(ValueError, match="MAJOR.MINOR.PATCH"):
        SemVer.parse(text)


@pytest.mark.parametrize(
    "part,expected",
    [("patch", "1.2.4"), ("minor", "1.3.0"), ("major", "2.0.0")],
)
def test_bump_resets_the_lower_parts(part, expected):
    assert str(SemVer(1, 2, 3).bump(part)) == expected


def test_bump_rejects_an_unknown_part():
    with pytest.raises(ValueError, match="unknown version part"):
        SemVer(1, 2, 3).bump("build")


# --- the fork tag (2026-10-13) -------------------------------------------------
#
# Two different images were both released as 1.0.3: Cormac's feat/iwr-calcs
# build (4e4e9c5d94b7, the one on the 2026-10-12 TrackMan bench) and this
# branch's, with the club-in span gate. This branch's versions carry ".j".


@pytest.mark.parametrize(
    "text,expected",
    [("1.0.3.j", SemVer(1, 0, 3, "j")), ("2.10.0.jx", SemVer(2, 10, 0, "jx"))],
)
def test_semver_parses_a_fork_tag(text, expected):
    assert SemVer.parse(text) == expected
    assert str(expected) == text


@pytest.mark.parametrize("text", ["1.0.3.", "1.0.3.J", "1.0.3.j1", "1.0.3-j", "1.0.3.j.k"])
def test_a_fork_tag_is_lowercase_letters_only(text):
    with pytest.raises(ValueError, match="MAJOR.MINOR.PATCH"):
        SemVer.parse(text)


def test_a_tagged_version_is_not_the_untagged_one():
    assert SemVer.parse("1.0.3.j") != SemVer.parse("1.0.3")


@pytest.mark.parametrize(
    "part,expected",
    [("patch", "1.0.4.j"), ("minor", "1.1.0.j"), ("major", "2.0.0.j")],
)
def test_bump_keeps_the_fork_tag(part, expected):
    assert str(SemVer(1, 0, 3, "j").bump(part)) == expected


def test_this_branchs_versions_carry_the_j_tag():
    assert read_release_version().tag == "j"


def test_semver_orders_numerically_not_lexically():
    assert SemVer.parse("1.10.0") > SemVer.parse("1.9.9")
    assert SemVer.parse("2.0.0") > SemVer.parse("1.99.99")
    assert SemVer.parse("1.0.10.j") > SemVer.parse("1.0.9.j")


# --- firmware/VERSION --------------------------------------------------------


def test_repository_version_file_is_a_valid_release_version():
    assert fw.VERSION_FILE == FIRMWARE_DIR / "VERSION"
    read_release_version()


def test_bump_rewrites_the_version_file(tmp_path):
    path = tmp_path / "VERSION"
    path.write_text("1.2.3\n", encoding="utf-8")

    old, new = bump_release_version("minor", path)

    assert (old, new) == (SemVer(1, 2, 3), SemVer(1, 3, 0))
    assert path.read_text(encoding="utf-8") == "1.3.0\n"


def test_bump_rewrites_a_tagged_version_file(tmp_path):
    path = tmp_path / "VERSION"
    path.write_text("1.0.3.j\n", encoding="utf-8")

    old, new = bump_release_version("patch", path)

    assert (str(old), str(new)) == ("1.0.3.j", "1.0.4.j")
    assert path.read_text(encoding="utf-8") == "1.0.4.j\n"


def test_bump_leaves_a_malformed_version_file_untouched(tmp_path):
    path = tmp_path / "VERSION"
    path.write_text("1.2\n", encoding="utf-8")

    with pytest.raises(ValueError):
        bump_release_version("patch", path)
    assert path.read_text(encoding="utf-8") == "1.2\n"


def test_bump_with_a_missing_version_file_raises(tmp_path):
    with pytest.raises(OSError):
        bump_release_version("patch", tmp_path / "VERSION")


# --- CLI reply parsing -------------------------------------------------------


def test_reply_parses_around_echo_done_and_prompt():
    assert parse_version_reply(REPLY) == FirmwareVersion(
        version="1.4.2",
        git="b6f4c36d2286-dirty",
        variant="hybrid-cadence",
        built="2026-09-30T15:37:51Z",
    )


def test_reply_ignores_keys_it_does_not_know():
    reply = "version=1.0.0 git=abc variant=v built=t extra=1\nDone\n"
    assert parse_version_reply(reply).version == "1.0.0"


@pytest.mark.parametrize(
    "line,missing",
    [
        ("version=1.0.0 variant=v built=t", "git"),
        ("version=1.0.0 git= variant=v built=t", "git"),
        ("version= git=a variant=v built=t", "version"),
        ("version=1.0.0 git=a variant=v", "built"),
    ],
)
def test_reply_with_a_missing_or_empty_key_is_rejected(line, missing):
    with pytest.raises(ValueError, match=missing):
        parse_version_reply(line + "\nDone\n")


@pytest.mark.parametrize("reply", ["", "Done\n", "frames=1 active=0\nDone\n"])
def test_reply_without_a_version_line_is_rejected(reply):
    with pytest.raises(ValueError, match="no version line"):
        parse_version_reply(reply)


def test_version_line_must_start_the_line():
    # A counter whose name ends in "version" must not pass for the identity.
    assert find_version_line("frames=1 dump_version=7 active=0\nDone\n") is None
    assert find_version_line("  version=1.0.0 git=a\n") == "version=1.0.0 git=a"


def test_firmware_version_string_is_readable():
    text = str(parse_version_reply(REPLY))
    assert text == "1.4.2 (git b6f4c36d2286-dirty, hybrid-cadence, built 2026-09-30T15:37:51Z)"


# --- driver ------------------------------------------------------------------


def test_driver_sends_stats_version_and_parses_the_reply():
    radar = _command_radar(b"", reply=REPLY.split("\r\n", 1)[1].encode())

    assert radar.firmware_version() == parse_version_reply(REPLY)
    assert radar.ser.writes == [b"stats version\n"]


def test_driver_reports_none_for_an_image_that_answers_with_counters():
    """Images without the sub-mode ignore the argument and print their stats."""
    radar = _command_radar(
        b"",
        reply=b"stats version\nframes=12 wraps=3 active=0 dump_version=7\nDone\nl3dump:/>",
    )

    assert radar.firmware_version() is None


def test_driver_raises_when_the_firmware_errors():
    radar = _command_radar(b"", reply=b"Error -1\nl3dump:/>")

    with pytest.raises(RuntimeError, match="config rejected"):
        radar.firmware_version()


def test_driver_raises_on_a_malformed_reply():
    radar = _command_radar(b"", reply=b"version=1.0.0\nDone\nl3dump:/>")

    with pytest.raises(RuntimeError, match="missing"):
        radar.firmware_version()


def test_driver_raises_when_the_reply_never_completes(monkeypatch):
    radar = _command_radar(b"", reply=b"version=1.0.0 git=a")
    monkeypatch.setattr(driver_module.time, "time", iter([0.0, 0.0, 99.0]).__next__)

    with pytest.raises(RuntimeError):
        radar.firmware_version()


# --- capture monitor startup -------------------------------------------------


def _monitor(tmp_path, radar):
    config = tmp_path / "radar.cfg"
    config.write_text("sensorStart\n", encoding="utf-8")
    return IWR6843CaptureMonitor(
        config_path=config,
        output_dir=tmp_path / "dumps",
        radar=radar,
        button_factory=FakeButton,
    )


def test_monitor_records_and_logs_the_flashed_version(tmp_path, caplog):
    radar = FakeRadar(_raw_dump())
    monitor = _monitor(tmp_path, radar)

    with caplog.at_level(logging.INFO, logger="openflight.iwr6843.monitor"):
        monitor.start()
    try:
        assert monitor.firmware_version == radar.version_reply
        assert "Firmware 1.2.3 (git abc123" in caplog.text
    finally:
        monitor.stop()


def test_monitor_warns_but_starts_on_an_unversioned_image(tmp_path, caplog):
    radar = FakeRadar(_raw_dump())
    radar.version_reply = None
    monitor = _monitor(tmp_path, radar)

    with caplog.at_level(logging.WARNING, logger="openflight.iwr6843.monitor"):
        monitor.start()
    try:
        assert monitor.firmware_version is None
        assert radar.configs, "startup must still send the config"
        assert "predates stats version" in caplog.text
    finally:
        monitor.stop()


def test_monitor_starts_when_the_version_query_fails(tmp_path, caplog):
    radar = FakeRadar(_raw_dump())
    radar.version_reply = RuntimeError("serial hiccup")
    monitor = _monitor(tmp_path, radar)

    with caplog.at_level(logging.WARNING, logger="openflight.iwr6843.monitor"):
        monitor.start()
    try:
        assert monitor.firmware_version is None
        assert radar.configs
        assert "Could not read firmware version: serial hiccup" in caplog.text
    finally:
        monitor.stop()


# --- openflight-firmware command line ----------------------------------------


def test_cli_show_prints_the_release_version(tmp_path, capsys):
    path = tmp_path / "VERSION"
    path.write_text("2.5.1\n", encoding="utf-8")

    assert cli.main(["--version-file", str(path), "show"]) == 0
    assert capsys.readouterr().out.strip() == "2.5.1"


def test_cli_bump_prints_old_and_new(tmp_path, capsys):
    path = tmp_path / "VERSION"
    path.write_text("2.5.1\n", encoding="utf-8")

    assert cli.main(["--version-file", str(path), "bump", "major"]) == 0
    assert capsys.readouterr().out.strip() == "2.5.1 -> 3.0.0"
    assert path.read_text(encoding="utf-8") == "3.0.0\n"


def test_cli_rejects_an_unknown_bump_part():
    with pytest.raises(SystemExit):
        cli.main(["bump", "build"])


@pytest.mark.parametrize("contents", [None, "banana\n"])
def test_cli_reports_a_missing_or_malformed_version_file(tmp_path, capsys, contents):
    path = tmp_path / "VERSION"
    if contents is not None:
        path.write_text(contents, encoding="utf-8")

    assert cli.main(["--version-file", str(path), "show"]) == 1
    assert "Error:" in capsys.readouterr().err


class _QueryRadar:
    reply: FirmwareVersion | None | Exception = None
    opened_with: list[str | None] = []

    def __init__(self, port=None):
        if isinstance(self.reply, OSError):
            raise self.reply
        self.opened_with.append(port)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

    def firmware_version(self):
        if isinstance(self.reply, Exception):
            raise self.reply
        return self.reply


@pytest.fixture
def query_radar(monkeypatch):
    _QueryRadar.reply = None
    _QueryRadar.opened_with = []
    monkeypatch.setattr(driver_module, "IWR6843Radar", _QueryRadar)
    return _QueryRadar


def test_cli_query_prints_the_flashed_version(query_radar, capsys):
    query_radar.reply = parse_version_reply(REPLY)

    assert cli.main(["query", "--port", "/dev/ttyUSB0"]) == 0
    assert query_radar.opened_with == ["/dev/ttyUSB0"]
    assert "Flashed firmware: 1.4.2 (git b6f4c36d2286-dirty" in capsys.readouterr().out


def test_cli_query_auto_detects_the_port(query_radar):
    assert cli.main(["query"]) == 0
    assert query_radar.opened_with == [None]


def test_cli_query_explains_an_unversioned_image(query_radar, capsys):
    assert cli.main(["query"]) == 0
    assert "predates stats version" in capsys.readouterr().out


@pytest.mark.parametrize(
    "error",
    [RuntimeError("no IWR6843 CLI found"), OSError("permission denied")],
)
def test_cli_query_fails_cleanly_without_a_board(query_radar, capsys, error):
    query_radar.reply = error

    assert cli.main(["query"]) == 1
    assert "Could not read the firmware version" in capsys.readouterr().err


def test_console_script_is_registered():
    pyproject = (REPO / "pyproject.toml").read_text(encoding="utf-8")
    assert 'openflight-firmware = "openflight.iwr6843.firmware_cli:main"' in pyproject


# --- firmware source contract ------------------------------------------------


def _function_body(source: str, signature: str) -> str:
    start = source.index(signature + "\n{")
    return source[start : source.index("\n}\n", start)]


def test_version_is_a_stats_sub_mode_not_a_new_cli_command():
    """The CLI table is at the SDK's CLI_MAX_CMD; a new entry would overflow it."""
    source = FIRMWARE_SOURCE.read_text(encoding="utf-8")
    commands = [cmd for _, cmd in re.findall(r"tableEntry\[(\d+)\]\.cmd\s*=\s*\"(\w+)\"", source)]
    stats = _function_body(source, "static int32_t l3_cli_stats(int32_t argc, char *argv[])")

    assert "version" not in commands
    assert 'argc == 2 && strcmp(argv[1], "version") == 0' in stats
    assert stats.index("l3_statsVersion()") < stats.index('CLI_write("frames=')
    assert '#include "fw_version.h"' in source


def test_stats_rejects_an_unknown_sub_mode():
    source = FIRMWARE_SOURCE.read_text(encoding="utf-8")
    stats = _function_body(source, "static int32_t l3_cli_stats(int32_t argc, char *argv[])")

    guard = stats[stats.index("if (argc != 1)") :]
    assert guard.index("Error: stats [version]") < guard.index("return -1;")


def test_firmware_version_line_carries_every_key_the_host_parses():
    source = FIRMWARE_SOURCE.read_text(encoding="utf-8")
    handler = _function_body(source, "static int32_t l3_statsVersion(void)")
    fmt = re.search(r'CLI_write\("([^"]*)"', handler).group(1)

    keys = [token.split("=", 1)[0] for token in fmt.replace("\\n", "").split()]
    assert keys == list(fw._REPLY_KEYS)  # pylint: disable=protected-access
    assert "return 0;" in handler, "the CLI prints Done only after a zero return"


def test_version_header_falls_back_for_bare_builds():
    header = VERSION_HEADER.read_text(encoding="utf-8")
    for name, fallback in [
        ("L3_FW_VERSION", "0.0.0-dev"),
        ("L3_FW_GIT", "unknown"),
        ("L3_FW_BUILT", "unknown"),
    ]:
        assert f'#ifndef {name}\n#define {name} "{fallback}"\n#endif' in header


def test_app_makefile_passes_the_stamps_as_string_literals():
    source = APP_MAKEFILE.read_text(encoding="utf-8")
    for name in ("L3_FW_VERSION", "L3_FW_GIT", "L3_FW_BUILT"):
        assert f"--define={name}='\"$({name})\"'" in source


def test_release_build_is_named_and_stamped_from_the_version_file():
    source = TOP_MAKEFILE.read_text(encoding="utf-8")
    start = source.index("\nbuild-native:")
    build = source[start : source.index("\n\n", start)]

    assert "FW_VERSION := $(strip $(shell cat VERSION 2>/dev/null))" in source
    assert "RELEASE_NAME ?= openflight_iwr6843_v$(FW_VERSION).bin" in source
    assert "build-native: check-version check-tools" in build
    assert 'L3_FW_VERSION="$(FW_VERSION)"' in build
    assert 'L3_FW_GIT="$(FW_GIT)"' in build
    assert 'L3_FW_BUILT="$(FW_BUILT)"' in build
    assert "docker-build: check-version" in source


def test_docker_build_forwards_host_stamps_into_the_container():
    source = TOP_MAKEFILE.read_text(encoding="utf-8")
    start = source.index("\ndocker-build:")
    recipe = source[start : source.index("\ndocker-shell:", start)]

    for stamp in ("FW_GIT", "FW_BUILT", "ALLOW_OVERWRITE", "RELEASE_NAME"):
        assert f'{stamp}="$({stamp})"' in recipe


# --- Makefile behaviour (needs make) -----------------------------------------

needs_make = pytest.mark.skipif(shutil.which("make") is None, reason="make not installed")


def _make(*args: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["make", "-s", "-C", str(FIRMWARE_DIR), *args],
        capture_output=True,
        text=True,
        check=False,
    )


@needs_make
def test_check_version_accepts_the_repository_version(tmp_path):
    result = _make("check-version", f"RELEASE_DIR={tmp_path}")

    assert result.returncode == 0, result.stdout + result.stderr
    version = read_release_version()
    assert f"openflight_iwr6843_v{version}.bin" in result.stdout


@needs_make
@pytest.mark.parametrize("bad", ["1.0", "v1.0.0", "1.0.0-rc1", "01.0.0"])
def test_check_version_rejects_a_malformed_version(tmp_path, bad):
    result = _make("check-version", f"RELEASE_DIR={tmp_path}", f"FW_VERSION={bad}")

    assert result.returncode != 0
    assert "must be MAJOR.MINOR.PATCH" in result.stdout


@needs_make
def test_check_version_accepts_a_fork_tag_and_names_the_release_after_it(tmp_path):
    result = _make("check-version", f"RELEASE_DIR={tmp_path}", "FW_VERSION=1.0.3.j")

    assert result.returncode == 0, result.stdout + result.stderr
    assert "openflight_iwr6843_v1.0.3.j.bin" in result.stdout


@needs_make
@pytest.mark.parametrize("bad", ["1.0.3.J", "1.0.3.j1", "1.0.3.", "1.0.3.0"])
def test_check_version_rejects_a_malformed_fork_tag(tmp_path, bad):
    result = _make("check-version", f"RELEASE_DIR={tmp_path}", f"FW_VERSION={bad}")

    assert result.returncode != 0
    assert "must be MAJOR.MINOR.PATCH" in result.stdout


@needs_make
def test_check_version_refuses_to_overwrite_a_shipped_release(tmp_path):
    version = read_release_version()
    (tmp_path / f"openflight_iwr6843_v{version}.bin").write_bytes(b"MSTR")

    refused = _make("check-version", f"RELEASE_DIR={tmp_path}")
    allowed = _make("check-version", f"RELEASE_DIR={tmp_path}", "ALLOW_OVERWRITE=1")

    assert refused.returncode != 0
    assert "already exists" in refused.stdout
    assert "bump-version" in refused.stdout
    assert allowed.returncode == 0, allowed.stdout + allowed.stderr


@needs_make
def test_bump_version_rejects_an_unknown_part():
    result = _make("bump-version", "PART=build")

    assert result.returncode != 0
    assert "PART must be major, minor or patch" in result.stdout
