"""Host C compiler discovery for the firmware modules."""

from __future__ import annotations

import sys

from openflight.iwr6843 import firmware_host as fw


def _no_path_compilers(monkeypatch):
    monkeypatch.setattr(fw.shutil, "which", lambda _name: None)


def test_path_compiler_wins_over_zig(monkeypatch):
    monkeypatch.setattr(
        fw.shutil, "which", lambda name: f"/usr/bin/{name}" if name == "gcc" else None
    )
    monkeypatch.setattr(fw, "_ziglang_available", lambda: True)
    assert fw.host_compiler_command() == ["/usr/bin/gcc"]
    assert fw.host_compiler() == "/usr/bin/gcc"


def test_zig_is_the_fallback_without_a_path_compiler(monkeypatch):
    _no_path_compilers(monkeypatch)
    monkeypatch.setattr(fw, "_ziglang_available", lambda: True)
    assert fw.host_compiler_command() == [sys.executable, "-m", "ziglang", "cc"]
    assert fw.host_compiler() is not None


def test_no_compiler_at_all(monkeypatch):
    _no_path_compilers(monkeypatch)
    monkeypatch.setattr(fw, "_ziglang_available", lambda: False)
    assert fw.host_compiler_command() is None
    assert fw.host_compiler() is None
