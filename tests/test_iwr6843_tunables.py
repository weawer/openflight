"""The registry of firmware constants a sweep may override, and how overrides reach the C configs."""

from __future__ import annotations

from dataclasses import replace

import pytest
from iwr6843_synth import synth_shot_dump

from openflight.iwr6843 import firmware_host as fw, firmware_replay as fr, tunables as tn

needs_compiler = pytest.mark.skipif(
    fw.host_compiler() is None, reason="no C compiler for the firmware modules"
)

TEE_BIN = int(1.372 / (6.0 / 128))


def _resolve(cfg, path):
    *parents, leaf = path.split(".")
    for name in parents:
        cfg = getattr(cfg, name)
    return cfg, leaf


def test_names_are_unique_and_match_root_and_path():
    names = [t.name for t in tn.TUNABLES]
    assert len(names) == len(set(names))
    assert all(t.name == f"{t.root}.{t.path}" for t in tn.TUNABLES)
    assert set(tn.BY_NAME) == set(names)


def test_bounds_and_steps_are_sane():
    for t in tn.TUNABLES:
        assert t.low < t.high, t.name
        assert 0 < t.step <= (t.high - t.low), t.name
        assert t.kind in ("int", "float"), t.name


_STRUCT_FOR_ROOT = {
    "trig": fw.TrigCfg,
    "club": fw.TrackCfg,
    "fit": fw.ImpactFitCfg,
    "ball": fw.BallTrackCfg,
}


@pytest.mark.parametrize("tunable", tn.TUNABLES, ids=lambda t: t.name)
def test_every_tunable_is_a_field_of_its_struct(tunable):
    cfg, leaf = _resolve(_STRUCT_FOR_ROOT[tunable.root](), tunable.path)
    assert hasattr(cfg, leaf)


@needs_compiler
def test_firmware_defaults_lie_inside_the_search_bounds():
    defaults = tn.read_defaults(fr._default_library())  # pylint: disable=protected-access
    assert set(defaults) == set(tn.BY_NAME)
    for name, value in defaults.items():
        t = tn.BY_NAME[name]
        assert t.low <= value <= t.high, f"{name}: default {value} outside {t.low}..{t.high}"


def test_unknown_and_out_of_bounds_overrides_are_named():
    with pytest.raises(ValueError, match="nope.gateBins"):
        tn.check_overrides({"nope.gateBins": 1})
    with pytest.raises(ValueError, match="club.gateBins"):
        tn.check_overrides({"club.gateBins": 99.0})
    tn.check_overrides({"club.gateBins": 2.5, "ball.hyps.coastUs": 2})


def test_apply_overrides_touches_only_its_root_and_rounds_ints():
    cfg = fw.BallTrackCfg()
    tn.apply_overrides(
        {"ball.hyps.gateM": 0.05, "ball.launchPoints": 5.4, "club.gateBins": 9.0}, "ball", cfg
    )
    assert cfg.hyps.gateM == pytest.approx(0.05)
    assert cfg.launchPoints == 5
    club = fw.TrackCfg()
    tn.apply_overrides({"ball.launchPoints": 5}, "club", club)
    assert club.gateBins == 0.0  # untouched (a fresh struct)


def test_replay_config_overrides_default_to_empty_and_are_replaceable():
    config = fr.ReplayConfig(tee_bin=TEE_BIN)
    assert dict(config.overrides) == {}
    assert dict(replace(config, overrides={"club.gateBins": 2.0}).overrides) == {
        "club.gateBins": 2.0
    }


def test_replay_refuses_an_unknown_override():
    raw = synth_shot_dump(ball_speed_ms=60.0)
    with pytest.raises(ValueError, match="nope.x"):
        fr.replay_dump(raw, fr.ReplayConfig(tee_bin=TEE_BIN, overrides={"nope.x": 1}))


def _ball_track(result):
    return [(p.frame, p.range_bin) for p in result.ball_points]


@needs_compiler
def test_an_override_reaches_the_firmware_and_changes_the_track():
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=1.372)
    base = fr.ReplayConfig(tee_bin=TEE_BIN, dest_bin=TEE_BIN)
    default = fr.replay_dump(raw, base)
    altered = fr.replay_dump(raw, replace(base, overrides={"ball.originGateBins": 2.0}))
    assert default.ball_points, "the synthetic ball is tracked by default"
    assert _ball_track(altered) != _ball_track(default)


@needs_compiler
def test_empty_overrides_reproduce_the_default_replay_exactly():
    raw = synth_shot_dump(ball_speed_ms=60.0, tee_range_m=1.372)
    base = fr.ReplayConfig(tee_bin=TEE_BIN, dest_bin=TEE_BIN)
    a = fr.replay_dump(raw, base)
    b = fr.replay_dump(raw, replace(base, overrides={}))
    assert _ball_track(a) == _ball_track(b)


@needs_compiler
def test_overriding_every_constant_with_its_default_replays_the_committed_recordings_unchanged():
    """The sweep's baseline (defaults as overrides) must equal the tests' plain replay."""
    if not fr.RECORDINGS_DIR.exists():
        pytest.skip("no committed recordings")
    configs = fr.recording_configs(fr.RECORDINGS_DIR)
    if not configs:
        pytest.skip("no committed recordings")
    defaults = tn.read_defaults(fr._default_library())  # pylint: disable=protected-access
    for path, config in configs:
        raw = path.read_bytes()
        plain = fr.replay_dump(raw, config)
        explicit = fr.replay_dump(raw, replace(config, overrides={**defaults, **config.overrides}))
        for attr in ("points", "ball_points"):
            got = [(p.frame, p.range_bin) for p in getattr(explicit, attr)]
            want = [(p.frame, p.range_bin) for p in getattr(plain, attr)]
            assert got == want, f"{path.name}: {attr} differ under the explicit defaults"
