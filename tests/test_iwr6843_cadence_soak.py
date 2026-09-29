"""Pass/fail logic for the IWR6843 cadence acceptance soak."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

import pytest

_PATH = Path(__file__).parents[1] / "scripts" / "hardware-test" / "iwr6843_cadence_soak.py"
_SPEC = importlib.util.spec_from_file_location("iwr6843_cadence_soak", _PATH)
soak = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = soak
_SPEC.loader.exec_module(soak)


def test_frame_period_reads_the_frame_cfg_periodicity(tmp_path):
    cfg = tmp_path / "profile.cfg"
    cfg.write_text("dfeDataOutputMode 1\nframeCfg 0 2 12 0 3 1 0\nsensorStart\n")

    assert soak.frame_period_s(str(cfg)) == pytest.approx(0.003)


def test_frame_period_rejects_a_config_without_frame_cfg(tmp_path):
    cfg = tmp_path / "profile.cfg"
    cfg.write_text("dfeDataOutputMode 1\nsensorStart\n")

    with pytest.raises(ValueError, match="frameCfg"):
        soak.frame_period_s(str(cfg))


def test_rearm_summary_reports_share_of_the_frame_period():
    stats = {"rearm_last_us": 40, "rearm_max_us": 60, "rearm_timed": 1000}

    summary = soak.rearm_summary(stats, period_s=0.002)

    assert "rearm_max_us=60" in summary
    assert "3.0%" in summary


def test_rearm_summary_is_none_on_older_firmware():
    assert soak.rearm_summary({}, period_s=0.002) is None


def _stats(**overrides) -> dict[str, int]:
    base = {"hwa_frames": 50_000, "hwa_missed": 0, "iq8_overrun": 0, "iq8_edma_err": 0}
    base.update(overrides)
    return base


class TestScratchStaleGate:
    """scratch_stale: the detect task losing the race against the HWA reusing
    its scratch buffer -- the risk a shorter frame period carries most
    directly, since ball/club tracking silently drops that frame."""

    def test_absent_on_a_non_compact_profile_does_not_fail(self):
        ok, lines = soak.evaluate(_stats(), args_frames=50_000, rate_cap=1.0)

        assert ok
        assert any("not reported" in line for line in lines)

    def test_zero_passes(self):
        ok, _lines = soak.evaluate(_stats(scratch_stale=0), args_frames=50_000, rate_cap=1.0)

        assert ok

    def test_any_stale_frame_fails(self):
        ok, lines = soak.evaluate(_stats(scratch_stale=1), args_frames=50_000, rate_cap=1.0)

        assert not ok
        assert any(line.startswith("FAIL:") and "scratch reused" in line for line in lines)

    def test_does_not_mask_other_failures(self):
        ok, lines = soak.evaluate(
            _stats(scratch_stale=2, iq8_overrun=1), args_frames=50_000, rate_cap=1.0
        )

        assert not ok
        assert sum(line.startswith("FAIL:") for line in lines) == 2


@pytest.mark.parametrize(
    "fields",
    [
        {"enabled": 0},
        {"latched": 1},
        {"scratch_stale": None},
        {"dropped": 1},
        {"stale": 1},
        {"active": 0},
    ],
)
def test_armed_soak_requires_running_detector_and_complete_clean_counters(fields):
    stats = _stats(enabled=1, latched=0, active=1, scratch_stale=0, dropped=0, stale=0)
    stats.update(fields)
    ok, _ = soak.evaluate(stats, args_frames=50000, armed=True)
    assert not ok


def test_armed_soak_clean_state_passes():
    ok, _ = soak.evaluate(
        _stats(
            enabled=1,
            latched=0,
            active=1,
            scratch_stale=0,
            dropped=0,
            stale=0,
            hwa_rearm_err=0,
            rf_faults=0,
            freeze_to=0,
        ),
        args_frames=50000,
        armed=True,
    )
    assert ok


def test_main_arms_with_calibrated_tee_and_saves_failure_evidence(monkeypatch, tmp_path):
    import json
    from unittest.mock import Mock

    radar = Mock()
    radar.__enter__ = Mock(return_value=radar)
    radar.__exit__ = Mock(return_value=False)
    radar.cmd.return_value = "Done"
    radar.stats.return_value = (
        "hwa_frames=1000 hwa_missed=0 iq8_overrun=0 iq8_edma_err=0 "
        "enabled=1 latched=1 active=0 scratch_stale=0 dropped=0 stale=0 "
        "hwa_rearm_err=0 rf_faults=0 freeze_to=0\nDone"
    )
    monkeypatch.setattr(soak, "IWR6843Radar", Mock(return_value=radar))
    monkeypatch.setattr(soak.Calibration, "load", Mock(return_value=Mock(range_bias_m=0.075)))
    output = tmp_path / "run.jsonl"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "soak",
            "--config",
            "config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg",
            "--self-trigger-tee-m",
            "1.845",
            "--frames",
            "1000",
            "--output",
            str(output),
        ],
    )
    assert soak.main() == 1
    assert radar.cmd.call_args_list[0].args == ("triggerCfg 41 6.0 2",)
    radar.stop_sensor.assert_called_once()
    events = [json.loads(line) for line in output.read_text().splitlines()]
    assert events[0]["trigger_command"] == "triggerCfg 41 6.0 2"
    diagnostic = next(event for event in events if event["event"] == "diagnostics")
    assert "triggerLog perf" in diagnostic["logs"]
    assert events[-1]["passed"] is False


def test_diagnostic_output_cannot_turn_a_stale_frame_into_a_pass(monkeypatch, tmp_path):
    import json
    from unittest.mock import Mock

    radar = Mock()
    radar.__enter__ = Mock(return_value=radar)
    radar.__exit__ = Mock(return_value=False)
    radar.cmd.return_value = "Done"
    clean = (
        "hwa_frames=1000 hwa_missed=0 iq8_overrun=0 iq8_edma_err=0 "
        "enabled=1 latched=0 active=1 scratch_stale=0 dropped=0 stale=0 "
        "hwa_rearm_err=0 rf_faults=0 freeze_to=0\nDone"
    )
    radar.stats.side_effect = [clean, clean.replace("scratch_stale=0", "scratch_stale=9")]
    monkeypatch.setattr(soak, "IWR6843Radar", Mock(return_value=radar))
    monkeypatch.setattr(soak.time, "monotonic", Mock(side_effect=[0.0, 10.0]))
    output = tmp_path / "run.jsonl"
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "soak",
            "--config",
            "config/iwr6843_l3dump_adaptive_47f3ms_53bin_a16.cfg",
            "--self-trigger-tee-m",
            "1.845",
            "--frames",
            "1000",
            "--output",
            str(output),
        ],
    )
    assert soak.main() == 1
    events = [json.loads(line) for line in output.read_text().splitlines()]
    assert events[-1]["passed"] is False
    assert any(e["event"] == "post_diagnostics_stats" for e in events)
