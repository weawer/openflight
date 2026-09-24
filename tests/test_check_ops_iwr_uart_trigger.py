"""Tests for the OPS-to-IWR UART timing diagnostic."""

import importlib.util
from pathlib import Path

import pytest

SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "hardware-test"
    / "check_ops_iwr_uart_trigger.py"
)
spec = importlib.util.spec_from_file_location("check_ops_iwr_uart_trigger", SCRIPT)
diagnostic = importlib.util.module_from_spec(spec)
spec.loader.exec_module(diagnostic)


@pytest.mark.parametrize(
    ("segments", "delay_ms"),
    [(6, 110.933333), (20, 51.2), (22, 42.666667)],
)
def test_estimated_trigger_timestamp_uses_ops_post_trigger_duration(segments, delay_ms):
    first_byte = 1000.0
    result = diagnostic.estimated_trigger_timestamp(first_byte, segments)
    assert (first_byte - result) * 1000 == pytest.approx(delay_ms)
