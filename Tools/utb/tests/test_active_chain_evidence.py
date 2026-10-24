# AP_FLAKE8_CLEAN
"""Offline negative checks for the active controller sequence evidence."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from test_utb_active_chain_sitl import check_released, check_sequence  # noqa: E402


def row(seq, time, mode=42):
    return dict(Seq=seq, Src=seq - 1, TimeUS=time, Md=mode)


def test_initialized_continuous_and_mode_entry():
    assert check_sequence([row(1, 1), row(2, 2501), row(1, 5001, 43), row(2, 7501, 43)]) == 2


@pytest.mark.parametrize('rows', [
    [row(2, 1)],
    [row(1, 1), row(3, 5001)],
    [row(1, 1), row(2, 2501), row(2, 5001)],
    [row(1, 1), row(2, 0)],
    [row(1, 1), row(2, 2501, 43)],
    [row(1, 1), dict(row(2, 2501), Src=0)],
])
def test_reject_missing_duplicate_reordered_or_delayed_steps(rows):
    with pytest.raises(AssertionError):
        check_sequence(rows)


def test_release_after_old_motor_slot_in_same_main_cycle():
    check_released([row(1, 2501), row(2, 5001)], 5001)


@pytest.mark.parametrize('rows', [[row(1, 5002)], [row(1, 7501)], [row(1, 5001), row(2, 5001)]])
def test_reject_motor_authority_retained_after_mode_transition(rows):
    with pytest.raises(AssertionError):
        check_released(rows, 5001)
