"""Tests for algorithms/pid_controller.py.

The I and D terms work per call (not per second): the integral is the sum of
the last `history_size` errors and the derivative is the change since the
previous call.
"""
import pytest

from algorithms.pid_controller import PIDController


def test_proportional_only():
    pid = PIDController(kp=2.0, ki=0.0, kd=0.0)
    correction, P, I, D = pid.compute(1.5)
    assert (correction, P, I, D) == (3.0, 3.0, 0.0, 0.0)


def test_zero_error_gives_zero_correction():
    pid = PIDController()
    assert pid.compute(0.0) == (0.0, 0.0, 0.0, 0.0)


def test_integral_sums_only_recent_errors():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, history_size=3, integral_limit=100)
    outputs = [pid.compute(e)[2] for e in [1, 1, 1, 1, -1]]
    # window of 3: 1, 1+1, 1+1+1, then the oldest error drops out
    assert outputs == [1, 2, 3, 3, 1]


def test_integral_is_clamped():
    pid = PIDController(kp=0.0, ki=1.0, kd=0.0, history_size=10, integral_limit=5)
    for _ in range(10):
        _, _, I, _ = pid.compute(2.0)
    assert pid.integral == 5
    assert I == 5
    for _ in range(10):
        pid.compute(-2.0)
    assert pid.integral == -5


def test_derivative_reacts_to_change():
    pid = PIDController(kp=0.0, ki=0.0, kd=3.0)
    assert pid.compute(1.0)[3] == 3.0  # from 0 to 1
    assert pid.compute(1.0)[3] == 0.0  # no change
    assert pid.compute(-1.0)[3] == -6.0  # from 1 to -1


@pytest.mark.parametrize("error, expected", [(10.0, 25), (-10.0, -25)])
def test_output_is_clamped_but_terms_are_not(error, expected):
    pid = PIDController(kp=7.5, ki=0.0, kd=0.0, max_output=25)
    correction, P, _, _ = pid.compute(error)
    assert correction == expected
    assert P == 7.5 * error


def test_sign_follows_error():
    left = PIDController().compute(-2.0)[0]
    right = PIDController().compute(2.0)[0]
    assert left < 0 < right
    assert left == -right


def test_reset_clears_memory():
    pid = PIDController(kp=1.0, ki=1.0, kd=1.0)
    for e in [2, 2, 2]:
        pid.compute(e)
    pid.reset()
    assert pid.error_history == [] and pid.integral == 0.0 and pid.previous_error == 0.0
    assert pid.compute(1.0) == PIDController(kp=1.0, ki=1.0, kd=1.0).compute(1.0)


def test_default_gains_match_line_follower():
    # infinity_circuit.py uses Kp=7.5, Ki=2.5, Kd=2.5
    pid = PIDController()
    assert (pid.Kp, pid.Ki, pid.Kd) == (7.5, 2.5, 2.5)
    # first call with a strong deviation: P=15, I=5, D=5 -> clamped to 25
    assert pid.compute(2.0) == (25, 15.0, 5.0, 5.0)
