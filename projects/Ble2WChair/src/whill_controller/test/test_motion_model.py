"""Tests for the measured WHILL dead-reckoning model."""

import math

import pytest

from whill_controller.motion_model import (
    BACKWARD_SPEED_MPS,
    FORWARD_SPEED_MPS,
    LEFT_TURN_RATE_RADPS,
    RIGHT_TURN_RATE_RADPS,
    DeadReckoningModel,
)


def test_measured_calibration_values():
    """Reproduce each measured displacement or angle exactly."""
    assert FORWARD_SPEED_MPS * 3.309 == pytest.approx(1.23)
    assert BACKWARD_SPEED_MPS * 3.432 == pytest.approx(1.25)
    assert math.sin(RIGHT_TURN_RATE_RADPS * 0.820) == pytest.approx(0.324675)
    assert math.sin(LEFT_TURN_RATE_RADPS * 1.096) == pytest.approx(0.4133333)


def test_forward_and_backward_integration():
    """Integrate straight motion from the startup origin."""
    model = DeadReckoningModel()
    state = model.integrate(0.25, {'w'})
    assert state.x == pytest.approx(FORWARD_SPEED_MPS * 0.25)
    assert state.y == pytest.approx(0.0)

    model.reset()
    state = model.integrate(0.25, {'s'})
    assert state.x == pytest.approx(-BACKWARD_SPEED_MPS * 0.25)
    assert state.y == pytest.approx(0.0)


def test_turn_direction_and_diagonal_mapping():
    """Use positive yaw for left turns and negative yaw for right turns."""
    model = DeadReckoningModel()
    left = model.integrate(0.1, {'a'})
    assert left.yaw > 0.0

    model.reset()
    right = model.integrate(0.1, {'d'})
    assert right.yaw < 0.0

    linear, angular = model.commanded_velocity({'z'})
    assert linear == pytest.approx(FORWARD_SPEED_MPS)
    assert angular == pytest.approx(LEFT_TURN_RATE_RADPS)


def test_disabled_tracking_stays_at_origin():
    """Ignore pointer-only commands when wheelchair tracking is disabled."""
    model = DeadReckoningModel()
    state = model.integrate(0.2, {'w', 'a'}, enabled=False)
    assert state.x == 0.0
    assert state.y == 0.0
    assert state.yaw == 0.0
