import numpy as np
import pytest

from types import SimpleNamespace

from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.common.starpilot_variables import DEFAULT_LATERAL_ACCELERATION
from openpilot.starpilot.common.starpilot_utilities import CURVE_HORIZON_SECONDS, calculate_road_curvature
from openpilot.starpilot.controls.lib.curve_speed_controller import (
  CSC_MAX_ACCEL_RATE,
  CSC_MAX_DECEL_RATE,
  CSC_MIN_SPEED,
  LAT_ACCEL_MAX,
  LAT_ACCEL_MIN,
  LAT_ACCEL_PARAM,
  PARAM_REFRESH_FRAMES,
  CurveSpeedController,
  is_manual_speed_control,
)


class FakeParams:
  def __init__(self, values=None):
    self.values = dict(values or {})

  def get_float(self, key):
    value = self.values.get(key)
    return None if value is None else float(value)


def make_controller(*, lateral_acceleration=None, weather_id=0, reduce_lat=0.0,
                     road_curvature=0.0, time_to_curve=4.0):
  params = FakeParams(
    {LAT_ACCEL_PARAM: lateral_acceleration} if lateral_acceleration is not None else {},
  )
  planner = SimpleNamespace(
    params=params,
    starpilot_weather=SimpleNamespace(weather_id=weather_id, reduce_lateral_acceleration=reduce_lat),
    road_curvature=road_curvature,
    time_to_curve=time_to_curve,
  )
  return planner, CurveSpeedController(SimpleNamespace(starpilot_planner=planner))


def make_sm(*, long_active=True, gas=False, brake=False, accel_pressed=False, override=False):
  return {
    "carControl": SimpleNamespace(longActive=long_active),
    "carState": SimpleNamespace(gasPressed=gas, brakePressed=brake),
    "starpilotCarState": SimpleNamespace(accelPressed=accel_pressed),
    "onroadEvents": [SimpleNamespace(overrideLongitudinal=override)] if override else [],
  }


def test_static_lateral_acceleration_is_loaded_and_clamped():
  _, controller = make_controller(lateral_acceleration=LAT_ACCEL_MAX + 10.0)
  assert controller.lateral_acceleration == pytest.approx(LAT_ACCEL_MAX)

  _, controller = make_controller(lateral_acceleration=LAT_ACCEL_MIN - 10.0)
  assert controller.lateral_acceleration == pytest.approx(LAT_ACCEL_MIN)

  _, controller = make_controller(lateral_acceleration=float("nan"))
  assert controller.lateral_acceleration == pytest.approx(DEFAULT_LATERAL_ACCELERATION)


def test_first_update_seeds_at_current_speed_and_curve_update_brakes():
  _, controller = make_controller(road_curvature=0.02, time_to_curve=4.0)

  controller.update_target(20.0)
  assert controller.target_set
  assert controller.target == pytest.approx(20.0)

  controller.update_target(20.0)
  curve_speed = max(np.sqrt(DEFAULT_LATERAL_ACCELERATION / 0.02), CSC_MIN_SPEED)
  expected_decel = min((20.0 - curve_speed) / 4.0, CSC_MAX_DECEL_RATE) * DT_MDL
  assert controller.target == pytest.approx(20.0 - expected_decel)


def test_target_does_not_drop_below_curve_speed_and_ramps_smoothly_on_exit():
  planner, controller = make_controller(lateral_acceleration=2.5, road_curvature=0.1, time_to_curve=0.1)
  controller.update_target(20.0)

  for _ in range(200):
    controller.update_target(20.0)

  curve_speed = max(np.sqrt(2.5 / 0.1), CSC_MIN_SPEED)
  assert controller.target == pytest.approx(curve_speed)

  # Stay at curve_speed rather than freezing at a slower current v_ego.
  controller.update_target(10.0)
  assert controller.target == pytest.approx(curve_speed)

  # When the curve opens up above target and v_ego, ramp upward smoothly instead of freezing at v_ego.
  planner.road_curvature = 0.004
  controller.target = 18.0
  controller.update_target(18.0)
  assert controller.target == pytest.approx(18.0 + CSC_MAX_ACCEL_RATE * DT_MDL)


def test_calculate_road_curvature_uses_bounded_horizon_geometric_curvature():
  model = SimpleNamespace(
    orientationRate=SimpleNamespace(z=[0.02, 0.04, 0.5], t=[1.0, 4.0, CURVE_HORIZON_SECONDS + 2.0]),
    velocity=SimpleNamespace(x=[25.0, 25.0, 25.0]),
  )
  curvature, time_to_curve = calculate_road_curvature(model, v_ego=15.0)
  assert curvature == pytest.approx(0.04 / 25.0)
  assert time_to_curve == pytest.approx(4.0)


def test_weather_reduces_the_curve_target():
  # Use a curve where both weather-adjusted speeds remain above the city-speed floor.
  _, dry = make_controller(road_curvature=0.01, time_to_curve=20.0)
  _, wet = make_controller(road_curvature=0.01, weather_id=1, reduce_lat=0.2, time_to_curve=20.0)
  dry.update_target(20.0)
  wet.update_target(20.0)
  dry.update_target(20.0)
  wet.update_target(20.0)

  assert wet.target < dry.target


def test_param_refresh_applies_live_slider_edits_without_learning():
  planner, controller = make_controller(lateral_acceleration=2.0)
  assert not controller.enable_training

  planner.params.values[LAT_ACCEL_PARAM] = 2.8
  for _ in range(PARAM_REFRESH_FRAMES - 1):
    controller.log_data(10.0, make_sm())
  assert controller.lateral_acceleration == pytest.approx(2.0)

  controller.log_data(10.0, make_sm())
  assert controller.lateral_acceleration == pytest.approx(2.8)
  assert not controller.enable_training


@pytest.mark.parametrize(
  ("long_active", "gas", "brake", "accel_pressed", "override"),
  [
    (False, False, False, False, False),
    (True, True, False, False, False),
    (True, False, True, False, False),
    (True, False, False, True, False),
    (True, False, False, False, True),
  ],
)
def test_manual_speed_control_detects_driver_ownership(long_active, gas, brake, accel_pressed, override):
  sm = make_sm(
    long_active=long_active,
    gas=gas,
    brake=brake,
    accel_pressed=accel_pressed,
    override=override,
  )
  assert is_manual_speed_control(sm)


def test_longitudinal_control_is_not_manual_when_driver_is_not_overriding():
  assert not is_manual_speed_control(make_sm())
