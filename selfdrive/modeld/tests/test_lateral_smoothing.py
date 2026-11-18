from types import SimpleNamespace

from cereal import car, log
import pytest

from openpilot.selfdrive.modeld.modeld import (
  get_car_lateral_smooth_seconds,
  get_lateral_smooth_seconds,
  sync_inactive_lateral_prev_action,
)


@pytest.mark.parametrize(("v_ego", "expected"), [
  (0.0, 0.4),
  (2.0, 0.4),
  (5.0, 0.2),
  (8.0, 0.0),
  (30.0, 0.0),
])
def test_lateral_smoothing_tapers_with_speed(v_ego, expected):
  assert get_lateral_smooth_seconds(v_ego, 0.4) == pytest.approx(expected)


@pytest.mark.parametrize("v_ego", [0.0, 5.0, 30.0])
def test_default_lateral_smoothing_is_disabled(v_ego):
  assert get_lateral_smooth_seconds(v_ego) == 0.0


@pytest.mark.parametrize("v_ego", [0.0, 5.0, 30.0])
def test_non_rivian_cars_keep_configured_starpilot_smoothing(v_ego):
  assert get_car_lateral_smooth_seconds("toyota", v_ego, 0.4) == 0.4


@pytest.mark.parametrize(("v_ego", "expected"), [
  (0.0, 0.4),
  (2.0, 0.4),
  (5.0, 0.2),
  (8.0, 0.0),
  (30.0, 0.0),
])
def test_subaru_uses_low_speed_configured_smoothing(v_ego, expected):
  assert get_car_lateral_smooth_seconds("subaru", v_ego, 0.4) == pytest.approx(expected)


@pytest.mark.parametrize(("v_ego", "maximum", "expected"), [
  (0.0, 0.4, 0.4),
  (5.0, 0.4, 0.2),
  (30.0, 0.4, 0.0),
  (0.0, 0.0, 0.0),
])
def test_rivian_uses_configured_smoothing(v_ego, maximum, expected):
  assert get_car_lateral_smooth_seconds("rivian", v_ego, maximum) == pytest.approx(expected)


def test_sync_inactive_lateral_prev_action_resets_curvature_when_lat_inactive():
  prev_action = log.ModelDataV2.Action(desiredCurvature=0.015, desiredAcceleration=-0.85, shouldStop=True)
  car_control = car.CarControl.new_message(latActive=False, currentCurvature=-0.004)

  synced = sync_inactive_lateral_prev_action(prev_action, car_control, car_control_alive=True)

  assert synced.desiredCurvature == pytest.approx(-0.004)
  assert synced.desiredAcceleration == pytest.approx(-0.85)
  assert synced.shouldStop is True


def test_sync_inactive_lateral_prev_action_preserves_prev_action_when_lat_active():
  prev_action = log.ModelDataV2.Action(desiredCurvature=0.015, desiredAcceleration=-0.85, shouldStop=False)
  car_control = car.CarControl.new_message(latActive=True, currentCurvature=-0.004)

  synced = sync_inactive_lateral_prev_action(prev_action, car_control, car_control_alive=True)

  assert synced is prev_action
  assert synced.desiredCurvature == pytest.approx(0.015)
  assert synced.desiredAcceleration == pytest.approx(-0.85)
  assert synced.shouldStop is False


@pytest.mark.parametrize(("car_control", "alive"), [
  (SimpleNamespace(latActive=False, currentCurvature=0.0), False),
  (SimpleNamespace(latActive=False, currentCurvature=float("nan")), True),
  (SimpleNamespace(latActive=False, currentCurvature=float("inf")), True),
  (None, True),
])
def test_sync_inactive_lateral_prev_action_guards_stale_or_nonfinite(car_control, alive):
  prev_action = log.ModelDataV2.Action(desiredCurvature=0.015, desiredAcceleration=0.4, shouldStop=False)

  synced = sync_inactive_lateral_prev_action(prev_action, car_control, car_control_alive=alive)

  assert synced is prev_action
  assert synced.desiredCurvature == pytest.approx(0.015)
  assert synced.desiredAcceleration == pytest.approx(0.4)
