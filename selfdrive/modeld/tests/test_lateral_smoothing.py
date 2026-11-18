from types import SimpleNamespace

from cereal import car, log
from opendbc.car.volkswagen.values import VolkswagenFlags
import pytest

from openpilot.starpilot.common.lateral_delay import VW_MEB_FLAG, get_effective_lateral_delay

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


@pytest.mark.parametrize(("v_ego", "vehicle_smooth", "expected"), [
  (2.0, 0.22, 0.22),
  (6.7, 0.22, 0.22),
  (8.95, 0.22, 0.16),
  (11.2, 0.22, 0.10),
  (30.0, 0.22, 0.10),
  (6.7, 0.0, 0.10),
])
def test_volkswagen_meb_lateral_smoothing_tapers_above_low_speed(v_ego, vehicle_smooth, expected):
  assert get_car_lateral_smooth_seconds("volkswagen", v_ego, 0.10, vehicle_smooth) == pytest.approx(expected)


def test_effective_lateral_delay_schedules_vw_meb_low_speed_only():
  assert int(VolkswagenFlags.MEB) == VW_MEB_FLAG
  meb_cp = SimpleNamespace(brand="volkswagen", flags=int(VolkswagenFlags.MEB))
  mqb_cp = SimpleNamespace(brand="volkswagen", flags=0)
  honda_cp = SimpleNamespace(brand="honda", flags=0)

  assert get_effective_lateral_delay(meb_cp, 3.0, 0.40) == pytest.approx(0.22)
  assert get_effective_lateral_delay(meb_cp, 6.7, 0.40) == pytest.approx(0.22)
  assert get_effective_lateral_delay(meb_cp, 11.0, 0.40) == pytest.approx(0.31)
  assert get_effective_lateral_delay(meb_cp, 15.0, 0.40) == pytest.approx(0.40)
  assert get_effective_lateral_delay(meb_cp, 20.0, 0.40) == pytest.approx(0.40)
  assert get_effective_lateral_delay(meb_cp, 3.0, 0.40, use_custom_delay=True) == pytest.approx(0.40)
  assert get_effective_lateral_delay(mqb_cp, 3.0, 0.30) == pytest.approx(0.30)
  assert get_effective_lateral_delay(honda_cp, 3.0, 0.30) == pytest.approx(0.30)
