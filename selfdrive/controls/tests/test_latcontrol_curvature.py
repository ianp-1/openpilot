from types import SimpleNamespace

from cereal import car, custom, log
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_CURVATURE
from openpilot.selfdrive.controls.lib.latcontrol_curvature import LatControlCurvature


def build_controller():
  cp = CarInterface.get_non_essential_params(CAR.VOLKSWAGEN_ID4_MK1)
  ci = CarInterface(cp, custom.StarPilotCarParams.new_message())
  controller = LatControlCurvature(cp.as_reader(), ci, DT_CTRL)

  cs = car.CarState.new_message()
  cs.vEgo = 15.0
  cs.steeringAngleDeg = 0.0

  params = log.LiveParametersData.new_message()
  params.steerRatio = cp.steerRatio
  params.stiffnessFactor = 1.0
  params.roll = 0.0
  params.angleOffsetDeg = 0.0
  return controller, cs, VehicleModel(cp), params


def test_curvature_controller_output_and_reset():
  controller, cs, vm, params = build_controller()
  toggles = SimpleNamespace()
  assert controller.pid.i_dt == DT_CTRL

  _, output, state = controller.update(True, cs, vm, params, False, 0.01, False, 0.2, None, None, toggles)
  assert state.active
  assert 0.0 < output <= MAX_CURVATURE

  _, output, state = controller.update(False, cs, vm, params, False, 0.01, False, 0.2, None, None, toggles)
  assert not state.active
  assert output == 0.0


def test_curvature_controller_uses_feedforward_during_driver_override():
  controller, cs, vm, params = build_controller()
  cs.steeringPressed = True
  desired_curvature = -0.015

  _, output, state = controller.update(True, cs, vm, params, False, desired_curvature, False, 0.2,
                                       None, None, SimpleNamespace())
  assert state.active
  assert output == desired_curvature


def test_curvature_controller_bleeds_integrator_below_pid_cutin():
  controller, cs, vm, params = build_controller()
  toggles = SimpleNamespace()
  cs.vEgo = 20.0
  for _ in range(100):
    controller.update(True, cs, vm, params, False, 0.01, False, 0.2, None, None, toggles)
  i_initial = abs(controller.pid.i)
  assert i_initial > 1e-4

  cs.vEgo = 5.0
  for _ in range(50):
    controller.update(True, cs, vm, params, False, 0.01, False, 0.2, None, None, toggles)
  i_half_sec = abs(controller.pid.i)
  assert 0.2 * i_initial < i_half_sec < 0.6 * i_initial

  for _ in range(100):
    controller.update(True, cs, vm, params, False, 0.01, False, 0.2, None, None, toggles)
  assert abs(controller.pid.i) < 0.06 * i_initial


def test_curvature_controller_saturation_scales_at_low_speed():
  controller, cs, vm, params = build_controller()
  toggles = SimpleNamespace()

  # 2e-3 1/m lag at 5.8 m/s (0.067 m/s^2) must NOT trip saturation.
  cs.vEgo = 5.8
  for _ in range(100):
    _, _, state = controller.update(True, cs, vm, params, False, 2e-3, False, 0.2, None, None, toggles)
  assert not state.saturated
  assert controller.sat_time == 0.0

  # >= 0.30 m/s^2 lag at low speed DOES trip saturation.
  controller.reset()
  high_low_speed_curv = 0.31 / (5.8 ** 2)
  for _ in range(100):
    _, _, state = controller.update(True, cs, vm, params, False, high_low_speed_curv, False, 0.2, None, None, toggles)
  assert state.saturated

  # Highway speed (>= 17.3 m/s) keeps the 1e-3 1/m floor.
  controller.reset()
  cs.vEgo = 20.0
  for _ in range(100):
    _, _, state = controller.update(True, cs, vm, params, False, 0.9e-3, False, 0.2, None, None, toggles)
  assert not state.saturated

  controller.reset()
  for _ in range(100):
    _, _, state = controller.update(True, cs, vm, params, False, 1.2e-3, False, 0.2, None, None, toggles)
  assert state.saturated
