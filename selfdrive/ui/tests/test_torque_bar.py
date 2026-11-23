import importlib
import sys
from types import ModuleType, SimpleNamespace

import pytest
from cereal import car, log


class _FakeSubMaster:
  def __init__(self, which_state: str, *, curvature: float = 0.0, desired_curvature: float = 0.0,
               v_ego: float = 25.0, lat_active: bool = True, roll: float = 0.0, torque: float = 0.0):
    cs = log.ControlsState.new_message()
    state_name = which_state if which_state in ("curvatureStateDEPRECATED", "angleState") else "pidState"
    cs.lateralControlState.init(state_name)
    lat_state = cs.lateralControlState

    self._data = {
      "controlsState": SimpleNamespace(
        lateralControlState=lat_state,
        curvature=curvature,
        desiredCurvature=desired_curvature,
      ),
      "carState": SimpleNamespace(vEgo=v_ego),
      "liveParameters": SimpleNamespace(roll=roll),
      "carControl": SimpleNamespace(latActive=lat_active),
      "carOutput": SimpleNamespace(actuatorsOutput=SimpleNamespace(torque=torque)),
    }

  def __getitem__(self, key):
    return self._data[key]


def _load_torque_bar(monkeypatch, *, sm, cp):
  def stub_module(name, **attrs):
    mod = ModuleType(name)
    for k, v in attrs.items():
      setattr(mod, k, v)
    monkeypatch.setitem(sys.modules, name, mod)

  class UIStatus:
    DISENGAGED = 0
    OVERRIDE = 1
    ENGAGED = 2

  class Widget:
    def __init__(self):
      pass

  stub_module("pyray", Color=lambda *a: a, Rectangle=object)
  stub_module("openpilot.selfdrive.ui.lib.starpilot_visuals", blend_colors=lambda a, b, t: a)
  stub_module("openpilot.selfdrive.ui.onroad.starpilot.rivian_lateral_mode",
              rivian_lateral_mode=SimpleNamespace(update=lambda: None, mode="none"))
  stub_module("openpilot.selfdrive.ui.ui_state",
              ui_state=SimpleNamespace(sm=sm, CP=cp, status=UIStatus.ENGAGED, always_on_lateral_active=False),
              UIStatus=UIStatus)
  stub_module("openpilot.system.ui.lib.application", gui_app=SimpleNamespace(target_fps=20))
  stub_module("openpilot.system.ui.lib.shader_polygon", draw_polygon=lambda *a, **k: None, Gradient=object)
  stub_module("openpilot.system.ui.widgets", Widget=Widget)

  mod_name = "openpilot.selfdrive.ui.onroad.starpilot.torque_bar"
  monkeypatch.delitem(sys.modules, mod_name, raising=False)
  return importlib.import_module(mod_name)


def test_torque_bar_sweeps_left_and_right_for_curvature_controller(monkeypatch):
  cp = car.CarParams.new_message(maxLateralAccel=2.5)
  sm = _FakeSubMaster("curvatureStateDEPRECATED", curvature=-0.001, desired_curvature=-0.0012, v_ego=25.0)
  mod = _load_torque_bar(monkeypatch, sm=sm, cp=cp)
  bar = mod.TorqueBar()

  for _ in range(10):
    bar._update_state()
  assert bar._torque_filter.x < -0.2

  sm._data["controlsState"].curvature = 0.001
  sm._data["controlsState"].desiredCurvature = 0.0012
  for _ in range(20):
    bar._update_state()
  assert bar._torque_filter.x > 0.2

  sm._data["carControl"].latActive = False
  for _ in range(25):
    bar._update_state()
  assert abs(bar._torque_filter.x) < 0.02


def test_torque_bar_guards_zero_max_lateral_accel_and_preserves_torque_cars(monkeypatch):
  cp_zero = car.CarParams.new_message(maxLateralAccel=0.0)
  sm = _FakeSubMaster("curvatureStateDEPRECATED", curvature=0.001, desired_curvature=0.001, v_ego=20.0)
  mod = _load_torque_bar(monkeypatch, sm=sm, cp=cp_zero)
  bar = mod.TorqueBar()
  bar._update_state()
  assert bar._torque_filter.x == pytest.approx(bar._torque_filter.x)
  assert bar._torque_filter.x > 0.0

  sm_pid = _FakeSubMaster("pidState", torque=0.4)
  mod_pid = _load_torque_bar(monkeypatch, sm=sm_pid, cp=car.CarParams.new_message(maxLateralAccel=2.5))
  bar_pid = mod_pid.TorqueBar()
  for _ in range(20):
    bar_pid._update_state()
  assert bar_pid._torque_filter.x < -0.3
