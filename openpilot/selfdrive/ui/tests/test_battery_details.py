"""Exercise battery validity without a display server or device hardware."""
import importlib.util
import sys
import time
from dataclasses import asdict
from pathlib import Path
from types import SimpleNamespace

import capnp
import pytest
from opendbc.car import structs


@pytest.fixture
def panel_module(monkeypatch):
  for name, module in {
    'pyray': SimpleNamespace(Rectangle=object),
    'openpilot.common.params': SimpleNamespace(Params=object),
    'openpilot.selfdrive.ui.ui_state': SimpleNamespace(ui_state=SimpleNamespace()),
    'openpilot.system.ui.lib.application': SimpleNamespace(gui_app=None, FontWeight=None),
    'openpilot.system.ui.widgets': SimpleNamespace(Widget=object),
  }.items():
    monkeypatch.setitem(sys.modules, name, module)
  path = Path(__file__).parents[1] / 'onroad/battery_details.py'
  spec = importlib.util.spec_from_file_location('battery_details_test_target', path)
  module = importlib.util.module_from_spec(spec)
  monkeypatch.setitem(sys.modules, spec.name, module)
  spec.loader.exec_module(module)
  return module


@pytest.mark.parametrize('valid', [False, True])
def test_display_requires_battery_validity(panel_module, valid):
  battery = structs.CarStateIC.BatteryDetails(valid=valid, capacity=60000, charge=30000, voltage=400, soc=50)

  class SubMaster(dict):
    def __init__(self):
      super().__init__(carStateIC=SimpleNamespace(batteryDetails=battery))
      self.recv_frame = {'carStateIC': 2}
      self.valid = {'carStateIC': True}
      self.alive = {'carStateIC': True}

  panel_module.ui_state.sm = SubMaster()
  panel_module.ui_state.started_frame = 1
  panel = panel_module.BatteryDetails.__new__(panel_module.BatteryDetails)
  panel._param_update_time = time.monotonic()
  panel._display_enabled = True
  panel._data_valid = True  # stale data from a previously valid sample
  panel._update_state()
  assert panel._data_valid is valid
  assert panel._capacity == (60000 if valid else 0)
  if not valid:
    panel._render(None)  # must return before attempting any drawing


def test_battery_validity_schema_roundtrip():
  schema_path = Path(__file__).parents[3] / 'cereal/custom.capnp'
  schema = capnp.load(str(schema_path), imports=[str(Path(structs.__file__).parent)])
  state = structs.CarStateIC.BatteryDetails(valid=True, voltage=400)
  message = schema.CarStateIC.new_message(batteryDetails=asdict(state))
  with schema.CarStateIC.from_bytes(message.to_bytes()) as decoded:
    assert decoded.batteryDetails.valid
    assert decoded.batteryDetails.voltage == 400
