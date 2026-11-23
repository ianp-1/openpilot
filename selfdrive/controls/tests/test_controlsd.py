from types import SimpleNamespace

import cereal.messaging as messaging
from cereal import car, log

from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.controlsd import (
  LANE_CHANGE_CURVE_BYPASS_LAT_ACCEL,
  LANE_CHANGE_SMOOTH_RELEASE_T,
  Controls,
)
from openpilot.selfdrive.controls.lib.drive_helpers import MAX_LATERAL_JERK
from openpilot.selfdrive.controls.lib.lane_centering import LaneCenteringController

STEER_ANGLE_SATURATION_THRESHOLD = 2.5  # matches latcontrol_angle.py


class FakeSubMaster:
  """Minimal SubMaster stand-in covering the topics Controls tests read."""

  def __init__(self):
    self._structs = {}
    self.valid = {}
    self.updated = {}
    self.logMonoTime = {}

  def add(self, topic, valid=True, log_mono_time=0, size=None, updated=False):
    msg = messaging.new_message(topic, size=size)
    struct = getattr(msg, topic)
    self._structs[topic] = struct
    self.valid[topic] = valid
    self.updated[topic] = updated
    self.logMonoTime[topic] = log_mono_time
    return struct

  def all_checks(self, service_list=()):
    return all(self.valid.get(topic, False) for topic in service_list)

  def __getitem__(self, topic):
    return self._structs[topic]


class FakePubMaster:
  def send(self, *args, **kwargs):
    pass


def _build_controls():
  """A Controls instance wired up just enough for publish() to run end to end,
  matching the __new__-then-set-attributes pattern test_nissan_leaf_fallback.py
  already uses for this same heavy class."""
  controls = Controls.__new__(Controls)

  controls.CP = car.CarParams.new_message(steerControlType=car.CarParams.SteerControlType.angle)
  controls.pm = FakePubMaster()
  controls.LoC = SimpleNamespace(long_control_state=car.CarControl.Actuators.LongControlState.off,
                                 pid=SimpleNamespace(p=0.0, i=0.0, f=0.0))
  controls.LaC = SimpleNamespace()  # no starpilot_lateral_state -> skips that debug send
  controls.starpilot_toggles = SimpleNamespace()
  controls.curvature = 0.0
  controls.desired_curvature = 0.0
  controls.calibrated_pose = None
  controls.steer_limited_by_safety = False

  sm = FakeSubMaster()
  sm.add('carState')
  sm.add('longitudinalPlan')
  sm.add('starpilotCarState')
  sm.add('selfdriveState')
  sm.add('driverAssistance', valid=False)
  sm.add('carOutput')
  sm.add('driverMonitoringState')
  sm.logMonoTime['modelV2'] = 0
  controls.sm = sm

  cc = car.CarControl.new_message()
  lac_log = log.ControlsState.LateralAngleState.new_message()
  return controls, cc, lac_log


def _set_steering(cc, sm, *, requested_deg, applied_deg):
  cc.actuators.steeringAngleDeg = requested_deg
  sm['carOutput'].actuatorsOutput.steeringAngleDeg = applied_deg


def test_aol_refreshes_steer_limited_by_safety():
  """
  Regression for AOL stale steer_limited_by_safety state.

  1. Begin with normal OP active and a requested/output mismatch, causing
     steer_limited_by_safety=True.
  2. Drop selfdriveState.active=False while keeping CC.latActive=True via AOL.
  3. Make requested and applied torque match.
  4. Verify steer_limited_by_safety clears instead of retaining the old True.
  """
  controls, cc, lac_log = _build_controls()

  cc.enabled = True
  cc.longActive = True
  cc.latActive = True
  controls.sm['selfdriveState'].active = True
  _set_steering(cc, controls.sm, requested_deg=10.0, applied_deg=0.0)
  controls.publish(cc, lac_log)
  assert controls.steer_limited_by_safety

  # AOL keeps latActive true while full self-drive is disengaged.
  controls.sm['selfdriveState'].active = False
  cc.enabled = False
  cc.longActive = False
  cc.latActive = True
  _set_steering(cc, controls.sm, requested_deg=10.0, applied_deg=10.0)
  controls.publish(cc, lac_log)
  assert not controls.steer_limited_by_safety


def test_lateral_inactive_clears_steer_limited_by_safety():
  """A previous limiting condition must not survive a latActive=False transition."""
  controls, cc, lac_log = _build_controls()

  cc.enabled = True
  cc.longActive = True
  cc.latActive = True
  controls.sm['selfdriveState'].active = True
  _set_steering(cc, controls.sm, requested_deg=10.0, applied_deg=0.0)
  controls.publish(cc, lac_log)
  assert controls.steer_limited_by_safety

  cc.enabled = False
  cc.longActive = False
  cc.latActive = False
  controls.publish(cc, lac_log)
  assert not controls.steer_limited_by_safety


def _build_state_control_controls():
  controls = Controls.__new__(Controls)
  cp = car.CarParams.new_message(
    steerControlType=car.CarParams.SteerControlType.curvatureDEPRECATED,
    brand="volkswagen",
    steerRatio=14.0,
    steerActuatorDelay=0.2,
  )
  cp.lateralTuning.init('pid')
  controls.CP = cp.as_reader()
  controls.pm = FakePubMaster()
  controls.CI = SimpleNamespace(get_pid_accel_limits=lambda *args: (-3.5, 2.0))
  controls.LoC = SimpleNamespace(
    long_control_state=car.CarControl.Actuators.LongControlState.off,
    reset=lambda: None,
    update=lambda *args, **kwargs: 0.0,
    pid=SimpleNamespace(p=0.0, i=0.0, f=0.0),
  )
  controls.LaC = SimpleNamespace(
    update=lambda active, cs, vm, lp, steer_limited, desired_curvature, *args: (
      0.0, float(desired_curvature), log.ControlsState.LateralCurvatureState.new_message(),
    ),
  )
  controls.VM = SimpleNamespace(update_params=lambda x, sr: None, calc_curvature=lambda angle, v, roll: 0.0)
  controls.rack_map = None
  controls.lane_centering = LaneCenteringController()
  controls.starpilot_toggles = SimpleNamespace(
    lane_change_pace=5,
    lane_change_jerk_factor=0.2,
    minimum_lane_change_speed=10.0,
    max_desired_acceleration=2.0,
    lane_centering=False,
    lane_center_offset=0.0,
    lane_centering_e2e_authority=0.0,
    lane_centering_pause_on_signal=False,
  )
  controls.curvature = 0.0
  controls.desired_curvature = 0.0
  controls.model_curvature_held = 0.0
  controls.model_curvature_from = 0.0
  controls.model_curvature_target = 0.0
  controls.model_curvature_elapsed = 0.0
  controls.model_action_interp = False
  controls.lc_smooth_release = 0.0
  controls.lc_entry_sign = 0.0
  controls.lc_arrest_jerk_factor = 1.0
  controls.turn_hold_curvature = 0.0
  controls.turn_hold_standstill_t = 0.0
  controls.turn_hold_swept = 0.0
  controls.turn_hold_handoff_t = 0.0
  controls.turn_hold_done = False
  controls.turn_blinker_swept = 0.0
  controls.turn_floor = 0.0
  controls.turn_release = 0.0
  controls.turn_override_out = 0.0
  controls.turn_hold_opposed_at_stop = False
  controls.turn_shaping = False
  controls.twitch_guard_remaining = 0.0
  controls.kona_non_scc_lateral_active = False
  controls.kona_non_scc_lateral_faulted = False
  controls.elantra_hev_2024_lateral_faulted = False
  controls.elantra_hev_2024_previous_cruise_enabled = False
  controls.calibrated_pose = None
  controls.lat_delay_schedule = None
  controls.steer_limited_by_safety = False
  controls.learn_stiffness = False
  controls.learn_steer_ratio = False
  controls.learn_angle_offset = False
  controls.ecu_disable_failed = False
  controls.ecu_disable_failed_checked = True

  sm = FakeSubMaster()
  sm.add('carState')
  lp = sm.add('liveParameters')
  lp.stiffnessFactor = 1.0
  lp.steerRatio = 14.0
  sm.add('longitudinalPlan')
  sm.add('modelV2')
  sm.add('lateralManeuverPlan', valid=False)
  sds = sm.add('selfdriveState')
  sds.enabled = True
  sds.active = True
  sm.add('starpilotCarState')
  sm.add('onroadEvents', size=0)
  sp_plan = sm.add('starpilotPlan')
  sp_plan.lateralCheck = True
  sm.add('radarState')
  sm.add('carOutput')
  live_delay = sm.add('liveDelay')
  live_delay.lateralDelay = 0.2
  sm.add('starpilotModelV2', valid=False)
  controls.sm = sm
  return controls


def test_lane_change_curve_bypass_releases_jerk_limit_on_onramp():
  """Normal highway lane changes (<= 1.2 m/s^2) stay jerk-shaped, while signaled
  on-ramp curves (> 1.4 m/s^2) immediately release jerk_factor = 1.0."""
  controls = _build_state_control_controls()
  cs = controls.sm['carState']
  model_v2 = controls.sm['modelV2']

  # Normal highway lane change at 25 m/s with 1.1875 m/s^2 (<= 1.2 m/s^2) remains jerk-shaped.
  cs.vEgo = 25.0
  model_v2.meta.laneChangeState = log.LaneChangeState.laneChangeStarting
  model_v2.action.desiredCurvature = 0.0019  # 0.0019 * 25^2 = 1.1875 <= 1.2 m/s^2
  controls.state_control()

  shaped_step = (MAX_LATERAL_JERK * controls.starpilot_toggles.lane_change_jerk_factor / (25.0 ** 2)) * DT_CTRL
  assert controls.lc_smooth_release == LANE_CHANGE_SMOOTH_RELEASE_T
  assert controls.lc_entry_sign == 1.0
  assert abs(controls.lc_arrest_jerk_factor - 0.2) < 1e-9
  assert abs(controls.desired_curvature - shaped_step) < 1e-9

  # Release tail after laneChangeState clears stays jerk-shaped for normal highway maneuvers.
  prev_curv = controls.desired_curvature
  model_v2.meta.laneChangeState = log.LaneChangeState.off
  controls.state_control()
  assert 0.0 < controls.lc_smooth_release < LANE_CHANGE_SMOOTH_RELEASE_T
  assert controls.lc_arrest_jerk_factor < 0.25
  assert (controls.desired_curvature - prev_curv) < 0.25 * (MAX_LATERAL_JERK / (25.0 ** 2)) * DT_CTRL

  # Signaled on-ramp curve (> 1.4 m/s^2) clears the LC hold and ramps jerk_factor smoothly via rise filter.
  assert LANE_CHANGE_CURVE_BYPASS_LAT_ACCEL == 1.4
  prev_curv = controls.desired_curvature
  prev_jf = controls.lc_arrest_jerk_factor
  cs.vEgo = 20.0
  model_v2.meta.laneChangeState = log.LaneChangeState.laneChangeStarting
  model_v2.action.desiredCurvature = 0.0040  # 0.0040 * 20^2 = 1.6 > 1.4 m/s^2
  controls.state_control()

  assert controls.lc_smooth_release == 0.0
  assert controls.lc_entry_sign == 0.0
  assert prev_jf < controls.lc_arrest_jerk_factor < 0.35
  ramped_step = (MAX_LATERAL_JERK * controls.lc_arrest_jerk_factor / (20.0 ** 2)) * DT_CTRL
  assert abs((controls.desired_curvature - prev_curv) - ramped_step) < 1e-9

  for _ in range(40):
    controls.state_control()
  assert controls.lc_arrest_jerk_factor > 0.85

  # Verify abs(curvature) symmetry for a left-curving on-ramp (> 1.4 m/s^2).
  controls.desired_curvature = 0.0
  controls.lc_smooth_release = LANE_CHANGE_SMOOTH_RELEASE_T
  controls.lc_entry_sign = -1.0
  controls.lc_arrest_jerk_factor = 0.2
  model_v2.action.desiredCurvature = -0.0040
  controls.state_control()

  assert controls.lc_smooth_release == 0.0
  assert controls.lc_entry_sign == 0.0
  assert 0.2 < controls.lc_arrest_jerk_factor < 0.35
  left_step = (MAX_LATERAL_JERK * controls.lc_arrest_jerk_factor / (20.0 ** 2)) * DT_CTRL
  assert abs(controls.desired_curvature - (-left_step)) < 1e-9

  # Clearing curve_bypass mid-ramp keeps ramping smoothly toward 1.0 instead of dropping.
  jf_mid = controls.lc_arrest_jerk_factor
  model_v2.meta.laneChangeState = log.LaneChangeState.off
  model_v2.action.desiredCurvature = -0.0010
  controls.state_control()
  assert controls.lc_arrest_jerk_factor > jf_mid
