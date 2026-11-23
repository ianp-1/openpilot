from types import SimpleNamespace

import pytest
from cereal import log

from openpilot.common.realtime import DT_MDL
from openpilot.starpilot.controls.lib.starpilot_following import (
  LANE_CHANGE_GAP_RAMP_IN_RATE,
  LANE_CHANGE_GAP_RAMP_OUT_ABORT_RATE,
  LANE_CHANGE_GAP_RAMP_OUT_RATE,
  StarPilotFollowing,
)


def _make_following(*, d_rel=42.0, v_lead=27.0, a_lead=0.0):
  planner = SimpleNamespace(
    starpilot_weather=SimpleNamespace(weather_id=0, increase_following_distance=0.0),
    tracking_lead=True,
    lead_one=SimpleNamespace(status=True, dRel=d_rel, vLead=v_lead, aLeadK=a_lead),
  )
  return StarPilotFollowing(planner)


def _make_sm(lc_state=log.LaneChangeState.off, lc_dir=log.LaneChangeDirection.left,
             left_bs=False, right_bs=False):
  return {
    "selfdriveState": SimpleNamespace(personality=0),
    "starpilotCarState": SimpleNamespace(trafficModeEnabled=False),
    "carState": SimpleNamespace(aEgo=0.0, standstill=False, leftBlindspot=left_bs, rightBlindspot=right_bs),
    "modelV2": SimpleNamespace(meta=SimpleNamespace(laneChangeState=lc_state, laneChangeDirection=lc_dir)),
  }


def _make_toggles():
  return SimpleNamespace(
    aggressive_follow=1.05,
    standard_follow=1.45,
    relaxed_follow=1.75,
    aggressive_jerk_acceleration=1.0,
    aggressive_jerk_danger=1.0,
    aggressive_jerk_speed=1.0,
    aggressive_jerk_deceleration=1.0,
    aggressive_jerk_speed_decrease=1.0,
    standard_jerk_acceleration=1.0,
    standard_jerk_danger=1.0,
    standard_jerk_speed=1.0,
    standard_jerk_deceleration=1.0,
    standard_jerk_speed_decrease=1.0,
    relaxed_jerk_acceleration=1.0,
    relaxed_jerk_danger=1.0,
    relaxed_jerk_speed=1.0,
    relaxed_jerk_deceleration=1.0,
    relaxed_jerk_speed_decrease=1.0,
    custom_personalities=False,
    conditional_slower_lead=False,
    lane_change_close_gap=True,
    lane_change_close_gap_seconds=0.6,
    minimum_lane_change_speed=15.0,
  )


def test_lane_change_gap_suppresses_far_lead_coast_and_ramps_smoothly_on_exit():
  toggles = _make_toggles()
  following = _make_following(d_rel=36.0, v_lead=27.0)
  for _ in range(30):
    following.update(True, 28.0, _make_sm(log.LaneChangeState.laneChangeStarting), toggles)

  assert following.lane_change_gap_active is True
  assert following.disable_throttle is False
  assert following.t_follow == pytest.approx(0.6)

  following.update(True, 28.0, _make_sm(log.LaneChangeState.off), toggles)
  assert following.lane_change_gap_active is False
  assert following.t_follow == pytest.approx(0.6 + LANE_CHANGE_GAP_RAMP_OUT_RATE * DT_MDL)


def test_lane_change_gap_snaps_back_quickly_on_blindspot_or_hard_lead_brake():
  toggles = _make_toggles()
  following = _make_following(d_rel=42.0, v_lead=27.0, a_lead=0.0)
  for _ in range(30):
    following.update(True, 28.0, _make_sm(log.LaneChangeState.laneChangeStarting), toggles)
  assert following.t_follow == pytest.approx(0.6)

  following.update(True, 28.0, _make_sm(log.LaneChangeState.laneChangeStarting, left_bs=True), toggles)
  assert following.lane_change_gap_active is False
  assert following.t_follow == pytest.approx(0.6 + LANE_CHANGE_GAP_RAMP_OUT_ABORT_RATE * DT_MDL)
