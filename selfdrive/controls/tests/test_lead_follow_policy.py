from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib.lead_follow_policy import (
  FOLLOW_GAP_BUFFER_FADE,
  FOLLOW_GAP_BUFFER_GAIN,
  FOLLOW_GAP_BUFFER_MIN,
  _catchup_cap,
  apply,
)
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import desired_follow_distance


def lead(*, d_rel=40.0, v_lead=20.0, a_lead=0.0, radar=False, model_prob=0.99, y_rel=0.0):
  return SimpleNamespace(
    status=True,
    dRel=d_rel,
    vLead=v_lead,
    aLeadK=a_lead,
    radar=radar,
    modelProb=model_prob,
    yRel=y_rel,
  )


def run(lead_one, *, lead_two=None, source="lead0", active=True, v_ego=20.0,
        previous=0.0, raw=0.0, post_departure=False, blocked=False, panic=False):
  return apply(
    lead_one,
    lead_two or SimpleNamespace(status=False),
    source=source,
    active=active,
    v_ego=v_ego,
    t_follow=1.45,
    previous_target=previous,
    raw_target=raw,
    tracking=active,
    post_departure=post_departure,
    blocked=blocked,
    panic_bypass=panic,
  )


def test_follow_policy_never_reselects_or_fuses_leads():
  lead_one = lead(d_rel=80.0, v_lead=20.0)
  lead_two = lead(d_rel=35.0, v_lead=19.5)

  assert run(lead_one, lead_two=lead_two, source="lead0").lead is lead_one
  assert run(lead_one, lead_two=lead_two, source="lead1").lead is lead_two

  inactive = run(lead_one, lead_two=lead_two, source="lead1", active=False, raw=0.3)
  assert inactive.lead is None
  assert inactive.target == pytest.approx(0.3)


@pytest.mark.parametrize("blocked", [True, False])
def test_follow_policy_keeps_stop_and_panic_outputs_outside_comfort_path(blocked):
  result = run(lead(d_rel=8.0, v_lead=0.0), raw=-1.2, blocked=blocked, panic=not blocked)
  assert result.lead is None
  assert result.target == pytest.approx(-1.2)


def test_follow_policy_limits_small_post_lead_reversal():
  result = run(lead(d_rel=36.0, v_lead=20.2), v_ego=20.0, previous=-0.08, raw=0.45)
  assert result.target < 0.45
  assert result.target - (-0.08) <= 0.18


def test_follow_policy_deadbands_small_steady_sign_reversal():
  result = run(lead(d_rel=37.2, v_lead=24.0, radar=True), v_ego=24.0, previous=0.24, raw=-0.24)

  assert result.target == pytest.approx(0.0)


def test_follow_policy_deadband_does_not_mask_closing_lead():
  result = run(lead(d_rel=37.2, v_lead=22.5, a_lead=-1.0, radar=True), v_ego=24.0, previous=0.24, raw=-0.24)

  assert result.target < 0.0


def test_follow_policy_never_relaxes_material_braking():
  result = run(lead(d_rel=25.0, v_lead=18.0), v_ego=25.0, previous=0.30, raw=-1.2)
  assert result.target == pytest.approx(-1.2)


def test_follow_policy_leaves_low_speed_vision_departure_uncapped():
  # This is the range the old vision-only cap affected; below 4.5 m/s the
  # longitudinal planner still has its independent weak-lead safety cap.
  result = run(lead(d_rel=12.0, v_lead=6.0), v_ego=5.0, raw=1.4)
  assert result.accel_cap is None
  assert result.target == pytest.approx(1.4)


@pytest.mark.parametrize("v_ego", [0.0, 2.0, 4.5, 6.0, 7.9])
def test_follow_policy_low_speed_vision_never_relaxes_braking(v_ego):
  result = run(
    lead(d_rel=8.0, v_lead=max(v_ego - 0.8, 0.0), a_lead=-0.8),
    v_ego=v_ego,
    previous=0.3,
    raw=-0.6,
  )

  assert result.target <= -0.6


def test_follow_policy_bypasses_post_departure_handoff():
  result = run(lead(d_rel=46.0, v_lead=22.0), v_ego=20.0, previous=0.0, raw=0.6, post_departure=True)
  assert result.target == pytest.approx(0.6)
  assert result.accel_cap is None


def test_follow_policy_slews_upward_matched_brake_floor_release():
  # Matched follow (v_ego=25, headway=1.45, relative_speed=1.6, a_lead=-0.33) gives
  # _matched_brake_floor < -FOLLOW_TRANSITION_MAX_BRAKE (-0.25). Releasing upward from
  # a deeper brake (-1.202) must slew smoothly instead of jumping straight to the floor.
  matched_lead = lead(d_rel=36.25, v_lead=23.4, a_lead=-0.33, radar=True)
  result = run(matched_lead, v_ego=25.0, previous=-1.202, raw=-1.202)

  assert result.brake_floor is not None
  assert result.brake_floor < -0.25
  assert result.target == pytest.approx(-1.202 + 0.06)


def test_follow_policy_keeps_downward_and_unmatched_braking_unclipped():
  matched_lead = lead(d_rel=36.25, v_lead=23.4, a_lead=-0.33, radar=True)
  downward = run(matched_lead, v_ego=25.0, previous=0.0, raw=-1.202)
  assert downward.target == pytest.approx(downward.brake_floor)

  deeper_downward = run(matched_lead, v_ego=25.0, previous=-0.10, raw=-0.50)
  assert deeper_downward.target == pytest.approx(deeper_downward.brake_floor)

  # Without a matched follow floor, upward transitions below -0.25 remain unclipped.
  unmatched_lead = lead(d_rel=36.25, v_lead=23.4, a_lead=-0.50, radar=True)
  unmatched = run(unmatched_lead, v_ego=25.0, previous=-1.202, raw=-0.35)
  assert unmatched.brake_floor is None
  assert unmatched.target == pytest.approx(-0.35)


def test_catchup_cap_fades_smoothly_across_gap_buffer():
  v_ego = 20.0
  v_lead = 20.0
  t_follow = 1.45
  buffer = max(FOLLOW_GAP_BUFFER_MIN, FOLLOW_GAP_BUFFER_GAIN * v_ego)
  desired_gap = float(desired_follow_distance(v_ego, v_lead, t_follow))

  def cap_at(gap_error):
    return _catchup_cap(
      lead(d_rel=desired_gap + gap_error, v_lead=v_lead, radar=False),
      v_ego,
      t_follow,
      source="lead0",
      tracking=True,
      post_departure=False,
    )

  cap_at_buffer = cap_at(buffer)
  cap_just_above = cap_at(buffer + 0.08)
  fade_caps = [cap_at(buffer + offset) for offset in (0.0, 0.08, 0.5, 1.5, 2.5, 2.95)]

  assert cap_at_buffer is not None
  assert cap_just_above is not None
  assert cap_just_above - cap_at_buffer < 0.05
  assert all(a < b for a, b in zip(fade_caps, fade_caps[1:]))
  assert 1.45 < fade_caps[-1] < 1.5
  assert cap_at(buffer + FOLLOW_GAP_BUFFER_FADE) is None
  assert cap_at(buffer + FOLLOW_GAP_BUFFER_FADE + 0.5) is None

  res = run(
    lead(d_rel=desired_gap + buffer + 0.08, v_lead=v_lead, radar=False),
    v_ego=v_ego,
    previous=cap_at_buffer,
    raw=1.4,
  )
  assert res.accel_cap == pytest.approx(cap_just_above)
  assert res.target <= cap_just_above + 1e-6


def test_catchup_cap_matched_radar_fades_smoothly_from_matched_release_start():
  v_ego = 24.0
  v_lead = 24.0
  t_follow = 1.45
  buffer = max(FOLLOW_GAP_BUFFER_MIN, FOLLOW_GAP_BUFFER_GAIN * v_ego)
  release_start = buffer - 0.75
  desired_gap = float(desired_follow_distance(v_ego, v_lead, t_follow))

  def cap_at(gap_error):
    return _catchup_cap(
      lead(d_rel=desired_gap + gap_error, v_lead=v_lead, radar=True),
      v_ego,
      t_follow,
      source="lead0",
      tracking=True,
      post_departure=False,
    )

  caps = [cap_at(release_start + offset) for offset in (0.0, 0.08, 0.75, 1.75, 2.95)]
  assert all(c is not None for c in caps)
  assert caps[1] - caps[0] < 0.05
  assert all(a < b for a, b in zip(caps, caps[1:]))
  assert 1.45 < caps[-1] < 1.5
  assert cap_at(release_start + FOLLOW_GAP_BUFFER_FADE) is None
  assert cap_at(release_start + FOLLOW_GAP_BUFFER_FADE + 0.5) is None
