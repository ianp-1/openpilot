import math

LATERAL_CONTROL_SOFTWARE_DELAY = 0.2
VW_MEB_FLAG = 16  # VolkswagenFlags.MEB
VW_MEB_LOW_SPEED_LAT_DELAY = 0.22
VW_MEB_LAT_DELAY_BP = (5.0, 14.0)


def full_lateral_delay(vehicle_delay: float) -> float:
  return float(vehicle_delay) + LATERAL_CONTROL_SOFTWARE_DELAY


def get_effective_lateral_delay(CP, v_ego: float, live_lateral_delay: float, use_custom_delay: bool = False) -> float:
  delay = float(live_lateral_delay)
  if use_custom_delay or getattr(CP, "brand", "") != "volkswagen":
    return delay
  if not (int(getattr(CP, "flags", 0)) & VW_MEB_FLAG):
    return delay
  if not (math.isfinite(delay) and math.isfinite(v_ego)):
    return delay
  low_delay = min(delay, VW_MEB_LOW_SPEED_LAT_DELAY)
  lo_v, hi_v = VW_MEB_LAT_DELAY_BP
  w = min(max((float(v_ego) - lo_v) / (hi_v - lo_v), 0.0), 1.0)
  return low_delay + (delay - low_delay) * w
