import random
import re
from types import SimpleNamespace

from opendbc.can import CANParser
from opendbc.car import Bus
from opendbc.car.structs import CarControl, CarParams
from opendbc.car.volkswagen.carcontroller import CarController
from opendbc.car.volkswagen.interface import CarInterface
from opendbc.car.volkswagen.values import CAR, DBC, FW_QUERY_CONFIG, WMI, VolkswagenFlags, VolkswagenSafetyFlags
from opendbc.car.volkswagen.fingerprints import FW_VERSIONS

Ecu = CarParams.Ecu

CHASSIS_CODE_PATTERN = re.compile('[A-Z0-9]{2}')
# TODO: determine the unknown groups
SPARE_PART_FW_PATTERN = re.compile(b'\xf1\x87(?P<gateway>[0-9][0-9A-Z]{2})(?P<unknown>[0-9][0-9A-Z][0-9])(?P<unknown2>[0-9A-Z]{2}[0-9])([A-Z0-9]| )')


class TestVolkswagenPlatformConfigs:
  MEB_CARS = {car for car in CAR if car.config.flags & VolkswagenFlags.MEB}

  @staticmethod
  def _get_meb_params(car, gateway=True, alpha_long=False):
    fingerprint = {bus: {} for bus in range(8)}
    if gateway:
      fingerprint[1][0x13D] = 32
    return CarInterface.get_params(car, fingerprint, [], alpha_long, False, False, None)

  def test_meb_platform_params(self):
    for car in self.MEB_CARS:
      cp = self._get_meb_params(car)
      assert cp.flags & VolkswagenFlags.MEB
      assert cp.transmissionType == CarParams.TransmissionType.direct
      assert cp.steerControlType == CarParams.SteerControlType.curvatureDEPRECATED
      assert cp.steerAtStandstill
      assert cp.safetyConfigs[-1].safetyModel == CarParams.SafetyModel.volkswagenMeb
      assert not cp.dashcamOnly
      assert not cp.radarUnavailable

      has_gen2_crc = bool(cp.safetyConfigs[-1].safetyParam & VolkswagenSafetyFlags.MEB_ALT_CRC)
      assert has_gen2_crc == bool(car.config.flags & VolkswagenFlags.MEB_GEN2)

  def test_meb_camera_harness_is_passive(self):
    cp = self._get_meb_params(CAR.VOLKSWAGEN_ID4_MK1, gateway=False, alpha_long=True)
    assert cp.dashcamOnly
    assert cp.radarUnavailable
    assert not cp.alphaLongitudinalAvailable
    assert not cp.openpilotLongitudinalControl
    assert not (cp.safetyConfigs[-1].safetyParam & VolkswagenSafetyFlags.LONG_CONTROL)

  def test_meb_docs_assume_required_gateway_harness(self):
    fingerprint = {bus: {} for bus in range(8)}
    cp = CarInterface.get_params(CAR.VOLKSWAGEN_ID4_MK1, fingerprint, [], True, False, True, None)

    assert cp.networkLocation == CarParams.NetworkLocation.gateway
    assert not cp.dashcamOnly
    assert cp.alphaLongitudinalAvailable

  def test_meb_gateway_longitudinal(self):
    cp = self._get_meb_params(CAR.VOLKSWAGEN_ID4_MK1, gateway=True, alpha_long=True)
    assert cp.alphaLongitudinalAvailable
    assert cp.openpilotLongitudinalControl
    assert not cp.pcmCruise
    assert cp.safetyConfigs[-1].safetyParam & VolkswagenSafetyFlags.LONG_CONTROL

  def test_taos_longitudinal_actuator_delay(self):
    taos_cp = CarInterface.get_non_essential_params(CAR.VOLKSWAGEN_TAOS_MK1)
    golf_cp = CarInterface.get_non_essential_params(CAR.VOLKSWAGEN_GOLF_MK7)

    assert abs(taos_cp.longitudinalActuatorDelay - 0.25) < 1e-6
    assert abs(golf_cp.longitudinalActuatorDelay - 0.15) < 1e-6

  def test_meb_steer_actuator_delay(self):
    for car in self.MEB_CARS:
      cp = self._get_meb_params(car)
      assert abs(cp.steerActuatorDelay - 0.2) < 1e-6
      assert abs(cp.lateralSmoothSeconds - 0.22) < 1e-6
    assert abs(self._get_meb_params(CAR.VOLKSWAGEN_ID4_MK1).steerRatio - 14.0) < 1e-6

    golf_cp = CarInterface.get_non_essential_params(CAR.VOLKSWAGEN_GOLF_MK7)
    assert abs(golf_cp.steerActuatorDelay - 0.1) < 1e-6
    assert abs(golf_cp.lateralSmoothSeconds - 0.0) < 1e-6

  @staticmethod
  def _run_meb_hca03_override_power(steering_torque_sequence):
    cp = TestVolkswagenPlatformConfigs._get_meb_params(CAR.VOLKSWAGEN_ID4_MK1, gateway=True, alpha_long=False)
    cc = CarController(DBC[cp.carFingerprint], cp)
    parser = CANParser(DBC[cp.carFingerprint][Bus.pt], [("HCA_03", 0)], cc.CAN.pt)

    CC = CarControl.new_message()
    CC.enabled = True
    CC.latActive = True
    CC.currentCurvature = 0.0
    CC.actuators.curvature = 0.0
    cc_reader = CC.as_reader()

    powers = []
    for step, steering_torque in enumerate(steering_torque_sequence):
      CS = SimpleNamespace(
        out=SimpleNamespace(
          cruiseState=SimpleNamespace(available=True),
          vEgo=15.0,
          vEgoRaw=15.0,
          steeringTorque=steering_torque,
          steeringPressed=abs(steering_torque) > cc.CCP.STEER_DRIVER_ALLOWANCE,
        ),
        curvature_meas=0.0,
        ldw_stock_values={},
        gra_stock_values={"COUNTER": 0},
      )
      for sub in range(cc.CCP.STEER_STEP):
        t_ns = (step * cc.CCP.STEER_STEP + sub) * 10_000_000
        _, can_sends = cc.update(cc_reader, CS, t_ns, None)
        hca_msgs = [msg for msg in can_sends if msg[0] == 0x303]
        if hca_msgs:
          parser.update([(t_ns, hca_msgs)])
          powers.append(parser.vl["HCA_03"]["Power"])
    return powers

  def test_meb_hca03_power_symmetric_driver_override(self):
    warmup = [0.0] * 25
    left_powers = self._run_meb_hca03_override_power(warmup + [300.0] * 25)
    right_powers = self._run_meb_hca03_override_power(warmup + [-300.0] * 25)

    assert abs(left_powers[24] - 50.0) < 1e-6
    assert abs(right_powers[24] - 50.0) < 1e-6
    assert right_powers[25] < 50.0
    assert right_powers == left_powers
    assert abs(left_powers[-1] - 4.0) < 1e-6
    assert abs(right_powers[-1] - 4.0) < 1e-6

  @staticmethod
  def _run_meb_hca03_curvatures(v_ego, target_curvatures, *, curvature_meas=0.0, current_curvature=0.0, steering_pressed=False):
    cp = TestVolkswagenPlatformConfigs._get_meb_params(CAR.VOLKSWAGEN_ID4_MK1, gateway=True, alpha_long=False)
    cc = CarController(DBC[cp.carFingerprint], cp)
    parser = CANParser(DBC[cp.carFingerprint][Bus.pt], [("HCA_03", 0)], cc.CAN.pt)

    curvatures = []
    for step, target_curvature in enumerate(target_curvatures):
      CC = CarControl.new_message()
      CC.enabled = True
      CC.latActive = True
      CC.currentCurvature = current_curvature
      CC.actuators.curvature = target_curvature
      cc_reader = CC.as_reader()
      CS = SimpleNamespace(
        out=SimpleNamespace(
          cruiseState=SimpleNamespace(available=True),
          vEgo=v_ego,
          vEgoRaw=v_ego,
          steeringTorque=0.0,
          steeringPressed=steering_pressed,
        ),
        curvature_meas=curvature_meas,
        ldw_stock_values={},
        gra_stock_values={"COUNTER": 0},
      )
      for sub in range(cc.CCP.STEER_STEP):
        t_ns = (step * cc.CCP.STEER_STEP + sub) * 10_000_000
        _, can_sends = cc.update(cc_reader, CS, t_ns, None)
        hca_msgs = [msg for msg in can_sends if msg[0] == 0x303]
        if hca_msgs:
          parser.update([(t_ns, hca_msgs)])
          mag = parser.vl["HCA_03"]["Curvature"]
          sign = 1.0 if parser.vl["HCA_03"]["Curvature_VZ"] else -1.0
          curvatures.append(mag * sign)
    return curvatures

  def test_meb_low_speed_curvature_offset_suppression_and_smoothing(self):
    low_speed = self._run_meb_hca03_curvatures(
      3.0, [0.010] * 100, curvature_meas=0.008, current_curvature=0.012,
    )
    mid_15mph = self._run_meb_hca03_curvatures(
      6.7, [0.010] * 100, curvature_meas=0.008, current_curvature=0.012,
    )
    high_speed = self._run_meb_hca03_curvatures(
      20.0, [0.010] * 120, curvature_meas=0.008, current_curvature=0.012,
    )
    assert abs(low_speed[-1] - 0.010) < 2e-5
    assert abs(mid_15mph[-1] - 0.010) < 2e-5
    assert abs(high_speed[-1] - 0.006) < 2e-5

    # Alternating ~2 Hz curvature wave (0.26s half-period at 50 Hz = 13 steps per half-cycle)
    osc = ([0.0015] * 13 + [-0.0015] * 13) * 6
    low_osc = self._run_meb_hca03_curvatures(3.0, osc)
    osc_15mph = self._run_meb_hca03_curvatures(6.7, osc)
    mid_osc = self._run_meb_hca03_curvatures(15.0, osc)
    override_osc = self._run_meb_hca03_curvatures(6.7, osc, steering_pressed=True)
    low_ptp = max(low_osc[-26:]) - min(low_osc[-26:])
    ptp_15mph = max(osc_15mph[-26:]) - min(osc_15mph[-26:])
    mid_ptp = max(mid_osc[-26:]) - min(mid_osc[-26:])
    override_ptp = max(override_osc[-26:]) - min(override_osc[-26:])
    assert low_ptp < mid_ptp * 0.75
    assert abs(ptp_15mph - low_ptp) < 2e-5
    assert abs(override_ptp - mid_ptp) < 2e-4

  def test_meb_bumpless_mid_turn_reengage(self):
    cp = self._get_meb_params(CAR.VOLKSWAGEN_ID4_MK1, gateway=True, alpha_long=False)
    cc = CarController(DBC[cp.carFingerprint], cp)
    parser = CANParser(DBC[cp.carFingerprint][Bus.pt], [("HCA_03", 0)], cc.CAN.pt)

    cs = SimpleNamespace(
      out=SimpleNamespace(
        cruiseState=SimpleNamespace(available=True),
        vEgo=15.0,
        vEgoRaw=15.0,
        steeringTorque=0.0,
        steeringPressed=False,
      ),
      curvature_meas=0.008,
      ldw_stock_values={},
      gra_stock_values={"COUNTER": 0},
    )

    def _step_once(lat_active: bool, target_curv: float, current_curv: float, step_idx: int) -> float:
      msg = CarControl.new_message(enabled=lat_active, latActive=lat_active, currentCurvature=current_curv)
      msg.actuators.curvature = target_curv
      reader = msg.as_reader()
      out = 0.0
      for sub in range(cc.CCP.STEER_STEP):
        t_ns = (step_idx * cc.CCP.STEER_STEP + sub) * 10_000_000
        _, sends = cc.update(reader, cs, t_ns, None)
        hca_msgs = [m for m in sends if m[0] == 0x303]
        if hca_msgs:
          parser.update([(t_ns, hca_msgs)])
          mag = parser.vl["HCA_03"]["Curvature"]
          sign = 1.0 if parser.vl["HCA_03"]["Curvature_VZ"] else -1.0
          out = mag * sign
      return out

    for i in range(5):
      _step_once(False, 0.0, 0.011, i)
    first_active_curv = _step_once(True, 0.011, 0.011, 5)
    assert abs(first_active_curv - 0.008) < 2e-5

  def test_spare_part_fw_pattern(self, subtests):
    # Relied on for determining if a FW is likely VW
    for platform, ecus in FW_VERSIONS.items():
      with subtests.test(platform=platform.value):
        for fws in ecus.values():
          for fw in fws:
            assert SPARE_PART_FW_PATTERN.match(fw) is not None, f"Bad FW: {fw}"

  def test_chassis_codes(self, subtests):
    for platform in CAR:
      with subtests.test(platform=platform.value):
        assert len(platform.config.wmis) > 0, "WMIs not set"
        assert len(platform.config.chassis_codes) > 0, "Chassis codes not set"
        assert all(CHASSIS_CODE_PATTERN.match(cc) for cc in
                   platform.config.chassis_codes), "Bad chassis codes"

        # Shared MEB chassis codes are valid only when VIN model-year sets are disjoint.
        for comp in CAR:
          if platform == comp:
            continue
          shared_chassis = platform.config.chassis_codes & comp.config.chassis_codes
          if shared_chassis:
            both_meb = platform.config.flags & VolkswagenFlags.MEB and comp.config.flags & VolkswagenFlags.MEB
            disjoint_years = (getattr(platform.config, "model_years", set()) and getattr(comp.config, "model_years", set()) and
                              not platform.config.model_years & comp.config.model_years)
            assert both_meb and disjoint_years, f"Shared chassis codes: {comp}"

  def test_custom_fuzzy_fingerprinting(self, subtests):
    all_radar_fw = list({fw for ecus in FW_VERSIONS.values() for fw in ecus[Ecu.fwdRadar, 0x757, None]})

    for platform in CAR:
      with subtests.test(platform=platform.name):
        model_years = getattr(platform.config, "model_years", set()) or {"0"}
        for wmi in WMI:
          for chassis_code in platform.config.chassis_codes | {"00"}:
            for model_year in model_years:
              vin = ["0"] * 17
              vin[0:3] = wmi
              vin[6:8] = chassis_code
              vin[9] = model_year
              vin = "".join(vin)

              # Check a few FW cases - expected, unexpected
              for radar_fw in random.sample(all_radar_fw, 5) + [b'\xf1\x875Q0907572G \xf1\x890571', b'\xf1\x877H9907572AA\xf1\x890396']:
                should_match = ((wmi in platform.config.wmis and chassis_code in platform.config.chassis_codes) and
                                radar_fw in all_radar_fw)

                live_fws = {(0x757, None): [radar_fw]}
                matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(live_fws, vin, FW_VERSIONS)

                expected_matches = {platform} if should_match else set()
                assert expected_matches == matches, "Bad match"
