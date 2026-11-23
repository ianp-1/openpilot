import random
from types import SimpleNamespace
import numpy as np

from cereal import messaging
from openpilot.selfdrive.locationd.paramsd import VehicleParamsLearner, resolve_vehicle_model_params, retrieve_initial_vehicle_params, migrate_cached_vehicle_params_if_needed
from openpilot.selfdrive.locationd.models.car_kf import CarKalman
from openpilot.selfdrive.locationd.test.test_locationd_scenarios import TEST_ROUTE
from openpilot.selfdrive.test.process_replay.migration import migrate, migrate_carParams
from openpilot.common.params import Params
from openpilot.tools.lib.logreader import LogReader


def get_random_live_parameters(CP):
  msg = messaging.new_message("liveParameters")
  msg.liveParameters.steerRatio = (random.random() + 0.5) * CP.steerRatio
  msg.liveParameters.stiffnessFactor = random.random()
  msg.liveParameters.angleOffsetAverageDeg = random.random()
  msg.liveParameters.debugFilterState.std = [random.random() for _ in range(CarKalman.P_initial.shape[0])]
  return msg


class TestParamsd:
  def test_force_auto_tune_off_locks_vehicle_model_params(self):
    toggles = SimpleNamespace(force_auto_tune_off=True, use_custom_steerRatio=True, steerRatio=16.8)

    steer_ratio, stiffness = resolve_vehicle_model_params(17.2, 0.84, toggles)

    assert steer_ratio == 16.8
    assert stiffness == 1.0

  def test_custom_steer_ratio_keeps_learned_stiffness(self):
    toggles = SimpleNamespace(force_auto_tune_off=False, use_custom_steerRatio=True, steerRatio=16.8)

    steer_ratio, stiffness = resolve_vehicle_model_params(17.2, 0.84, toggles)

    assert steer_ratio == 16.8
    assert stiffness == 0.84

  def test_vehicle_model_learning_remains_default(self):
    toggles = SimpleNamespace(force_auto_tune_off=False, use_custom_steerRatio=False, steerRatio=16.8)

    steer_ratio, stiffness = resolve_vehicle_model_params(17.2, 0.84, toggles)

    assert steer_ratio == 17.2
    assert stiffness == 0.84

  def test_read_saved_params(self):
    params = Params()

    lr = migrate(LogReader(TEST_ROUTE), [migrate_carParams])
    CP = next(m for m in lr if m.which() == "carParams").carParams

    msg = get_random_live_parameters(CP)
    params.put("LiveParametersV2", msg.to_bytes())
    params.put("CarParamsPrevRoute", CP.as_builder().to_bytes())

    migrate_cached_vehicle_params_if_needed(params) # this is not tested here but should not mess anything up or throw an error
    sr, sf, offset, p_init = retrieve_initial_vehicle_params(params, CP, replay=True, debug=True)
    np.testing.assert_allclose(sr, msg.liveParameters.steerRatio)
    np.testing.assert_allclose(sf, msg.liveParameters.stiffnessFactor)
    np.testing.assert_allclose(offset, msg.liveParameters.angleOffsetAverageDeg)
    np.testing.assert_equal(p_init.shape, CarKalman.P_initial.shape)
    np.testing.assert_allclose(np.diagonal(p_init), msg.liveParameters.debugFilterState.std)

  # TODO Remove this test after the support for old format is removed
  def test_read_saved_params_old_format(self):
    params = Params()

    lr = migrate(LogReader(TEST_ROUTE), [migrate_carParams])
    CP = next(m for m in lr if m.which() == "carParams").carParams

    msg = get_random_live_parameters(CP)
    params.put("LiveParameters", msg.liveParameters.to_dict())
    params.put("CarParamsPrevRoute", CP.as_builder().to_bytes())
    params.remove("LiveParametersV2")

    migrate_cached_vehicle_params_if_needed(params)
    sr, sf, offset, _ = retrieve_initial_vehicle_params(params, CP, replay=True, debug=True)
    np.testing.assert_allclose(sr, msg.liveParameters.steerRatio)
    np.testing.assert_allclose(sf, msg.liveParameters.stiffnessFactor)
    np.testing.assert_allclose(offset, msg.liveParameters.angleOffsetAverageDeg)
    assert params.get("LiveParametersV2") is not None

  def test_read_saved_params_corrupted_old_format(self):
    params = Params()
    params.put("LiveParameters", {})
    params.remove("LiveParametersV2")

    migrate_cached_vehicle_params_if_needed(params)
    assert params.get("LiveParameters") is None
    assert params.get("LiveParametersV2") is None

  def test_mount_roll_offset_subtracts_static_roll_without_affecting_zero_default(self):
    lr = migrate(LogReader(TEST_ROUTE), [migrate_carParams])
    CP = next(m for m in lr if m.which() == "carParams").carParams

    pose_msg = messaging.new_message("livePose")
    pose_msg.livePose.orientationNED.x = float(np.radians(-1.45))
    pose_msg.livePose.orientationNED.xStd = float(np.radians(0.1))
    pose_msg.livePose.angularVelocityDevice.valid = True
    pose_msg.livePose.sensorsOK = True

    learner_default = VehicleParamsLearner(CP, CP.steerRatio, 1.0, 0.0)
    learner_default.handle_log(1.0, "livePose", pose_msg.livePose)
    learner_default.handle_log(1.05, "livePose", pose_msg.livePose)
    np.testing.assert_allclose(learner_default.observed_roll, np.radians(-1.45), atol=1e-6)

    learner_id4 = VehicleParamsLearner(CP, CP.steerRatio, 1.0, 0.0, mount_roll_offset_rad=np.radians(-1.45))
    learner_id4.handle_log(1.0, "livePose", pose_msg.livePose)
    learner_id4.handle_log(1.05, "livePose", pose_msg.livePose)
    np.testing.assert_allclose(learner_id4.observed_roll, 0.0, atol=1e-6)
