from __future__ import annotations

from dataclasses import dataclass

from openpilot.starpilot.common.per_car_calibration import (
  ID4_DEFAULT_CAMERA_OFFSET_M,
  LIVE_CALIBRATION_KEYS,
  sync_per_car_calibration,
)


@dataclass
class _StubCP:
  carFingerprint: str
  brand: str = "volkswagen"

  def to_bytes(self) -> bytes:
    return f"CP:{self.carFingerprint}".encode("ascii")


class _MemoryParams:
  def __init__(self, base_dir):
    self._base_dir = base_dir
    self._store: dict[str, object] = {}

  def get_param_path(self, key: str = "") -> str:
    d = self._base_dir / "d"
    d.mkdir(parents=True, exist_ok=True)
    return str(d / key) if key else str(d)

  def get(self, key: str, block: bool = False, return_default: bool = False, encoding=None, default=None):
    return self._store.get(key, default)

  def get_float(self, key: str, default: float = 0.0) -> float:
    val = self._store.get(key)
    return float(val) if val is not None else float(default)

  def put(self, key: str, val) -> None:
    self._store[key] = val

  def put_float(self, key: str, val: float) -> None:
    self._store[key] = float(val)

  def remove(self, key: str) -> None:
    self._store.pop(key, None)


def test_per_car_calibration_banks_and_restores_across_crv_and_id4(tmp_path):
  params = _MemoryParams(tmp_path)
  bank_path = tmp_path / "per_car_calibration.json"
  crv_cp = _StubCP("HONDA_CRV_5G", brand="honda")
  id4_cp = _StubCP("VOLKSWAGEN_ID4_MK1", brand="volkswagen")

  # 1. CR-V first boot
  assert sync_per_car_calibration(params, None, crv_cp, bank_path=bank_path) is None
  assert params.get_float("CameraOffset") == 0.0
  params.put("CalibrationParams", b"crv-cal")
  params.put("LiveParametersV2", b"crv-lp")
  params.put("LiveDelay", b"crv-delay")

  # 2. Swap CR-V -> unseen ID.4: clears CR-V blobs, sets ID.4 CameraOffset = +0.10
  prev_out = sync_per_car_calibration(params, crv_cp, id4_cp, bank_path=bank_path)
  assert prev_out is crv_cp
  for key in LIVE_CALIBRATION_KEYS:
    assert params.get(key) is None
  assert params.get_float("CameraOffset") == ID4_DEFAULT_CAMERA_OFFSET_M

  # Converge ID.4 blobs + user tweaks CameraOffset slightly in UI to 0.09
  params.put("CalibrationParams", b"id4-cal")
  params.put("LiveParametersV2", b"id4-lp")
  params.put("LiveDelay", b"id4-delay")
  params.put_float("CameraOffset", 0.09)

  # 3. Swap ID.4 -> CR-V: restores CR-V blobs + 0.0, returns CR-V CarParamsPrevRoute
  prev_out = sync_per_car_calibration(params, id4_cp, crv_cp, bank_path=bank_path)
  assert prev_out == crv_cp.to_bytes()
  assert params.get("CalibrationParams") == b"crv-cal"
  assert params.get("LiveParametersV2") == b"crv-lp"
  assert params.get("LiveDelay") == b"crv-delay"
  assert params.get_float("CameraOffset") == 0.0

  # 4. Swap CR-V -> ID.4: restores ID.4 blobs + custom 0.09 CameraOffset on frame 0
  prev_out = sync_per_car_calibration(params, crv_cp, id4_cp, bank_path=bank_path)
  assert prev_out == id4_cp.to_bytes()
  assert params.get("CalibrationParams") == b"id4-cal"
  assert params.get("LiveParametersV2") == b"id4-lp"
  assert params.get("LiveDelay") == b"id4-delay"
  assert params.get_float("CameraOffset") == 0.09


def test_legacy_id4_camera_offset_migrates_without_stomping_user_override(tmp_path):
  params = _MemoryParams(tmp_path)
  bank_path = tmp_path / "per_car_calibration.json"
  id4_cp = _StubCP("VOLKSWAGEN_ID4_MK1", brand="volkswagen")

  params.put_float("CameraOffset", 0.11)
  sync_per_car_calibration(params, id4_cp, id4_cp, bank_path=bank_path)
  assert params.get_float("CameraOffset") == ID4_DEFAULT_CAMERA_OFFSET_M

  params.put_float("CameraOffset", 0.12)
  sync_per_car_calibration(params, id4_cp, id4_cp, bank_path=bank_path)
  assert params.get_float("CameraOffset") == 0.12


def test_explicit_reset_calibration_is_respected(tmp_path):
  params = _MemoryParams(tmp_path)
  bank_path = tmp_path / "per_car_calibration.json"
  crv_cp = _StubCP("HONDA_CRV_5G", brand="honda")
  id4_cp = _StubCP("VOLKSWAGEN_ID4_MK1", brand="volkswagen")

  params.put("CalibrationParams", b"id4-cal")
  sync_per_car_calibration(params, id4_cp, id4_cp, bank_path=bank_path)

  # Simulate UI "Reset Calibration" removing live keys while offroad before a CR-V swap
  for key in LIVE_CALIBRATION_KEYS:
    params.remove(key)
  sync_per_car_calibration(params, id4_cp, crv_cp, bank_path=bank_path)
  for key in LIVE_CALIBRATION_KEYS:
    assert params.get(key) is None

  # Swapping back to ID.4 still restores ID.4's pre-reset bank safely
  prev_out = sync_per_car_calibration(params, crv_cp, id4_cp, bank_path=bank_path)
  assert prev_out == id4_cp.to_bytes()
  assert params.get("CalibrationParams") == b"id4-cal"

  # Same-car UI reset wipes active car's live & banked blobs cleanly
  for key in LIVE_CALIBRATION_KEYS:
    params.remove(key)
  sync_per_car_calibration(params, id4_cp, id4_cp, bank_path=bank_path)
  sync_per_car_calibration(params, id4_cp, crv_cp, bank_path=bank_path)
  prev_out = sync_per_car_calibration(params, crv_cp, id4_cp, bank_path=bank_path)
  assert prev_out is crv_cp
  assert params.get("CalibrationParams") is None
