from __future__ import annotations

import base64
import json
import math
import os
from pathlib import Path


ID4_DEFAULT_CAMERA_OFFSET_M = 0.10
LEGACY_ID4_CAMERA_OFFSET_M = 0.11
ID4_DEFAULT_MOUNT_ROLL_OFFSET_DEG = -1.45
LIVE_CALIBRATION_KEYS = (
  "CalibrationParams",
  "LiveParametersV2",
  "LiveDelay",
  "LiveTorqueParameters",
)


def default_camera_offset_for_car(car_fingerprint: str) -> float:
  return ID4_DEFAULT_CAMERA_OFFSET_M if str(car_fingerprint).startswith("VOLKSWAGEN_ID4") else 0.0


def default_mount_roll_offset_deg_for_car(car_fingerprint: str) -> float:
  return ID4_DEFAULT_MOUNT_ROLL_OFFSET_DEG if str(car_fingerprint).startswith("VOLKSWAGEN_ID4") else 0.0


def _default_bank_path(params) -> Path:
  return Path(params.get_param_path()).resolve().parent / "per_car_calibration.json"


def _load_bank(path: Path) -> dict:
  try:
    data = json.loads(path.read_text(encoding="utf-8"))
    if isinstance(data, dict) and isinstance(data.get("cars"), dict):
      return data
  except Exception:
    pass
  return {"version": 1, "cars": {}}


def _save_bank(path: Path, bank: dict) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  tmp = path.with_suffix(".tmp")
  with open(tmp, "w", encoding="utf-8", newline="\n") as f:
    json.dump(bank, f, indent=2, sort_keys=True)
    f.write("\n")
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, path)


def _extract_fingerprint(cp_or_bytes) -> str:
  if cp_or_bytes is None:
    return ""
  if isinstance(cp_or_bytes, (bytes, bytearray)):
    try:
      from cereal import car
      with car.CarParams.from_bytes(bytes(cp_or_bytes)) as cp:
        return str(getattr(cp, "carFingerprint", "") or "")
    except Exception:
      return ""
  return str(getattr(cp_or_bytes, "carFingerprint", "") or "")


def _serialize_cp(cp):
  if isinstance(cp, (bytes, bytearray)):
    return bytes(cp)
  if hasattr(cp, "to_bytes"):
    if hasattr(cp, "clear_write_flag"):
      cp.clear_write_flag()
    out = cp.to_bytes()
    if hasattr(cp, "clear_write_flag"):
      cp.clear_write_flag()
    return out
  return cp


def _read_camera_offset(params) -> float | None:
  raw = params.get("CameraOffset")
  if raw in (None, b"", ""):
    return None
  try:
    val = float(raw)
  except (TypeError, ValueError):
    val = float(params.get_float("CameraOffset"))
  return val if math.isfinite(val) else None


def _encode_blobs(live_blobs: dict[str, bytes | None]) -> dict[str, str]:
  encoded: dict[str, str] = {}
  for k, val in live_blobs.items():
    if val is not None:
      encoded[k] = base64.b64encode(bytes(val)).decode("ascii")
  return encoded


def _resolve_camera_offset(fp: str, saved_cam) -> float:
  if saved_cam is None or not math.isfinite(float(saved_cam)):
    return default_camera_offset_for_car(fp)
  saved = float(saved_cam)
  if fp.startswith("VOLKSWAGEN_ID4") and math.isclose(saved, LEGACY_ID4_CAMERA_OFFSET_M, abs_tol=1e-4):
    return ID4_DEFAULT_CAMERA_OFFSET_M
  return saved


def _resolve_mount_roll_offset_deg(fp: str, saved_roll) -> float:
  if saved_roll is not None:
    try:
      val = float(saved_roll)
      if math.isfinite(val):
        return val
    except (TypeError, ValueError):
      pass
  return default_mount_roll_offset_deg_for_car(fp)


def get_mount_roll_offset_deg(params, car_fingerprint: str, *, bank_path: Path | None = None) -> float:
  fp = str(car_fingerprint or "")
  if not fp:
    return 0.0
  resolved_path = bank_path if bank_path is not None else _default_bank_path(params)
  bank = _load_bank(resolved_path)
  entry = bank.get("cars", {}).get(fp) or {}
  return _resolve_mount_roll_offset_deg(fp, entry.get("mount_roll_offset_deg"))


def sync_per_car_calibration(params, prev_cp_bytes, curr_cp, *, bank_path: Path | None = None):
  """Bank and restore calibration/learner blobs + CameraOffset per carFingerprint."""
  curr_fp = _extract_fingerprint(curr_cp)
  if not curr_fp or getattr(curr_cp, "brand", "mock") in ("", "mock"):
    return prev_cp_bytes

  resolved_path = bank_path if bank_path is not None else _default_bank_path(params)
  bank = _load_bank(resolved_path)
  cars = bank.setdefault("cars", {})

  prev_fp = _extract_fingerprint(prev_cp_bytes)
  live_blobs = {k: params.get(k) for k in LIVE_CALIBRATION_KEYS}
  cur_cam = _read_camera_offset(params)

  prev_entry = cars.get(prev_fp) if prev_fp else None
  prev_had_blobs = bool(prev_entry and any(prev_entry.get("blobs", {}).values()))
  manual_reset = prev_had_blobs and not any(v is not None for v in live_blobs.values())

  if not prev_fp or prev_fp == curr_fp:
    curr_entry = cars.setdefault(curr_fp, {"blobs": {}, "camera_offset": None})
    curr_entry["blobs"] = _encode_blobs(live_blobs)
    prev_cam = curr_entry.get("camera_offset")
    is_id4 = curr_fp.startswith("VOLKSWAGEN_ID4")
    if cur_cam is None:
      target_cam = _resolve_camera_offset(curr_fp, prev_cam)
      params.put_float("CameraOffset", target_cam)
      curr_entry["camera_offset"] = target_cam
    elif is_id4 and prev_cam is None and (
      math.isclose(cur_cam, LEGACY_ID4_CAMERA_OFFSET_M, abs_tol=1e-4)
      or math.isclose(cur_cam, 0.0, abs_tol=1e-4)
    ):
      params.put_float("CameraOffset", ID4_DEFAULT_CAMERA_OFFSET_M)
      curr_entry["camera_offset"] = ID4_DEFAULT_CAMERA_OFFSET_M
    else:
      curr_entry["camera_offset"] = float(cur_cam)
    curr_entry["mount_roll_offset_deg"] = _resolve_mount_roll_offset_deg(curr_fp, curr_entry.get("mount_roll_offset_deg"))
    _save_bank(resolved_path, bank)
    return prev_cp_bytes

  # Car swap: bank outgoing vehicle's live state unless user explicitly reset calibration.
  prev_entry = cars.setdefault(prev_fp, {"blobs": {}, "camera_offset": None})
  if not manual_reset:
    prev_entry["blobs"] = _encode_blobs(live_blobs)
  if cur_cam is not None:
    if (
      prev_fp.startswith("VOLKSWAGEN_ID4")
      and prev_entry.get("camera_offset") is None
      and math.isclose(cur_cam, LEGACY_ID4_CAMERA_OFFSET_M, abs_tol=1e-4)
    ):
      prev_entry["camera_offset"] = ID4_DEFAULT_CAMERA_OFFSET_M
    else:
      prev_entry["camera_offset"] = float(cur_cam)
  prev_entry["mount_roll_offset_deg"] = _resolve_mount_roll_offset_deg(prev_fp, prev_entry.get("mount_roll_offset_deg"))

  curr_entry = cars.setdefault(curr_fp, {"blobs": {}, "camera_offset": None})
  curr_blobs = curr_entry.get("blobs") or {}

  if manual_reset:
    curr_entry["blobs"] = {}
    for k in LIVE_CALIBRATION_KEYS:
      params.remove(k)
    effective_prev_cp = prev_cp_bytes
  elif any(curr_blobs.get(k) is not None for k in LIVE_CALIBRATION_KEYS):
    for k in LIVE_CALIBRATION_KEYS:
      encoded = curr_blobs.get(k)
      if encoded is not None:
        params.put(k, base64.b64decode(encoded.encode("ascii"), validate=True))
      else:
        params.remove(k)
    effective_prev_cp = _serialize_cp(curr_cp)
  else:
    for k in LIVE_CALIBRATION_KEYS:
      params.remove(k)
    effective_prev_cp = prev_cp_bytes

  target_cam = _resolve_camera_offset(curr_fp, curr_entry.get("camera_offset"))
  curr_entry["camera_offset"] = float(target_cam)
  curr_entry["mount_roll_offset_deg"] = _resolve_mount_roll_offset_deg(curr_fp, curr_entry.get("mount_roll_offset_deg"))
  params.put_float("CameraOffset", float(target_cam))
  _save_bank(resolved_path, bank)
  return effective_prev_cp
