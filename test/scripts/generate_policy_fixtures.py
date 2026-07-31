#!/usr/bin/env python3
"""Generate minimal valid policy packages for gtests.

Packages are written into a build directory (not the source tree):

  python3 test/scripts/generate_policy_fixtures.py --output /path/to/build/fixtures
"""

from __future__ import annotations

import argparse
import hashlib
import shutil
from pathlib import Path
from typing import List, Optional

import numpy as np


def write_meta(
    path: Path,
    *,
    horizon: int = 4,
    control_dt: float = 0.01,
    duration: Optional[float] = None,
) -> None:
    if duration is None:
        duration = control_dt * horizon
    joints = [
        "FL_hip_joint",
        "FL_thigh_joint",
        "FL_calf_joint",
        "FR_hip_joint",
        "FR_thigh_joint",
        "FR_calf_joint",
        "RL_hip_joint",
        "RL_thigh_joint",
        "RL_calf_joint",
        "RR_hip_joint",
        "RR_thigh_joint",
        "RR_calf_joint",
    ]
    ready = [float(i) * 0.01 for i in range(12)]
    perm = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
    unitree = [0.0] * 12
    for i, p in enumerate(perm):
        unitree[p] = ready[i]
    mean = [0.0] * 33
    std = [1.0] * 33
    jo = "\n".join(f"- {j}" for j in joints)
    path.write_text(
        f"""schema_version: 1
model:
  input_name: obs
  output_name: actions
  input_shape: [1, 33]
  output_shape: [1, 12]
  input_dtype: float32
  output_dtype: float32
observation:
  num_obs: 33
  quaternion_format: wxyz
  quaternion_frame: episode_relative_imu
  quaternion_sign_convention: nonnegative_w
  gyro_frame: body
  phase_encoding: cos_sin_normalized_horizon
  layout:
  - name: rel_quat_wxyz
    start: 0
    dim: 4
  - name: joint_pos
    start: 4
    dim: 12
  - name: body_gyro
    start: 16
    dim: 3
  - name: joint_vel
    start: 19
    dim: 12
  - name: phase_cos_sin
    start: 31
    dim: 2
action:
  num_actions: 12
  space: tanh_normalized
  scale_baked_into_onnx: false
  position_correction_limit_rad: 0.6
execution:
  mode: one_shot
  control_dt: {control_dt}
  horizon_N: {horizon}
  duration_s: {duration}
  reference_sampling: zero_order_hold
  completion_behavior: hold_current
pd_gains:
  kp: 50.0
  kd: 3.0
joint_order:
{jo}
normalization:
  baked_into_onnx: true
  eps: 0.01
  mean: {mean}
  std: {std}
ready_position:
  joint_order: policy
  values: {ready}
  unitree_order_values: {unitree}
reference:
  path: reference
  metadata: reference/metadata.yaml
  arrays:
  - q_des.npy
  - qd_des.npy
  - tau_ff.npy
provenance:
  checkpoint: /tmp/does/not/exist/checkpoint.pt
  source_onnx_dir: /tmp/does/not/exist/onnx
"""
    )


def write_ref_meta(path: Path, *, horizon: int = 4, control_dt: float = 0.01) -> None:
    joints = [
        "FL_hip_joint",
        "FL_thigh_joint",
        "FL_calf_joint",
        "FR_hip_joint",
        "FR_thigh_joint",
        "FR_calf_joint",
        "RL_hip_joint",
        "RL_thigh_joint",
        "RL_calf_joint",
        "RR_hip_joint",
        "RR_thigh_joint",
        "RR_calf_joint",
    ]
    jo = "\n".join(f"- {j}" for j in joints)
    path.write_text(
        f"""schema_version: 1
control_dt: {control_dt}
horizon_N: {horizon}
joint_order:
{jo}
source_wbc_run: /tmp/ignored/provenance
"""
    )


def write_npy(path: Path, rows: int, dtype=np.float64) -> None:
    data = np.arange(rows * 12, dtype=dtype).reshape(rows, 12)
    np.save(path, data)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_manifest(pkg: Path) -> None:
    files = [
        "policy.onnx",
        "policy_meta.yaml",
        "reference/metadata.yaml",
        "reference/q_des.npy",
        "reference/qd_des.npy",
        "reference/tau_ff.npy",
    ]
    lines = [f"{sha256_file(pkg / rel)}  {rel}" for rel in files]
    (pkg / "MANIFEST.sha256").write_text("\n".join(lines) + "\n")


def make_package(root: Path, name: str, *, horizon: int = 4, dtype=np.float64) -> Path:
    pkg = root / name
    if pkg.exists():
        shutil.rmtree(pkg)
    (pkg / "reference").mkdir(parents=True)
    write_meta(pkg / "policy_meta.yaml", horizon=horizon)
    write_ref_meta(pkg / "reference" / "metadata.yaml", horizon=horizon)
    write_npy(pkg / "reference" / "q_des.npy", horizon, dtype=dtype)
    write_npy(pkg / "reference" / "qd_des.npy", horizon, dtype=dtype)
    write_npy(pkg / "reference" / "tau_ff.npy", horizon, dtype=dtype)
    (pkg / "policy.onnx").write_bytes(b"ONNX_DUMMY_FOR_TESTS")
    write_manifest(pkg)
    return pkg


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        required=True,
        help="Directory that will contain generated valid_package fixtures",
    )
    args = parser.parse_args(argv)

    root: Path = args.output
    root.mkdir(parents=True, exist_ok=True)
    make_package(root, "valid_package", dtype=np.float64)
    make_package(root, "valid_package_f32", dtype=np.float32)
    print(f"Wrote fixtures under {root}")


if __name__ == "__main__":
    main()
