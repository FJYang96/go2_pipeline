#!/usr/bin/env python3
"""Generate parity.npz for a packaged policy using Python onnxruntime.

Example (on the host, with the hop package available):

  python3 test/scripts/generate_parity_npz.py \\
      --policy-dir ../../../../misc/hop_robustify_20260724_164647 \\
      --in-place

This writes <policy_dir>/parity.npz and refreshes MANIFEST.sha256.
"""

from __future__ import annotations

import argparse
import hashlib
import math
from pathlib import Path
from typing import List

import numpy as np

# Unitree FR,FL,RR,RL <- policy FL,FR,RL,RR
_POLICY_TO_UNITREE = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]


def reorder_policy_to_unitree(values: np.ndarray) -> np.ndarray:
    out = np.zeros_like(values)
    for policy_i, unitree_i in enumerate(_POLICY_TO_UNITREE):
        out[..., unitree_i] = values[..., policy_i]
    return out


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_manifest(pkg: Path) -> None:
    files = [
        "policy.onnx",
        "policy_meta.yaml",
        "obs_normalizer.npz",
        "parity.npz",
        "reference/metadata.yaml",
        "reference/q_des.npy",
        "reference/qd_des.npy",
        "reference/tau_ff.npy",
    ]
    lines = []
    for rel in files:
        path = pkg / rel
        if path.is_file():
            lines.append(f"{sha256_file(path)}  {rel}")
    (pkg / "MANIFEST.sha256").write_text("\n".join(lines) + "\n")


def build_obs_rows(
    q_des: np.ndarray,
    qd_des: np.ndarray,
    times: np.ndarray,
    duration_s: float,
) -> np.ndarray:
    """Synthetic but consistent packed observations for offline confirmation."""
    rows = []
    for t, q, dq in zip(times, q_des, qd_des):
        angle = 2.0 * math.pi * float(t) / float(duration_s)
        obs = np.zeros(33, dtype=np.float64)
        obs[0:4] = np.array([1.0, 0.0, 0.0, 0.0])
        obs[4:16] = q
        obs[16:19] = 0.0
        obs[19:31] = dq
        obs[31] = math.cos(angle)
        obs[32] = math.sin(angle)
        rows.append(obs)
    return np.stack(rows, axis=0)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--policy-dir", type=Path, required=True)
    parser.add_argument(
        "--times",
        type=float,
        nargs="+",
        default=[0.0, 0.05, 0.55, 1.09],
        help="Policy times in seconds (must be within [0, duration_s))",
    )
    parser.add_argument(
        "--in-place",
        action="store_true",
        help="Write parity.npz into the policy directory and refresh MANIFEST",
    )
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    try:
        import onnxruntime as ort
    except ImportError as exc:
        raise SystemExit(
            "onnxruntime is required on the host to generate parity.npz"
        ) from exc

    import yaml

    policy_dir: Path = args.policy_dir.resolve()
    meta = yaml.safe_load((policy_dir / "policy_meta.yaml").read_text())
    control_dt = float(meta["execution"]["control_dt"])
    duration_s = float(meta["execution"]["duration_s"])
    limit = float(meta["action"]["position_correction_limit_rad"])
    input_name = meta["model"]["input_name"]
    output_name = meta["model"]["output_name"]

    q_des = np.load(policy_dir / "reference" / "q_des.npy")
    qd_des = np.load(policy_dir / "reference" / "qd_des.npy")
    tau_ff = np.load(policy_dir / "reference" / "tau_ff.npy")

    times = np.asarray(args.times, dtype=np.float64)
    for t in times:
        if not (0.0 <= t < duration_s):
            raise SystemExit(f"time {t} outside [0, {duration_s})")
    ref_index = np.floor(times / control_dt).astype(np.int64)
    obs = build_obs_rows(q_des[ref_index], qd_des[ref_index], times, duration_s)

    session = ort.InferenceSession(
        str(policy_dir / "policy.onnx"), providers=["CPUExecutionProvider"]
    )
    actions = session.run(
        [output_name], {input_name: obs.astype(np.float32)}
    )[0].astype(np.float64)

    q_policy = q_des[ref_index] + np.clip(actions, -1.0, 1.0) * limit
    qd_policy = qd_des[ref_index]
    tau_policy = tau_ff[ref_index]

    payload = {
        "schema_version": np.int64(1),
        "batch_size": np.int64(len(times)),
        "duration_s": np.float64(duration_s),
        "policy_time_seconds": times,
        "obs": obs.astype(np.float64),
        "actions": actions.astype(np.float64),
        "q_des_unitree": reorder_policy_to_unitree(q_policy),
        "qd_des_unitree": reorder_policy_to_unitree(qd_policy),
        "tau_ff_unitree": reorder_policy_to_unitree(tau_policy),
        "reference_index": ref_index,
        "atol": np.float64(1e-5),
        "rtol": np.float64(1e-5),
    }

    output = args.output
    if args.in_place:
        output = policy_dir / "parity.npz"
    if output is None:
        raise SystemExit("pass --in-place or --output")

    np.savez(output, **payload)
    if args.in_place:
        write_manifest(policy_dir)
    print(f"Wrote {output} with batch_size={len(times)}")


if __name__ == "__main__":
    main()
