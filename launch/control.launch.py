from datetime import datetime
from pathlib import Path
import hashlib
import os
import shutil

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


OWNERSHIP_PHRASE = "SPORT_MODE_DISABLED"


def _onnxruntime_version():
    version_file = Path(
        os.environ.get("ONNXRUNTIME_ROOT", "/opt/onnxruntime")
    ) / "VERSION_NUMBER"
    if version_file.is_file():
        return version_file.read_text(encoding="utf-8").strip()
    return "unknown"


def _scalar_or_list(value):
    if isinstance(value, list):
        return value
    return [float(value)] * 12


def _write_runtime_environment(run_directory, policy_dir):
    policy_dir = Path(policy_dir).resolve()
    meta = yaml.safe_load(
        (policy_dir / "policy_meta.yaml").read_text(encoding="utf-8")
    )
    execution = meta["execution"]
    pd_gains = meta["pd_gains"]
    control_dt = float(execution["control_dt"])
    manifest_path = policy_dir / "MANIFEST.sha256"
    package_hash = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    payload = {
        "policy_dir": str(policy_dir),
        "package_hash": package_hash,
        "onnxruntime_version": _onnxruntime_version(),
        "execution_provider": "CPU",
        "policy_rate_hz": 1.0 / control_dt,
        "horizon_N": int(execution["horizon_N"]),
        "duration_s": float(execution["duration_s"]),
        "policy_kp": _scalar_or_list(pd_gains["kp"]),
        "policy_kd": _scalar_or_list(pd_gains["kd"]),
        "completion_behavior": "move_to_neutral",
    }
    (run_directory / "runtime_environment.yaml").write_text(
        yaml.safe_dump(payload, sort_keys=False), encoding="utf-8"
    )


def _snapshot_policy_artifacts(run_directory, policy_dir):
    policy_dir = Path(policy_dir).resolve()
    shutil.copy2(policy_dir / "policy_meta.yaml", run_directory / "policy_meta.yaml")
    shutil.copy2(
        policy_dir / "reference" / "metadata.yaml",
        run_directory / "reference_metadata.yaml",
    )
    shutil.copy2(policy_dir / "MANIFEST.sha256", run_directory / "MANIFEST.sha256")
    _write_runtime_environment(run_directory, policy_dir)


def _launch(context):
    config_path = Path(LaunchConfiguration("config").perform(context)).resolve()
    hardware_mode = LaunchConfiguration("hardware_mode").perform(context).lower() == "true"
    ownership = LaunchConfiguration("ownership_ack").perform(context)
    policy_dir = LaunchConfiguration("policy_dir").perform(context).strip()

    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    supervisor_params = dict(config["safety_supervisor"]["ros__parameters"])
    supervisor_params["hardware_mode"] = hardware_mode
    supervisor_params["ownership_acknowledged"] = ownership == OWNERSHIP_PHRASE
    supervisor_params["policy_dir"] = policy_dir
    policy_params = dict(config["policy_runner"]["ros__parameters"])
    policy_params["policy_dir"] = policy_dir

    actions = [
        Node(
            package="go2_nn_control",
            executable="safety_supervisor",
            name="safety_supervisor",
            output="screen",
            parameters=[supervisor_params],
        ),
        Node(
            package="go2_nn_control",
            executable="policy_runner",
            name="policy_runner",
            output="screen",
            parameters=[policy_params],
        ),
    ]

    logging = config["logging"]
    if logging["enabled"]:
        if not policy_dir:
            raise RuntimeError(
                "logging requires a non-empty policy_dir launch argument"
            )
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        root = Path(logging["output_directory"]).expanduser().resolve()
        run_directory = root / f"go2_{timestamp}"
        run_directory.mkdir(parents=True, exist_ok=False)
        snapshot = run_directory / "experiment.yaml"
        shutil.copy2(config_path, snapshot)
        digest = hashlib.sha256(config_path.read_bytes()).hexdigest()
        (run_directory / "experiment.sha256").write_text(
            f"{digest}  experiment.yaml\n", encoding="utf-8"
        )
        _snapshot_policy_artifacts(run_directory, policy_dir)
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2", "bag", "record",
                    "--storage", str(logging["storage"]),
                    "--output", str(run_directory / "bag"),
                    *[str(topic) for topic in logging["topics"]],
                ],
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("go2_nn_control"))
        / "config"
        / "default_experiment.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("hardware_mode", default_value="false"),
            DeclareLaunchArgument("ownership_ack", default_value=""),
            DeclareLaunchArgument("policy_dir", default_value=""),
            OpaqueFunction(function=_launch),
        ]
    )
