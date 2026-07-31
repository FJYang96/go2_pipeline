from datetime import datetime
from pathlib import Path
import hashlib
import shutil

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


OWNERSHIP_PHRASE = "SPORT_MODE_DISABLED"


def _launch(context):
    config_path = Path(LaunchConfiguration("config").perform(context)).resolve()
    hardware_mode = LaunchConfiguration("hardware_mode").perform(context).lower() == "true"
    ownership = LaunchConfiguration("ownership_ack").perform(context)

    with config_path.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    supervisor_params = dict(config["safety_supervisor"]["ros__parameters"])
    supervisor_params["hardware_mode"] = hardware_mode
    supervisor_params["ownership_acknowledged"] = ownership == OWNERSHIP_PHRASE
    policy_params = config["policy_runner"]["ros__parameters"]

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
            OpaqueFunction(function=_launch),
        ]
    )
