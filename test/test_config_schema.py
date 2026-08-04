import importlib.util
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_default_experiment_is_complete_and_isolated():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "default_experiment.yaml").read_text()
    )
    params = config["safety_supervisor"]["ros__parameters"]
    required_scalars = {
        "hardware_mode",
        "hardware_profile_complete",
        "command_rate_hz",
        "low_state_timeout_seconds",
        "policy_timeout_seconds",
        "ready_transition_seconds",
        "neutral_transition_seconds",
        "transition_timeout_seconds",
        "position_tolerance",
        "velocity_tolerance",
        "quaternion_min_norm",
        "quaternion_max_norm",
        "shutdown_passive_seconds",
        "motion_switcher_response_timeout_seconds",
        "motion_switcher_verify_timeout_seconds",
        "motion_switcher_publisher_settle_seconds",
        "motion_switcher_publisher_timeout_seconds",
        "low_state_topic",
        "low_command_topic",
        "observation_topic",
        "policy_action_topic",
        "applied_command_topic",
        "status_topic",
        "estop_topic",
        "qos_depth",
    }
    assert required_scalars <= params.keys()
    assert "policy_rate_hz" not in params
    assert "phase_period_seconds" not in params
    assert "kp" not in params
    assert "kd" not in params
    for name in (
        "transition_kp",
        "transition_kd",
        "ready_position",
        "neutral_position",
        "joint_position_min",
        "joint_position_max",
        "desired_velocity_limit",
        "target_rate_limit",
        "feedforward_torque_limit",
    ):
        assert len(params[name]) == 12
    assert params["hardware_mode"] is False
    assert params["hardware_profile_complete"] is False
    assert params["low_command_topic"] != "/lowcmd"
    assert params["low_state_topic"] != "/lowstate"
    assert all(limit == 0.0 for limit in params["feedforward_torque_limit"])


def test_logging_records_raw_and_processed_topics():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "default_experiment.yaml").read_text()
    )
    topics = set(config["logging"]["topics"])
    assert {
        "/ws_control/test_lowstate",
        "/ws_control/test_lowcmd",
        "/ws_control/observation",
        "/ws_control/policy_action",
        "/ws_control/applied_command",
        "/ws_control/status",
    } <= topics


def test_all_profiles_use_runtime_ownership_and_timeouts():
    for name in ("default_experiment.yaml", "mujoco.yaml", "hardware.yaml"):
        config = yaml.safe_load((PACKAGE_ROOT / "config" / name).read_text())
        params = config["safety_supervisor"]["ros__parameters"]
        assert "ownership_acknowledged" not in params
        assert params["motion_switcher_response_timeout_seconds"] > 0.0
        assert params["motion_switcher_verify_timeout_seconds"] > 0.0
        assert params["motion_switcher_publisher_settle_seconds"] >= 0.0
        assert params["motion_switcher_publisher_timeout_seconds"] > 0.0


def test_launch_hardware_mode_uses_config_unless_explicitly_overridden():
    launch_path = PACKAGE_ROOT / "launch" / "control.launch.py"
    spec = importlib.util.spec_from_file_location("control_launch", launch_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert module._resolve_hardware_mode(True, "") is True
    assert module._resolve_hardware_mode(False, "") is False
    assert module._resolve_hardware_mode(False, "true") is True
    assert module._resolve_hardware_mode(True, "false") is False
