# Go2 NN Control

This package is a fail-closed wrapper around Unitree Go2 `/lowstate` and
`/lowcmd`. The default policy holds measured joint positions. Replace
`hold_current_policy` in `policy_runner.cpp`, or construct `PolicyRunner` with
another `PolicyFunction`, when the network integration is available.

## Build and local launch

```bash
source /home/fengjun/robot/vendor/unitree_ros2/setup_local.sh
cd /home/fengjun/robot/ws_control
colcon build --symlink-install
source install/setup.bash
ros2 launch go2_nn_control control.launch.py
```

The default experiment profile is deliberately local-only and uses
`/ws_control/test_lowstate` and `/ws_control/test_lowcmd`, not the robot topics. Copy
`config/default_experiment.yaml`, fill in measured ready/neutral positions and
real hardware limits, change the topics to `/lowstate` and `/lowcmd`, and set
`hardware_profile_complete: true` only after review.

Hardware launch requires manual shutdown of Unitree sport mode and an explicit
per-run acknowledgment:

```bash
ros2 launch go2_nn_control control.launch.py \
  config:=/absolute/path/to/experiment.yaml \
  hardware_mode:=true \
  ownership_ack:=SPORT_MODE_DISABLED
```

The supervisor additionally requires healthy `/lowstate` before arming. It
cannot detect a controller internal to robot firmware; the ownership phrase is
an operator assertion, not proof.

## Controls

| Key | Service | Effect |
|---|---|---|
| `o` | `/ws_control/acknowledge_ownership` | Local-mode acknowledgment |
| `a` | `/ws_control/arm` | Move to ready |
| `p` | `/ws_control/start_policy` | Reset reference/phase and start |
| `s` | `/ws_control/stop_policy` | Stop and hold measured pose |
| `r` | `/ws_control/recover` | Move held pose to neutral |
| `e` or Space | `/ws_control/estop` | Latch passive zero-torque mode |
| `x` | `/ws_control/reset_estop` | Reset only with healthy state |

The launch file does not start the keyboard node because ROS 2 launch does not
reliably forward terminal input to child nodes. Run it in a separate configured
terminal:

```bash
ros2 run go2_nn_control keyboard_control
```

The Boolean `/ws_control/estop_request` topic is another E-stop input.

## Observation conventions

The typed policy receives native Unitree joint order and a relative quaternion
`inverse(q_start) * q_current`. `q_start` is captured at each policy start.
`gather_policy_observation_33()` is a standalone future-NN helper producing:

`quat(wxyz), q(FL,FR,RL,RR), gyro(xyz), dq(FL,FR,RL,RR), phase(cos,sin)`.

## Testing

Detailed, command-by-command procedures, expected behavior, goals, and pass
criteria are in [TESTING.md](TESTING.md).

### Risk ladder

- **Level 0 — none:** unit tests; never starts DDS command nodes.
- **Level 1 — none only when isolated:** fake-state launch tests on a dedicated
  ROS domain and loopback DDS.
- **Level 2 — very low:** robot-network monitoring without any `/lowcmd`
  publisher.
- **Level 3 — low but nonzero:** passive messages on supported hardware.
- **Level 4 — medium:** ready/neutral PD transitions with the robot suspended.
- **Level 5 — high:** mock policy and watchdog testing while suspended.
- **Level 6 — critical:** user policy on the ground.

Run `ros2 run go2_nn_control run_risk_test.sh 0` for Level 0. Levels 3–6
require a printed acknowledgment but are never automatically started. A
software E-stop is not a substitute for an independent physical torque/power
cutoff.
