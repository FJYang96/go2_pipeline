# Go2 Controller Test Manual

This manual progresses from tests that cannot command hardware to tests that
produce real actuator torque. Do not skip levels. A successful software test
does not establish that gains, joint limits, stance targets, or a learned
policy are safe for a physical robot.

All commands assume the repository is at `/home/fengjun/robot`. Commands using
`setup_local.sh` communicate over loopback only. Commands using `setup.sh` use
the robot network interface configured by the Unitree installation.

## Level 0: Build and unit tests

**Risk:** None. No ROS nodes or DDS command publishers are started.

**Goal:** Verify compilation, quaternion math, the exact 33D observation
ordering, Unitree-to-policy joint permutation, CRC generation, and the
configuration schema.

Run:

```bash
cd /home/fengjun/robot/ws_control
source /home/fengjun/robot/vendor/unitree_ros2/setup_local.sh
colcon build --symlink-install --packages-select go2_nn_control
source install/setup.bash
colcon test --packages-select go2_nn_control --event-handlers console_direct+
colcon test-result --verbose
```

Alternatively, after building:

```bash
ros2 run go2_nn_control run_risk_test.sh 0
```

Expected behavior:

- The package builds without errors.
- `test_observation_wrappers` reports four passing tests.
- `test_unitree_crc` reports one passing test.
- `test_config_schema` reports two passing test cases.
- `colcon test-result` reports zero errors and zero failures.

Pass criterion: every test passes. Do not proceed if quaternion, observation
ordering, CRC, or configuration tests fail.

## Level 1: Complete pipeline on loopback with fake state

**Risk:** None, provided `setup_local.sh` is used and the default isolated
topics remain unchanged.

**Goal:** Exercise fake low state → relative observation → policy callback →
safety supervisor → passive/PD command, including lifecycle services,
watchdogs, E-stop, and logging.

The default profile uses `/ws_control/test_lowstate` and
`/ws_control/test_lowcmd`; it cannot address the robot's `/lowcmd`.

### 1. Start the stack

Terminal 1:

```bash
cd /home/fengjun/robot/ws_control
source /home/fengjun/robot/vendor/unitree_ros2/setup_local.sh
source install/setup.bash
ros2 launch go2_nn_control control.launch.py keyboard:=false
```

Expected behavior:

- The supervisor prints `initialized in PASSIVE mode`.
- The policy runner and rosbag recorder remain running.
- No hardware topic named `/lowcmd` is created.

Confirm the topic isolation in Terminal 2:

```bash
cd /home/fengjun/robot/ws_control
source /home/fengjun/robot/vendor/unitree_ros2/setup_local.sh
source install/setup.bash
ros2 topic list | sort
ros2 topic info /ws_control/test_lowcmd
```

Expected behavior: `/ws_control/test_lowcmd` has one publisher. `/lowcmd`
should not appear unless an unrelated process created it.

### 2. Publish a stationary fake robot

Keep this publisher running in Terminal 2:

```bash
ros2 topic pub --rate 500 \
  /ws_control/test_lowstate \
  unitree_go/msg/LowState \
  "{imu_state: {quaternion: [1.0, 0.0, 0.0, 0.0],
                gyroscope: [0.1, 0.2, 0.3]}}"
```

Unspecified fixed-array motor states default to zero, matching the synthetic
ready and neutral positions in `default_experiment.yaml`.

Expected behavior:

- `/ws_control/status` changes `low_state_healthy` to `true`.
- `/ws_control/observation` is published at approximately 250 Hz.
- Before policy start, the relative quaternion is identity and phase is
  `[1, 0]`.

Inspect a single message in Terminal 3:

```bash
cd /home/fengjun/robot/ws_control
source /home/fengjun/robot/vendor/unitree_ros2/setup_local.sh
source install/setup.bash
ros2 topic echo --once /ws_control/status
ros2 topic echo --once /ws_control/observation
ros2 topic hz /ws_control/observation
```

Press `Ctrl-C` after `ros2 topic hz` reports enough samples.

### 3. Exercise the lifecycle

In Terminal 3:

```bash
ros2 service call /ws_control/acknowledge_ownership std_srvs/srv/Trigger "{}"
ros2 service call /ws_control/arm std_srvs/srv/Trigger "{}"
sleep 3
ros2 topic echo --once /ws_control/status
ros2 service call /ws_control/start_policy std_srvs/srv/Trigger "{}"
sleep 1
ros2 topic echo --once /ws_control/status
ros2 topic echo --once /ws_control/policy_action
ros2 topic echo --once /ws_control/applied_command
```

Expected behavior:

- Ownership acknowledgment succeeds in local mode.
- Arm reports `moving to ready stance`.
- After the two-second transition, status is `READY_HOLD`.
- Policy start succeeds and status becomes `POLICY`.
- The placeholder policy commands the measured zero joint positions, zero
  desired velocities, and no feedforward torque.
- Applied Kp/Kd values match the experiment YAML.
- The processed phase is ordered `[cos, sin]` and changes over time.

Stop and recover:

```bash
ros2 service call /ws_control/stop_policy std_srvs/srv/Trigger "{}"
ros2 topic echo --once /ws_control/status
ros2 service call /ws_control/recover std_srvs/srv/Trigger "{}"
sleep 4
ros2 topic echo --once /ws_control/status
```

Expected behavior: states progress through `HOLD_CURRENT`,
`MOVE_TO_NEUTRAL`, and `NEUTRAL_HOLD`.

### 4. Exercise E-stop and reset

```bash
ros2 service call /ws_control/estop std_srvs/srv/Trigger "{}"
ros2 topic echo --once /ws_control/status
ros2 topic echo --once /ws_control/applied_command
ros2 service call /ws_control/arm std_srvs/srv/Trigger "{}"
ros2 service call /ws_control/reset_estop std_srvs/srv/Trigger "{}"
ros2 topic echo --once /ws_control/status
```

Expected behavior:

- E-stop becomes latched and status is `ESTOP`.
- All twelve applied modes are passive and Kp, Kd, and torque are zero.
- Arming while E-stop is latched fails.
- Reset succeeds only while fake low state remains healthy.
- Reset returns to `PASSIVE`; it never resumes policy automatically.

Also test the Boolean E-stop input:

```bash
ros2 topic pub --once /ws_control/estop_request std_msgs/msg/Bool "{data: true}"
```

### 5. Exercise the state watchdog

Repeat the arm and policy-start sequence, then stop the fake low-state
publisher in Terminal 2 with `Ctrl-C`.

Expected behavior: within approximately 20 ms, status enters a latched `ESTOP`
with a low-state watchdog fault and commands become passive.

### 6. Verify the bag

Stop the launch cleanly with `Ctrl-C`, then run:

```bash
cd /home/fengjun/robot/ws_control
latest_bag="$(find bags -mindepth 2 -maxdepth 2 -type d -name bag | sort | tail -n 1)"
ros2 bag info "$latest_bag"
```

Expected behavior: the bag contains test low state/command, observation,
policy action, applied command, supervisor status, parameter events, and ROS
logs. Its parent directory also contains `experiment.yaml` and
`experiment.sha256`.

Level 1 passes when all lifecycle transitions, both E-stop inputs, watchdog
behavior, command contents, and bag topics match the expectations above.

## Level 2: Read-only robot communication

**Risk:** Very low. No controller process may be launched at this level.

**Goal:** Verify the physical network and `/lowstate` contents without creating
any `/lowcmd` publisher.

Prerequisites:

- Robot secured in a safe area.
- Unitree sport mode remains in its normal state.
- No `safety_supervisor`, vendor low-level example, or other command process is
  running.

Run:

```bash
source /home/fengjun/robot/vendor/unitree_ros2/setup.sh
ros2 topic info /lowcmd
ros2 topic hz /lowstate
ros2 topic echo --once /lowstate
```

Expected behavior:

- `/lowstate` arrives near the robot's high-frequency state rate.
- IMU quaternion values are finite with norm near one.
- Twelve motor states contain plausible positions and velocities.
- `/lowcmd` has no publisher from this package.

Pass criterion: stable state reception and plausible measurements. Stop here
if packets are stale, fields are invalid, or another unknown command publisher
exists.

## Level 3: Passive-command hardware test

**Risk:** Low but nonzero. This writes to the real robot command topic.

**Goal:** Verify that the supervisor starts and remains passive, and that
E-stop/reset never produces servo gains or feedforward torque.

Prerequisites:

- Robot fully supported with feet clear of the ground.
- Independent physical torque/power cutoff within reach.
- Unitree sport mode manually disabled.
- Exactly one intended `/lowcmd` publisher.
- A copied experiment YAML with real limits/stances,
  `hardware_profile_complete: true`, and `/lowstate`/`/lowcmd` topics.
- Do not call `arm` during this level.

Review the risk acknowledgment:

```bash
ros2 run go2_nn_control run_risk_test.sh 3 \
  I_ACCEPT_LEVEL_3_HARDWARE_RISK
```

Launch:

```bash
source /home/fengjun/robot/vendor/unitree_ros2/setup.sh
cd /home/fengjun/robot/ws_control
source install/setup.bash
ros2 launch go2_nn_control control.launch.py \
  config:=/absolute/path/to/reviewed_experiment.yaml \
  hardware_mode:=true \
  ownership_ack:=SPORT_MODE_DISABLED \
  keyboard:=false
```

In another configured terminal:

```bash
ros2 topic echo --once /ws_control/status
ros2 topic echo --once /ws_control/applied_command
ros2 service call /ws_control/estop std_srvs/srv/Trigger "{}"
ros2 topic echo --once /ws_control/applied_command
ros2 service call /ws_control/reset_estop std_srvs/srv/Trigger "{}"
```

Expected behavior: status begins in `PASSIVE`; every motor mode is passive;
Kp, Kd, and torque remain zero before, during, and after E-stop/reset.

Immediately use the physical cutoff and stop the launch if any actuator
produces torque or unexpected motion.

## Level 4: Suspended ready and neutral transitions

**Risk:** Medium. PD torque is intentionally enabled.

**Goal:** Validate joint conventions, real stance targets, gains, rate limits,
transition timing, stop behavior, and neutral recovery without ground contact.

Prerequisites: all Level 3 prerequisites and a successful Level 3 test.
Feedforward torque limits must remain zero.

```bash
ros2 run go2_nn_control run_risk_test.sh 4 \
  I_ACCEPT_LEVEL_4_HARDWARE_RISK
```

Start the hardware launch as in Level 3. Then:

```bash
ros2 service call /ws_control/arm std_srvs/srv/Trigger "{}"
ros2 topic echo /ws_control/status
```

Expected behavior:

- Motion follows a smooth minimum-jerk transition toward `ready_position`.
- No leg moves in the wrong direction or uses another leg's target.
- Status reaches `READY_HOLD` only after position and velocity tolerances pass.
- A failed transition remains bounded and eventually latches a timeout fault.

After inspecting the ready stance, test recovery without starting policy:
E-stop and reset, then repeat Level 4 only after correcting configuration.
The normal recovery service is intentionally reachable from `HOLD_CURRENT`,
which is entered by stopping an active policy.

## Level 5: Suspended placeholder-policy test

**Risk:** High. Continuous low-level closed-loop control is active.

**Goal:** Verify ready-to-policy handoff, reference-quaternion capture,
hold-current stop, watchdogs, and neutral recovery on supported hardware.

Prerequisites: successful Levels 3–4, robot suspended, feedforward limits zero.

```bash
ros2 run go2_nn_control run_risk_test.sh 5 \
  I_ACCEPT_LEVEL_5_HARDWARE_RISK
```

Start the hardware launch, then:

```bash
ros2 service call /ws_control/arm std_srvs/srv/Trigger "{}"
# Wait for READY_HOLD and inspect the robot.
ros2 topic echo --once /ws_control/status
ros2 service call /ws_control/start_policy std_srvs/srv/Trigger "{}"
ros2 topic echo --once /ws_control/observation
ros2 topic echo --once /ws_control/policy_action
ros2 service call /ws_control/stop_policy std_srvs/srv/Trigger "{}"
ros2 service call /ws_control/recover std_srvs/srv/Trigger "{}"
```

Expected behavior:

- The first policy observation has a relative quaternion close to identity and
  phase close to `[1, 0]`.
- The hold-current placeholder causes no intentional pose change.
- Normal stop enters `HOLD_CURRENT`.
- Explicit recovery smoothly reaches `NEUTRAL_HOLD`.

For the policy watchdog test, E-stop and support the robot first, then repeat
the sequence and terminate only the `policy_runner` process. The supervisor
must latch E-stop within the configured policy timeout.

## Level 6: User policy on the ground

**Risk:** Critical. A policy or configuration error can cause a fall, violent
motion, or equipment/personnel injury.

**Goal:** Validate the actual learned policy only after it has passed all
software and suspended-hardware tests.

Integrate the policy by constructing `go2_nn_control::PolicyRunner` with a
callable matching `PolicyFunction`. Rebuild and repeat Levels 0, 1, and 5
before any ground test:

```bash
cd /home/fengjun/robot/ws_control
source /home/fengjun/robot/vendor/unitree_ros2/setup_local.sh
colcon build --symlink-install --packages-select go2_nn_control
source install/setup.bash
colcon test --packages-select go2_nn_control
colcon test-result --verbose
```

Before a ground run:

```bash
ros2 run go2_nn_control run_risk_test.sh 6 \
  I_ACCEPT_LEVEL_6_HARDWARE_RISK
```

Required safeguards:

- Clear test area and minimum personnel.
- Operator holding an independent physical torque/power cutoff.
- Conservative experiment-specific gains, target-rate limits, joint limits,
  and initially zero feedforward limits.
- Verified ready and neutral stances.
- Successful bag recording and status monitoring.

Use the same arm/start/stop/recover services as Level 5. The first ground run
should be brief and should test E-stop before attempting locomotion.

Pass/fail criteria must be defined for the particular learned policy before
this level; the generic controller pipeline cannot determine acceptable gait
or balance behavior.
