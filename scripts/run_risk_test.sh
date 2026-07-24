#!/usr/bin/env bash
set -euo pipefail

level="${1:-}"
ack="${2:-}"

case "$level" in
  0)
    echo "LEVEL 0 — NO HARDWARE RISK"
    echo "Runs C++ unit tests only; no DDS command publisher is started."
    exec colcon test --packages-select go2_nn_control --event-handlers console_direct+
    ;;
  1)
    echo "LEVEL 1 — NO HARDWARE RISK ONLY ON AN ISOLATED ROS DOMAIN"
    echo "Integration testing is not yet connected to robot DDS."
    echo "Use a dedicated ROS_DOMAIN_ID and local CycloneDDS configuration."
    ;;
  2)
    echo "LEVEL 2 — VERY LOW RISK: READ-ONLY ROBOT NETWORK"
    echo "No /lowcmd publisher may be started. Inspect /lowstate only."
    ;;
  3|4|5|6)
    expected="I_ACCEPT_LEVEL_${level}_HARDWARE_RISK"
    echo "LEVEL $level — HARDWARE-AFFECTING MANUAL TEST"
    echo "This repository intentionally does not automate actuator-enabling tests."
    echo "Robot support, an independent physical torque cutoff, and an operator are required."
    if [[ "$ack" != "$expected" ]]; then
      echo "Refusing. Re-run only after review with acknowledgment: $expected" >&2
      exit 2
    fi
    echo "Acknowledgment accepted, but no hardware command was started."
    echo "Follow README.md and invoke the selected launch/service steps manually."
    ;;
  *)
    echo "Usage: $0 LEVEL [I_ACCEPT_LEVEL_N_HARDWARE_RISK]" >&2
    echo "Levels: 0 unit, 1 isolated integration, 2 read-only, 3 passive write,"
    echo "        4 suspended stance, 5 suspended policy, 6 ground policy" >&2
    exit 2
    ;;
esac
