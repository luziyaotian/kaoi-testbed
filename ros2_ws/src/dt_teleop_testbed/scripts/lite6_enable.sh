#!/usr/bin/env bash
# lite6_enable.sh — enable Lite 6 for motion.
#
# Runs the three service calls required after the xarm_api driver starts:
#   1. motion_enable all joints
#   2. set mode to 0 (position control)
#   3. set state to 0 (ready to move)
#
# Idempotent — safe to run multiple times. Useful after:
#   - first driver startup each session
#   - recovering from an emergency-stop
#   - changing modes by mistake
#
# Prerequisites:
#   - ros2 env sourced
#   - xarm_api lite6_driver.launch.py already running
#   - /ufactory/motion_enable, /ufactory/set_mode, /ufactory/set_state
#     services are visible (ros2 service list | grep ufactory)

set -e

echo "==> Checking driver is running..."
if ! ros2 service list 2>/dev/null | grep -q '/ufactory/motion_enable'; then
    echo "ERROR: /ufactory/motion_enable service not found." >&2
    echo "  Is the driver running? Try:" >&2
    echo "    ros2 launch xarm_api lite6_driver.launch.py robot_ip:=192.168.1.167" >&2
    exit 1
fi

echo "==> Clearing any prior error state..."
# Attempt to clear errors — this is harmless if none exist.
ros2 service call /ufactory/clean_error xarm_msgs/srv/Call '{}' \
    2>/dev/null || true
ros2 service call /ufactory/clean_warn  xarm_msgs/srv/Call '{}' \
    2>/dev/null || true

echo "==> Enabling all joints..."
ros2 service call /ufactory/motion_enable \
    xarm_msgs/srv/SetInt16ById "{id: 8, data: 1}"

echo "==> Setting position control mode (mode 0)..."
ros2 service call /ufactory/set_mode \
    xarm_msgs/srv/SetInt16 "{data: 0}"

echo "==> Setting ready state (state 0)..."
ros2 service call /ufactory/set_state \
    xarm_msgs/srv/SetInt16 "{data: 0}"

echo "==> Verifying state..."
STATE_RESPONSE=$(ros2 service call /ufactory/get_state \
    xarm_msgs/srv/GetInt16 '{}' 2>/dev/null || true)
if echo "$STATE_RESPONSE" | grep -q 'data=0'; then
    echo "[OK] Lite 6 is enabled, mode=0, state=0, ready for motion"
else
    echo "WARN: Could not confirm state via get_state. Response:"
    echo "$STATE_RESPONSE"
    echo "  Check manually in UFACTORY Studio (http://<robot_ip>:18333)"
fi
