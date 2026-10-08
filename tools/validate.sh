#!/usr/bin/env bash
# Usage: bash tools/validate.sh offline|synthetic|allocation|live [serial_port]
set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace"
source /opt/ros/humble/setup.bash
source install/setup.bash
mode="${1:-offline}"
if [[ "$mode" == offline ]]; then
  ctest --test-dir build/underwater_mcu_gateway --output-on-failure
  ctest --test-dir build/underwater_devices --output-on-failure
  ctest --test-dir build/underwater_control --output-on-failure
  exit 0
fi
if [[ "$mode" == allocation ]]; then
  ros2 run underwater_bringup serial_control_probe
  exit 0
fi
if [[ "$mode" != synthetic && "$mode" != live ]]; then
  echo "Usage: bash tools/validate.sh offline|synthetic|allocation|live [serial_port]" >&2
  exit 2
fi
export ROS_LOCALHOST_ONLY=1
mkdir -p "log/integration-$mode"
if [[ "$mode" == synthetic ]]; then
  export ROS_DOMAIN_ID=189
  namespace=fixture
  args=(base.launch.xml gateway_enabled:=false namespace:=fixture)
  probe=(-p synthetic:=true)
else
  export ROS_DOMAIN_ID=188
  namespace=underwater_robot
  args=(robot.launch.xml)
  if [[ -n "${2:-}" ]]; then args+=("serial_port:=$2"); fi
  probe=()
fi
env --default-signal=INT ros2 launch underwater_bringup "${args[@]}" \
  </dev/null >"log/integration-$mode/launch.log" 2>&1 &
launch_pid=$!
finish() {
  kill -INT "$launch_pid" 2>/dev/null || true
  for ((n=0;n<50;n++)); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
    sleep 0.1
  done
  if kill -0 "$launch_pid" 2>/dev/null; then kill -TERM "$launch_pid" 2>/dev/null || true; fi
  wait "$launch_pid" || true
}
trap finish EXIT
timeout --signal=INT --kill-after=3s 20s ros2 run underwater_bringup telemetry_probe \
  --ros-args -r "__ns:=/$namespace" "${probe[@]}" | tee "log/integration-$mode/probe.log"
finish
trap - EXIT
cat "log/integration-$mode/launch.log"
