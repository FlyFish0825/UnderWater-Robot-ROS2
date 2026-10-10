#!/usr/bin/env bash
# Usage: bash tools/validate.sh offline|synthetic|allocation|imu-calibration|live [serial_port]
# 作用：统一验证入口，把包内单测（ctest）与三个集成探针归并成五种模式。
# 运行方式：先 colcon build 并 source install/setup.bash，再执行上面的 Usage 命令。
# 通过标准：整体退出码 0；offline 由三个 ctest 判定，其余模式由对应探针打印 PASS 并返回 0 判定。
# 是否需要真实硬件：offline/synthetic/allocation/imu-calibration 都不需要；
#   live 需要真机串口在线（可选传 serial_port），且探针只接收、不发送控制指令。
set -eo pipefail # 命令失败即中止（-e）；管道中任一命令失败也算失败（pipefail）
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" # 仓库根目录，不依赖调用者的 cwd
cd "$workspace"
source /opt/ros/humble/setup.bash
source install/setup.bash
mode="${1:-offline}"
if [[ "$mode" == offline ]]; then
  # 三个包的包内测试；任一 ctest 失败即因 -e 直接退出。
  ctest --test-dir build/underwater_mcu_gateway --output-on-failure
  ctest --test-dir build/underwater_devices --output-on-failure
  ctest --test-dir build/underwater_control --output-on-failure
  exit 0
fi
if [[ "$mode" == allocation ]]; then
  # 串口控制集成探针：内部自建伪串口，不需要真硬件。
  ros2 run underwater_bringup serial_control_probe
  exit 0
fi
if [[ "$mode" == imu-calibration ]]; then
  # IMU 标定集成探针：内部自行拉起 devices_node 并合成网关帧。
  ros2 run underwater_bringup imu_calibration_probe
  exit 0
fi
if [[ "$mode" != synthetic && "$mode" != live ]]; then
  echo "Usage: bash tools/validate.sh offline|synthetic|allocation|imu-calibration|live [serial_port]" >&2
  exit 2 # 退出码 2 = 参数用法错误
fi
export ROS_LOCALHOST_ONLY=1
mkdir -p "log/integration-$mode" # launch 与探针日志留在 log/ 下，便于失败后取证
if [[ "$mode" == synthetic ]]; then
  export ROS_DOMAIN_ID=189 # synthetic 固定域 189，与 live(188) 隔离，避免误连真机话题
  namespace=fixture
  args=(base.launch.xml gateway_enabled:=false camera_enabled:=false tcp_port:=0 namespace:=fixture) # 不启网关/相机/TCP，只测内部解析
  probe=(-p synthetic:=true)
else
  export ROS_DOMAIN_ID=188 # live 固定域 188
  namespace=underwater_robot
  args=(robot.launch.xml)
  if [[ -n "${2:-}" ]]; then args+=("serial_port:=$2"); fi # 可选覆盖真机串口路径
  probe=()
fi
env --default-signal=INT ros2 launch underwater_bringup "${args[@]}" \
  </dev/null >"log/integration-$mode/launch.log" 2>&1 & # 后台启动；</dev/null 避免抢占调用者终端输入
launch_pid=$!
# finish：先 SIGINT 优雅停止 launch，最多等 5 秒（50 × 0.1s）；仍存活则 SIGTERM 兜底。
finish() {
  kill -INT "$launch_pid" 2>/dev/null || true
  for ((n=0;n<50;n++)); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then break; fi
    sleep 0.1
  done
  if kill -0 "$launch_pid" 2>/dev/null; then kill -TERM "$launch_pid" 2>/dev/null || true; fi
  wait "$launch_pid" || true # 回收退出码，失败也不阻断清理
}
trap finish EXIT # 探针异常退出时同样要收掉 launch
timeout --signal=INT --kill-after=3s 20s ros2 run underwater_bringup telemetry_probe \
  --ros-args -r "__ns:=/$namespace" "${probe[@]}" | tee "log/integration-$mode/probe.log" # 20s 上限；输出同时落盘
finish
trap - EXIT # finish 已执行，取消 EXIT trap 避免重复清理
cat "log/integration-$mode/launch.log" # 打印 launch 日志，便于对照探针失败原因
