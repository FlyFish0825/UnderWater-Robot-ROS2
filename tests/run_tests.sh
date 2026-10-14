#!/usr/bin/env bash
# ==============================================================================
# tests/run_tests.sh — 水下机器人测试中枢快速执行脚本
# ==============================================================================
# 用法:
#   bash tests/run_tests.sh [mode]
#
# 模式支持:
#   syntax         - 静态语法与启动 XML 配置检查 (无需 ROS2 环境)
#   offline        - (默认) 运行全量包内单元测试 (无需硬件与网络)
#   remote-manual  - 运行手柄控制状态机与 TCP 9003 ROS2 节点测试
#   network        - 运行 TCP 9000~9003 四大端口端到端通信集成测试
#   all            - 依次运行所有离线测试与网络测试
# ==============================================================================

set -eo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$workspace"

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 自动推导可用的 Python 解释器 (兼容 Linux/Jetson 与本地 Windows 环境)
find_python() {
  if command -v python3 >/dev/null 2>&1 && ! which python3 2>/dev/null | grep -qi "WindowsApps"; then
    echo "python3"
  elif command -v python >/dev/null 2>&1 && ! which python 2>/dev/null | grep -qi "WindowsApps"; then
    echo "python"
  elif [ -f "C:/Users/Administered/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/python.exe" ]; then
    echo "C:/Users/Administered/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/python.exe"
  else
    echo "python"
  fi
}
PYTHON="$(find_python)"

mode="${1:-offline}"

echo -e "${BLUE}================================================================${NC}"
echo -e "${BLUE}   水下机器人测试中枢 (Testing Center) - 运行模式: [${mode}]${NC}"
echo -e "${BLUE}================================================================${NC}"

# 1. 语法检查模式
run_syntax() {
  echo -e "${YELLOW}>>> [1/2] 校验 Python 脚本语法...${NC}"
  "$PYTHON" -m py_compile internal/underwater_bringup/test/remote_manual_launch_test.py
  "$PYTHON" -m py_compile internal/underwater_control/test/remote_manual_ros_test.py
  "$PYTHON" -m py_compile tools/test_tcp_forwarding.py
  "$PYTHON" -m py_compile tools/test_tcp_forwarding_client.py
  echo -e "${GREEN}>>> 脚本语法校验通过 (PASS)${NC}"

  echo -e "${YELLOW}>>> [2/2] 检查 Launch XML 启动参数与安全默认配置...${NC}"
  "$PYTHON" -m unittest internal/underwater_bringup/test/remote_manual_launch_test.py
  echo -e "${GREEN}>>> 启动配置与安全门禁检查通过 (PASS)${NC}"
}

# 2. 离线单元测试模式 (ctest)
run_offline() {
  if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null || true
  fi
  if [ -f "install/setup.bash" ]; then
    source install/setup.bash 2>/dev/null || true
  fi

  if [ ! -d "build" ]; then
    echo -e "${RED}提示: 当前未检测到编译目录 (build/)。${NC}"
    echo -e "${YELLOW}在 Jetson/Linux 环境下请先执行: colcon build${NC}"
    echo -e "${YELLOW}在本地环境可运行语法与静态配置检查: bash tests/run_tests.sh syntax${NC}"
    exit 1
  fi

  echo -e "${YELLOW}>>> [1/3] 执行 underwater_mcu_gateway 单测 (网关/门禁/协议)... ${NC}"
  ctest --test-dir build/underwater_mcu_gateway --output-on-failure

  echo -e "${YELLOW}>>> [2/3] 执行 underwater_devices 单测 (传感器/姿态)... ${NC}"
  ctest --test-dir build/underwater_devices --output-on-failure

  echo -e "${YELLOW}>>> [3/3] 执行 underwater_control 单测 (推力分配/手柄状态机)... ${NC}"
  ctest --test-dir build/underwater_control --output-on-failure

  echo -e "${GREEN}>>> 全量离线单元测试全部通过 (ALL PASS)${NC}"
}

# 3. 手柄业务控制专项测试
run_remote_manual() {
  echo -e "${YELLOW}>>> 运行手柄远程控制专项测试...${NC}"
  if [ -f "build/underwater_control/remote_manual_test" ]; then
    build/underwater_control/remote_manual_test
    echo -e "${GREEN}>>> remote_manual_test (纯状态机与防洪保护) 通过${NC}"
  fi
  if [ -f "build/underwater_control/remote_tcp_test" ]; then
    build/underwater_control/remote_tcp_test
    echo -e "${GREEN}>>> remote_tcp_test (非阻塞网络套接字) 通过${NC}"
  fi
  if [ -f "/opt/ros/humble/setup.bash" ] && [ -f "install/setup.bash" ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source install/setup.bash 2>/dev/null || true
    "$PYTHON" internal/underwater_control/test/remote_manual_ros_test.py
    echo -e "${GREEN}>>> remote_manual_ros_test (ROS2 节点级集成) 通过${NC}"
  fi
  echo -e "${GREEN}>>> 手柄远程控制全链路测试通过 (PASS)${NC}"
}

# 4. 网络通信集成测试
run_network() {
  echo -e "${YELLOW}>>> 运行 TCP 9000~9003 四大端口转发链路测试...${NC}"
  "$PYTHON" tools/test_tcp_forwarding.py
  echo -e "${GREEN}>>> 网络转发测试通过 (PASS)${NC}"
}

case "$mode" in
  syntax)
    run_syntax
    ;;
  offline)
    run_offline
    ;;
  remote-manual)
    run_remote_manual
    ;;
  network)
    run_network
    ;;
  all)
    run_syntax
    run_offline
    run_remote_manual
    run_network
    echo -e "${GREEN}================================================================${NC}"
    echo -e "${GREEN}   所有选定测试全部顺利通过！(ALL TESTS PASSED)${NC}"
    echo -e "${GREEN}================================================================${NC}"
    ;;
  *)
    echo "未知模式: $mode"
    echo "用法: bash tests/run_tests.sh [syntax|offline|remote-manual|network|all]"
    exit 2
    ;;
esac
