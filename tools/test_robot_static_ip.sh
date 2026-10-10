#!/usr/bin/env bash
# Mock NetworkManager/systemd only; never modifies this computer's networking.
# 作用：用 shell 函数伪造 nmcli/systemctl/systemd-run/ip/arping，离线回归 robot-static-ip.sh
#   的 stage/confirm/rollback/validate 流程与失败路径（地址冲突、profile 未激活、非法子网）。
# 运行方式：bash tools/test_robot_static_ip.sh（无需 root，仓库内直接执行即可）。
# 通过标准：脚本正常结束并打印 "PASS static-address mock checks: ..."；任一声明为失败的场景被接受即立刻 exit 1。
# 是否需要真实硬件：不需要。全程只操作临时目录里的假状态文件，不触碰本机网络配置。
set -euo pipefail # -e 让断言 [[ ... ]] 失败即中止，因此断言可以直接裸写
script=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/robot-static-ip.sh # 被测脚本（同目录）
MOCK_DIR=$(mktemp -d)
export MOCK_DIR
trap 'rm -rf -- "$MOCK_DIR"' EXIT # 无论成败都清理临时目录
export UNDERWATER_NETWORK_STATE_DIR="$MOCK_DIR/state" # 被测脚本的状态目录重定向到临时目录，避免污染机器
original=11111111-1111-1111-1111-111111111111
target=22222222-2222-2222-2222-222222222222
export original target
# reset_mock：恢复初始假状态——当前生效 profile 为 original、无待确认改动、清空调用记录。
reset_mock() {
  rm -rf -- "$UNDERWATER_NETWORK_STATE_DIR"
  printf '%s' "$original" >"$MOCK_DIR/active"
  printf yes >"$MOCK_DIR/original_auto"
  printf no >"$MOCK_DIR/target_auto"
  : >"$MOCK_DIR/calls"
}
id() { printf '0\n'; } # 始终返回 uid 0，绕过被测脚本的 root 检查
flock() { :; }
chmod() { :; }
install() { # 模拟 -d 建目录与文件拷贝两种用法
  if [[ $1 == -d ]]; then mkdir -p -- "$4"
  else cp -- "$3" "$4"; fi
}
ip() { # 依据 mock 中当前生效 profile 返回不同地址，用于测试“地址是否已生效”的判断
  if [[ $(cat "$MOCK_DIR/active") == "$target" ]]; then
    printf '2: eth0 inet 192.168.20.70/24 brd 192.168.20.255 scope global eth0\n'
  else
    printf '2: eth0 inet 192.168.20.71/24 brd 192.168.20.255 scope global eth0\n'
  fi
}
arping() { [[ ${MOCK_CONFLICT:-no} == no ]]; } # MOCK_CONFLICT=yes 时模拟地址被占用（ARP 探测失败）
# systemctl：记录调用；并按被测脚本的预期返回“定时器/服务尚未加载”等状态码。
systemctl() {
  printf 'systemctl %s\n' "$*" >>"$MOCK_DIR/calls"
  # A transient rollback service is not loaded until the timer fires.
  if [[ $* == 'is-active --quiet underwater-network-rollback.service' ]]; then return 3; fi # 3 = 未激活
  if [[ $* == *'stop underwater-network-rollback.timer underwater-network-rollback.service'* ]]; then return 5; fi # 5 = 服务不存在
}
systemd-run() { printf 'systemd-run %s\n' "$*" >>"$MOCK_DIR/calls"; }
# nmcli：按命令串分发假响应，并记录调用；'connection modify ... connection.autoconnect' 会写回 mock 状态。
nmcli() {
  printf 'nmcli %s\n' "$*" >>"$MOCK_DIR/calls"
  case "$*" in
    '-g GENERAL.TYPE device show eth0') echo ethernet ;;
    '-g GENERAL.CON-UUID device show eth0') cat "$MOCK_DIR/active" ;;
    '--escape no -g GENERAL.HWADDR device show eth0') echo '74:25:54:90:98:84' ;;
    '-g connection.autoconnect connection show uuid '* ) cat "$MOCK_DIR/original_auto" ;;
    '-g connection.uuid connection show id '* ) echo "$target" ;;
    '-g ipv4.method connection show uuid '* ) echo manual ;;
    'connection clone '* ) : ;;
    'connection modify uuid '* )
      local profile=$4
      shift 4
      while (($#)); do
        if [[ $1 == connection.autoconnect ]]; then
          if [[ $profile == "$original" ]]; then printf '%s' "$2" >"$MOCK_DIR/original_auto"
          else printf '%s' "$2" >"$MOCK_DIR/target_auto"; fi
        fi
        shift 2
      done ;;
    '--wait '* ) printf '%s' "$6" >"$MOCK_DIR/active" ;; # connection up 生效后切换 active profile
    *) echo "Unhandled mock nmcli: $*" >&2; return 1 ;; # 未覆盖的调用视为测试失败，避免假通过
  esac
}
export -f id flock chmod install ip arping systemctl systemd-run nmcli # 被测脚本以子进程运行，需要导出这些函数
run() { bash "$script" "$@"; }
# 用例 1：stage 应克隆出目标 profile 但不自动连接，写入 pending.env，并注册 180 秒回滚定时器。
reset_mock
run stage eth0 192.168.20.70/24 192.168.20.1 192.168.20.1
[[ $(cat "$MOCK_DIR/active") == "$target" && $(cat "$MOCK_DIR/target_auto") == no ]]
[[ -f $UNDERWATER_NETWORK_STATE_DIR/pending.env ]]
[[ $(cat "$MOCK_DIR/original_auto") == yes ]] # 未确认前原 profile 的 autoconnect 不得被改动
grep -q -- '--on-active=180s' "$MOCK_DIR/calls"
# 用例 2：confirm 后目标 profile 置为自启、原 profile 停用，状态归档为 confirmed.env。
run confirm
[[ $(cat "$MOCK_DIR/target_auto") == yes && $(cat "$MOCK_DIR/original_auto") == no ]]
[[ -f $UNDERWATER_NETWORK_STATE_DIR/confirmed.env && ! -f $UNDERWATER_NETWORK_STATE_DIR/pending.env ]]
# 用例 3：rollback 应恢复原 profile 生效与自启状态，状态归档为 rolled-back.env。
reset_mock
run stage eth0 192.168.20.70/24 192.168.20.1 192.168.20.1
run rollback
[[ $(cat "$MOCK_DIR/active") == "$original" && $(cat "$MOCK_DIR/original_auto") == yes ]]
[[ $(cat "$MOCK_DIR/target_auto") == no && -f $UNDERWATER_NETWORK_STATE_DIR/rolled-back.env ]]
# 用例 4：ARP 探测发现地址冲突时必须失败，且不得克隆 profile、不得留下待确认状态。
reset_mock
if MOCK_CONFLICT=yes run stage eth0 192.168.20.70/24 192.168.20.1 192.168.20.1; then
  echo 'FAIL: address conflict accepted' >&2; exit 1
fi
! grep -q 'connection clone' "$MOCK_DIR/calls"
[[ ! -f $UNDERWATER_NETWORK_STATE_DIR/pending.env ]]
# 用例 5：静态 profile 实际未生效时 confirm 必须失败，且不改动任何 autoconnect 设置。
reset_mock
run stage eth0 192.168.20.70/24 192.168.20.1 192.168.20.1
printf '%s' "$original" >"$MOCK_DIR/active"
if run confirm; then echo 'FAIL: inactive static profile confirmed' >&2; exit 1; fi
[[ $(cat "$MOCK_DIR/original_auto") == yes ]]
# 用例 6：六组非法子网（网络号/广播号、跨子网网关、/33、超范围八位组、地址=网关）都必须被 validate 拒绝。
for pair in '192.168.20.0/24 192.168.20.1' '192.168.20.255/24 192.168.20.1' \
  '192.168.20.70/24 192.168.21.1' '192.168.20.70/33 192.168.20.1' \
  '256.168.20.70/24 192.168.20.1' '192.168.20.70/24 192.168.20.70'; do
  read -r cidr gateway <<<"$pair"
  if run validate "$cidr" "$gateway"; then echo "FAIL: invalid subnet accepted: $pair" >&2; exit 1; fi
done
# 用例 7：合法子网应通过 validate 并打印 PASS。
run validate 192.168.20.70/24 192.168.20.1
printf 'PASS static-address mock checks: staging, confirmation, rollback, conflict, inactive profile, six invalid subnet cases\n'
