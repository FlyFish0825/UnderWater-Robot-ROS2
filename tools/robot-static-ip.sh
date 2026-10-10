#!/usr/bin/env bash
# Persistent NetworkManager IPv4, with a timed rollback independent of SSH.
# stage IFACE ADDRESS/PREFIX GATEWAY DNS ; confirm ; status ; rollback
# 作用：在机器人上用 NetworkManager 持久化静态 IPv4；改地址后若 180 秒内没有 confirm，
#   由 systemd 定时器自动回滚，避免改错地址后彻底失联（回滚不依赖当前 SSH 会话）。
# 运行方式：必须在机器人上以 root 执行，如 sudo bash tools/robot-static-ip.sh stage eth0 192.168.20.70/24 192.168.20.1 192.168.20.1，
#   重连后 180 秒内再执行 sudo bash <state_dir>/robot-static-ip.sh confirm。
#   另有只读子命令 validate ADDRESS/PREFIX GATEWAY 与 status，不需要 root。
# 通过标准：命令退出码 0；validate 额外打印 "PASS subnet validation"，confirm 打印已持久化提示。
# 是否需要真实硬件：改地址/确认/回滚都需要真实机器人（会改动其网络配置）；validate 为纯计算，无副作用。
set -euo pipefail # -u 很重要：未定义变量（如漏掉 state 字段）应立即报错而不是被当成空值
export LC_ALL=C # 固定 locale，保证 nmcli/ip 输出与脚本里的英文匹配
state_dir=${UNDERWATER_NETWORK_STATE_DIR:-/var/lib/underwater-network} # 状态目录可被测试用环境变量覆盖
timer=underwater-network-rollback
fail() { printf '%s\n' "$*" >&2; exit 1; } # 退出码 1 = 失败，错误信息走 stderr
# ipv4_number：把点分十进制 IPv4 转成 32 位整数；非法输入返回 1 而非退出。
ipv4_number() {
  local value=$1 a b c d tail octet
  IFS=. read -r a b c d tail <<< "$value"
  [[ -z ${tail:-} && -n ${d:-} ]] || return 1 # 必须恰好四段
  for octet in "$a" "$b" "$c" "$d"; do
    [[ $octet =~ ^[0-9]{1,3}$ ]] && ((10#$octet <= 255)) || return 1 # 10# 前缀保证按十进制解析，避免 08/09 报错
  done
  printf '%s\n' "$(((10#$a<<24)|(10#$b<<16)|(10#$c<<8)|10#$d))"
}
# validate_subnet：校验地址与前缀合法、且地址和网关是同一子网内的两个不同可用主机。
validate_subnet() {
  local cidr=$1 gateway=$2 address prefix host gateway_number mask network broadcast
  [[ $cidr == */* ]] || fail 'Address must include a prefix, for example 192.168.20.70/24'
  address=${cidr%/*}; prefix=${cidr##*/}
  [[ $prefix =~ ^[0-9]{1,2}$ ]] && ((10#$prefix >= 1 && 10#$prefix <= 30)) || fail 'Prefix must be 1..30' # /31、/32 无可用主机，故上限 30
  host=$(ipv4_number "$address") || fail 'Invalid IPv4 address'
  gateway_number=$(ipv4_number "$gateway") || fail 'Invalid IPv4 gateway'
  mask=$(((0xffffffff << (32-10#$prefix)) & 0xffffffff))
  network=$((host & mask)); broadcast=$((network | (0xffffffff ^ mask)))
  ((host > network && host < broadcast && gateway_number > network && gateway_number < broadcast && host != gateway_number)) || fail 'Address/gateway must be different usable hosts in the same subnet' # 排除网络号、广播号与地址=网关
}
mode=${1:-status}
if [[ $mode == validate ]]; then
  [[ $# == 3 ]] || fail 'validate ADDRESS/PREFIX GATEWAY'
  validate_subnet "$2" "$3"; printf 'PASS subnet validation\n'; exit 0 # 纯离线校验，供测试脚本与改地址前预检使用
fi
[[ $(id -u) == 0 ]] || fail 'Run with sudo on the robot' # 以下分支都要改网络，必须 root
for command in nmcli systemctl systemd-run ip flock; do command -v "$command" >/dev/null || fail "Missing $command"; done
install -d -m 0700 "$state_dir" # 状态目录仅 root 可访问，内含口令级网络配置
exec 9>"$state_dir/.lock"
flock -x 9 # 全局排他锁：把 stage/confirm/rollback 串行化，包括定时器触发的回滚
state=$state_dir/pending.env
# load_state：读取 stage 阶段写入的待确认状态（iface/original/target/cidr/original_auto）。
load_state() {
  [[ -f $state ]] || fail 'No pending static-address change'
  # The root-only file is produced below using shell-escaped values.
  source "$state"
}
case "$mode" in
stage)
  [[ $# == 5 ]] || fail 'stage IFACE ADDRESS/PREFIX GATEWAY DNS'
  [[ ! -f $state ]] || fail 'A change is already pending; confirm or rollback first' # 不允许叠加两笔待确认改动
  iface=$2; cidr=$3; gateway=$4; dns=$5
  [[ $iface =~ ^[a-zA-Z0-9_.:-]+$ ]] || fail 'Invalid interface'
  validate_subnet "$cidr" "$gateway"
  ipv4_number "$dns" >/dev/null || fail 'DNS must be an IPv4 address'
  [[ $(nmcli -g GENERAL.TYPE device show "$iface") == ethernet ]] || fail 'Only a managed Ethernet interface is supported'
  original=$(nmcli -g GENERAL.CON-UUID device show "$iface") # 记下当前生效的 profile，回滚时用它恢复
  [[ $original =~ ^[0-9a-fA-F-]{36}$ ]] || fail 'Interface has no active NetworkManager profile'
  mac=$(nmcli --escape no -g GENERAL.HWADDR device show "$iface")
  [[ $mac =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]] || fail 'Invalid Ethernet MAC from NetworkManager'
  address=${cidr%/*}
  current=$(ip -4 -o address show dev "$iface")
  if [[ " $current " != *" inet $cidr "* ]]; then
    command -v arping >/dev/null || fail 'Install iputils-arping before changing the address'
    arping -D -I "$iface" -c 3 -w 4 "$address" || fail 'Address conflict or ARP probe failure; no changes made' # 发 3 个 ARP 探测包、最多等 4 秒，冲突则不改动任何配置
  fi
  original_auto=$(nmcli -g connection.autoconnect connection show uuid "$original") # 记住原 profile 的自动连接设置，回滚时还原
  profile="underwater-static-$iface-$(date +%Y%m%d%H%M%S)"
  nmcli connection clone uuid "$original" "$profile" # 克隆而非直接修改原 profile，保证回滚有退路
  target=$(nmcli -g connection.uuid connection show id "$profile")
  # Disable the clone immediately; do not allow a second profile to win at boot.
  nmcli connection modify uuid "$target" connection.autoconnect no
  nmcli connection modify uuid "$target" connection.interface-name "$iface" 802-3-ethernet.mac-address "$mac" \
    ipv4.method manual ipv4.addresses "$cidr" ipv4.gateway "$gateway" \
    ipv4.dns "$dns" ipv4.ignore-auto-dns yes ipv4.may-fail no
  {
    printf 'iface=%q\noriginal=%q\ntarget=%q\ncidr=%q\noriginal_auto=%q\n' "$iface" "$original" "$target" "$cidr" "$original_auto"
  } > "$state" # %q 做 shell 转义，load_state 可安全 source
  chmod 0600 "$state"
  # This copy survives the invoking shell and SSH connection.
  install -m 0700 "${BASH_SOURCE[0]}" "$state_dir/robot-static-ip.sh" # 自拷贝到状态目录，供定时器与断线后执行
  systemctl stop "$timer.timer" "$timer.service" 2>/dev/null || true # 清掉可能残留的旧定时器
  systemctl reset-failed "$timer.service" 2>/dev/null || true
  # 注册 180 秒后触发的回滚定时器；精度 1 秒，验证窗口到期即恢复原 profile。
  systemd-run --unit="$timer" --on-active=180s --timer-property=AccuracySec=1s \
    --setenv="UNDERWATER_NETWORK_STATE_DIR=$state_dir" /bin/bash "$state_dir/robot-static-ip.sh" rollback
  printf 'Reconnect to %s and run sudo bash %s confirm within 180 seconds. Otherwise the original profile is restored.\n' "$address" "$state_dir/robot-static-ip.sh"
  nmcli --wait 20 connection up uuid "$target" ifname "$iface" # 切换连接，最多等 20 秒；此刻 SSH 通常会断开
  ;;
confirm)
  load_state
  [[ $(nmcli -g GENERAL.CON-UUID device show "$iface") == "$target" ]] || fail 'Static profile is not active'
  [[ $(nmcli -g ipv4.method connection show uuid "$target") == manual ]] || fail 'IPv4 is still dynamic'
  [[ " $(ip -4 -o address show dev "$iface") " == *" inet $cidr "* ]] || fail 'Expected address is not assigned'
  # The file lock serialises confirmation with rollback, including manual calls.
  timer_state=$(systemctl show -p LoadState --value "$timer.timer")
  if [[ $timer_state != not-found && -n $timer_state ]]; then systemctl stop "$timer.timer"; fi # 先摘掉定时器，防止确认过程中被回滚
  if systemctl is-active --quiet "$timer.service"; then systemctl stop "$timer.service"; fi
  [[ $(nmcli -g GENERAL.CON-UUID device show "$iface") == "$target" ]] || fail 'Rollback already changed the active profile; stage again' # 二次确认，排除竞态
  nmcli connection modify uuid "$target" connection.autoconnect yes connection.autoconnect-priority 100 # 优先级 100 保证重启后静态 profile 胜出
  nmcli connection modify uuid "$original" connection.autoconnect no # 此时才停用旧 profile
  mv "$state" "$state_dir/confirmed.env" # 状态归档为 confirmed，表示已无待回滚改动
  printf 'Static IPv4 persisted. Verify a new SSH connection and both TCP streams, then verify again after reboot.\n'
  ;;
rollback)
  load_state
  nmcli connection modify uuid "$target" connection.autoconnect no
  nmcli connection modify uuid "$original" connection.autoconnect "$original_auto" # 恢复原 profile 改前的自动连接设置
  nmcli --wait 30 connection up uuid "$original" ifname "$iface"
  mv "$state" "$state_dir/rolled-back.env"
  printf 'Original NetworkManager profile restored\n'
  ;;
status)
  nmcli -f GENERAL.DEVICE,GENERAL.TYPE,GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY device show
  if [[ -f $state ]]; then printf 'Pending change; automatic rollback timer:\n'; systemctl list-timers "$timer.timer" --no-pager; fi # 有待确认改动时顺带显示倒计时
  ;;
*) fail 'Usage: robot-static-ip.sh stage IFACE ADDRESS/PREFIX GATEWAY DNS | confirm | status | rollback' ;;
esac
