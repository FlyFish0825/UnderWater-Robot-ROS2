#!/usr/bin/env bash
# Persistent NetworkManager IPv4, with a timed rollback independent of SSH.
# stage IFACE ADDRESS/PREFIX GATEWAY DNS ; confirm ; status ; rollback
set -euo pipefail
export LC_ALL=C
state_dir=${UNDERWATER_NETWORK_STATE_DIR:-/var/lib/underwater-network}
timer=underwater-network-rollback
fail() { printf '%s\n' "$*" >&2; exit 1; }
ipv4_number() {
  local value=$1 a b c d tail octet
  IFS=. read -r a b c d tail <<< "$value"
  [[ -z ${tail:-} && -n ${d:-} ]] || return 1
  for octet in "$a" "$b" "$c" "$d"; do
    [[ $octet =~ ^[0-9]{1,3}$ ]] && ((10#$octet <= 255)) || return 1
  done
  printf '%s\n' "$(((10#$a<<24)|(10#$b<<16)|(10#$c<<8)|10#$d))"
}
validate_subnet() {
  local cidr=$1 gateway=$2 address prefix host gateway_number mask network broadcast
  [[ $cidr == */* ]] || fail 'Address must include a prefix, for example 192.168.20.70/24'
  address=${cidr%/*}; prefix=${cidr##*/}
  [[ $prefix =~ ^[0-9]{1,2}$ ]] && ((10#$prefix >= 1 && 10#$prefix <= 30)) || fail 'Prefix must be 1..30'
  host=$(ipv4_number "$address") || fail 'Invalid IPv4 address'
  gateway_number=$(ipv4_number "$gateway") || fail 'Invalid IPv4 gateway'
  mask=$(((0xffffffff << (32-10#$prefix)) & 0xffffffff))
  network=$((host & mask)); broadcast=$((network | (0xffffffff ^ mask)))
  ((host > network && host < broadcast && gateway_number > network && gateway_number < broadcast && host != gateway_number)) || fail 'Address/gateway must be different usable hosts in the same subnet'
}
mode=${1:-status}
if [[ $mode == validate ]]; then
  [[ $# == 3 ]] || fail 'validate ADDRESS/PREFIX GATEWAY'
  validate_subnet "$2" "$3"; printf 'PASS subnet validation\n'; exit 0
fi
[[ $(id -u) == 0 ]] || fail 'Run with sudo on the robot'
for command in nmcli systemctl systemd-run ip flock; do command -v "$command" >/dev/null || fail "Missing $command"; done
install -d -m 0700 "$state_dir"
exec 9>"$state_dir/.lock"
flock -x 9
state=$state_dir/pending.env
load_state() {
  [[ -f $state ]] || fail 'No pending static-address change'
  # The root-only file is produced below using shell-escaped values.
  source "$state"
}
case "$mode" in
stage)
  [[ $# == 5 ]] || fail 'stage IFACE ADDRESS/PREFIX GATEWAY DNS'
  [[ ! -f $state ]] || fail 'A change is already pending; confirm or rollback first'
  iface=$2; cidr=$3; gateway=$4; dns=$5
  [[ $iface =~ ^[a-zA-Z0-9_.:-]+$ ]] || fail 'Invalid interface'
  validate_subnet "$cidr" "$gateway"
  ipv4_number "$dns" >/dev/null || fail 'DNS must be an IPv4 address'
  [[ $(nmcli -g GENERAL.TYPE device show "$iface") == ethernet ]] || fail 'Only a managed Ethernet interface is supported'
  original=$(nmcli -g GENERAL.CON-UUID device show "$iface")
  [[ $original =~ ^[0-9a-fA-F-]{36}$ ]] || fail 'Interface has no active NetworkManager profile'
  mac=$(nmcli --escape no -g GENERAL.HWADDR device show "$iface")
  [[ $mac =~ ^([[:xdigit:]]{2}:){5}[[:xdigit:]]{2}$ ]] || fail 'Invalid Ethernet MAC from NetworkManager'
  address=${cidr%/*}
  current=$(ip -4 -o address show dev "$iface")
  if [[ " $current " != *" inet $cidr "* ]]; then
    command -v arping >/dev/null || fail 'Install iputils-arping before changing the address'
    arping -D -I "$iface" -c 3 -w 4 "$address" || fail 'Address conflict or ARP probe failure; no changes made'
  fi
  original_auto=$(nmcli -g connection.autoconnect connection show uuid "$original")
  profile="underwater-static-$iface-$(date +%Y%m%d%H%M%S)"
  nmcli connection clone uuid "$original" "$profile"
  target=$(nmcli -g connection.uuid connection show id "$profile")
  # Disable the clone immediately; do not allow a second profile to win at boot.
  nmcli connection modify uuid "$target" connection.autoconnect no
  nmcli connection modify uuid "$target" connection.interface-name "$iface" 802-3-ethernet.mac-address "$mac" \
    ipv4.method manual ipv4.addresses "$cidr" ipv4.gateway "$gateway" \
    ipv4.dns "$dns" ipv4.ignore-auto-dns yes ipv4.may-fail no
  {
    printf 'iface=%q\noriginal=%q\ntarget=%q\ncidr=%q\noriginal_auto=%q\n' "$iface" "$original" "$target" "$cidr" "$original_auto"
  } > "$state"
  chmod 0600 "$state"
  # This copy survives the invoking shell and SSH connection.
  install -m 0700 "${BASH_SOURCE[0]}" "$state_dir/robot-static-ip.sh"
  systemctl stop "$timer.timer" "$timer.service" 2>/dev/null || true
  systemctl reset-failed "$timer.service" 2>/dev/null || true
  systemd-run --unit="$timer" --on-active=180s --timer-property=AccuracySec=1s \
    --setenv="UNDERWATER_NETWORK_STATE_DIR=$state_dir" /bin/bash "$state_dir/robot-static-ip.sh" rollback
  printf 'Reconnect to %s and run sudo bash %s confirm within 180 seconds. Otherwise the original profile is restored.\n' "$address" "$state_dir/robot-static-ip.sh"
  nmcli --wait 20 connection up uuid "$target" ifname "$iface"
  ;;
confirm)
  load_state
  [[ $(nmcli -g GENERAL.CON-UUID device show "$iface") == "$target" ]] || fail 'Static profile is not active'
  [[ $(nmcli -g ipv4.method connection show uuid "$target") == manual ]] || fail 'IPv4 is still dynamic'
  [[ " $(ip -4 -o address show dev "$iface") " == *" inet $cidr "* ]] || fail 'Expected address is not assigned'
  # The file lock serialises confirmation with rollback, including manual calls.
  timer_state=$(systemctl show -p LoadState --value "$timer.timer")
  if [[ $timer_state != not-found && -n $timer_state ]]; then systemctl stop "$timer.timer"; fi
  if systemctl is-active --quiet "$timer.service"; then systemctl stop "$timer.service"; fi
  [[ $(nmcli -g GENERAL.CON-UUID device show "$iface") == "$target" ]] || fail 'Rollback already changed the active profile; stage again'
  nmcli connection modify uuid "$target" connection.autoconnect yes connection.autoconnect-priority 100
  nmcli connection modify uuid "$original" connection.autoconnect no
  mv "$state" "$state_dir/confirmed.env"
  printf 'Static IPv4 persisted. Verify a new SSH connection and both TCP streams, then verify again after reboot.\n'
  ;;
rollback)
  load_state
  nmcli connection modify uuid "$target" connection.autoconnect no
  nmcli connection modify uuid "$original" connection.autoconnect "$original_auto"
  nmcli --wait 30 connection up uuid "$original" ifname "$iface"
  mv "$state" "$state_dir/rolled-back.env"
  printf 'Original NetworkManager profile restored\n'
  ;;
status)
  nmcli -f GENERAL.DEVICE,GENERAL.TYPE,GENERAL.CONNECTION,IP4.ADDRESS,IP4.GATEWAY device show
  if [[ -f $state ]]; then printf 'Pending change; automatic rollback timer:\n'; systemctl list-timers "$timer.timer" --no-pager; fi
  ;;
*) fail 'Usage: robot-static-ip.sh stage IFACE ADDRESS/PREFIX GATEWAY DNS | confirm | status | rollback' ;;
esac
