#!/usr/bin/env bash
# Mock NetworkManager/systemd only; never modifies this computer's networking.
set -euo pipefail
script=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)/robot-static-ip.sh
MOCK_DIR=$(mktemp -d)
export MOCK_DIR
trap 'rm -rf -- "$MOCK_DIR"' EXIT
export UNDERWATER_NETWORK_STATE_DIR="$MOCK_DIR/state"
original=11111111-1111-1111-1111-111111111111
target=22222222-2222-2222-2222-222222222222
export original target
reset_mock() {
  rm -rf -- "$UNDERWATER_NETWORK_STATE_DIR"
  printf '%s' "$original" >"$MOCK_DIR/active"
  printf yes >"$MOCK_DIR/original_auto"
  printf no >"$MOCK_DIR/target_auto"
  : >"$MOCK_DIR/calls"
}
id() { printf '0\n'; }
flock() { :; }
chmod() { :; }
install() {
  if [[ $1 == -d ]]; then mkdir -p -- "$4"
  else cp -- "$3" "$4"; fi
}
ip() {
  if [[ $(cat "$MOCK_DIR/active") == "$target" ]]; then
    printf '2: eth0 inet 192.168.20.70/24 brd 192.168.20.255 scope global eth0\n'
  else
    printf '2: eth0 inet 192.168.20.71/24 brd 192.168.20.255 scope global eth0\n'
  fi
}
arping() { [[ ${MOCK_CONFLICT:-no} == no ]]; }
systemctl() {
  printf 'systemctl %s\n' "$*" >>"$MOCK_DIR/calls"
  # A transient rollback service is not loaded until the timer fires.
  if [[ $* == 'is-active --quiet underwater-network-rollback.service' ]]; then return 3; fi
  if [[ $* == *'stop underwater-network-rollback.timer underwater-network-rollback.service'* ]]; then return 5; fi
}
systemd-run() { printf 'systemd-run %s\n' "$*" >>"$MOCK_DIR/calls"; }
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
    '--wait '* ) printf '%s' "$6" >"$MOCK_DIR/active" ;;
    *) echo "Unhandled mock nmcli: $*" >&2; return 1 ;;
  esac
}
export -f id flock chmod install ip arping systemctl systemd-run nmcli
run() { bash "$script" "$@"; }
reset_mock
run stage eth0 192.168.20.70/24 192.168.20.1 192.168.20.1
[[ $(cat "$MOCK_DIR/active") == "$target" && $(cat "$MOCK_DIR/target_auto") == no ]]
[[ -f $UNDERWATER_NETWORK_STATE_DIR/pending.env ]]
[[ $(cat "$MOCK_DIR/original_auto") == yes ]]
grep -q -- '--on-active=180s' "$MOCK_DIR/calls"
run confirm
[[ $(cat "$MOCK_DIR/target_auto") == yes && $(cat "$MOCK_DIR/original_auto") == no ]]
[[ -f $UNDERWATER_NETWORK_STATE_DIR/confirmed.env && ! -f $UNDERWATER_NETWORK_STATE_DIR/pending.env ]]
reset_mock
run stage eth0 192.168.20.70/24 192.168.20.1 192.168.20.1
run rollback
[[ $(cat "$MOCK_DIR/active") == "$original" && $(cat "$MOCK_DIR/original_auto") == yes ]]
[[ $(cat "$MOCK_DIR/target_auto") == no && -f $UNDERWATER_NETWORK_STATE_DIR/rolled-back.env ]]
reset_mock
if MOCK_CONFLICT=yes run stage eth0 192.168.20.70/24 192.168.20.1 192.168.20.1; then
  echo 'FAIL: address conflict accepted' >&2; exit 1
fi
! grep -q 'connection clone' "$MOCK_DIR/calls"
[[ ! -f $UNDERWATER_NETWORK_STATE_DIR/pending.env ]]
reset_mock
run stage eth0 192.168.20.70/24 192.168.20.1 192.168.20.1
printf '%s' "$original" >"$MOCK_DIR/active"
if run confirm; then echo 'FAIL: inactive static profile confirmed' >&2; exit 1; fi
[[ $(cat "$MOCK_DIR/original_auto") == yes ]]
for pair in '192.168.20.0/24 192.168.20.1' '192.168.20.255/24 192.168.20.1' \
  '192.168.20.70/24 192.168.21.1' '192.168.20.70/33 192.168.20.1' \
  '256.168.20.70/24 192.168.20.1' '192.168.20.70/24 192.168.20.70'; do
  read -r cidr gateway <<<"$pair"
  if run validate "$cidr" "$gateway"; then echo "FAIL: invalid subnet accepted: $pair" >&2; exit 1; fi
done
run validate 192.168.20.70/24 192.168.20.1
printf 'PASS static-address mock checks: staging, confirmation, rollback, conflict, inactive profile, six invalid subnet cases\n'
