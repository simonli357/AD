#!/usr/bin/env bash

set -euo pipefail

# Run this on the laptop. USB-C SSH is preferred; current-hotspot SSH is the fallback.

JETSON_USER="${JETSON_USER:-scandy}"
JETSON_PASSWORD="${JETSON_PASSWORD:-alex}"
JETSON_USB_IP="${JETSON_USB_IP:-192.168.55.1}"
JETSON_WIFI_IP="${JETSON_WIFI_IP:-}"
JETSON_WIFI_IP_FALLBACK="${JETSON_WIFI_IP_FALLBACK:-192.168.50.110}"
JETSON_HOTSPOT_IP="${JETSON_HOTSPOT_IP:-10.89.16.119}"
JETSON_REPO="${JETSON_REPO:-/home/${JETSON_USER}/AD}"

WIFI_SSID="${WIFI_SSID:-BoschFMC}"
WIFI_PASSWORD="${WIFI_PASSWORD:-bosch23581321}"

SSH_CONNECT_TIMEOUT="${SSH_CONNECT_TIMEOUT:-7}"
SSH_INITIAL_ATTEMPTS="${SSH_INITIAL_ATTEMPTS:-3}"
SSH_WAIT_ATTEMPTS="${SSH_WAIT_ATTEMPTS:-30}"
SSH_WAIT_SLEEP_SECONDS="${SSH_WAIT_SLEEP_SECONDS:-2}"

SSH_OPTS=(
  -o "StrictHostKeyChecking=accept-new"
  -o "ConnectTimeout=${SSH_CONNECT_TIMEOUT}"
  -o "ServerAliveInterval=5"
  -o "ServerAliveCountMax=3"
)

log() {
  printf '\n== %s\n' "$*"
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    printf 'Missing required command: %s\n' "$1" >&2
    exit 1
  fi
}

shell_quote() {
  printf '%q' "$1"
}

remote_env() {
  printf 'JETSON_PASSWORD=%s JETSON_REPO=%s WIFI_SSID=%s WIFI_PASSWORD=%s ' \
    "$(shell_quote "$JETSON_PASSWORD")" \
    "$(shell_quote "$JETSON_REPO")" \
    "$(shell_quote "$WIFI_SSID")" \
    "$(shell_quote "$WIFI_PASSWORD")"
}

remote_exec() {
  local host="$1"
  SSHPASS="$JETSON_PASSWORD" sshpass -e ssh "${SSH_OPTS[@]}" \
    "${JETSON_USER}@${host}" "$(remote_env) bash -s"
}

remote_wlan_ip() {
  local host="$1"
  remote_exec "$host" <<'REMOTE'
set -euo pipefail
ip -4 -o addr show wlan0 | awk '{ split($4, a, "/"); print a[1]; exit }'
REMOTE
}

wifi_visible_local() {
  local ssid="$1"

  sudo nmcli radio wifi on
  sudo nmcli device wifi rescan >/dev/null 2>&1 || true
  nmcli -g SSID device wifi list | grep -Fxq "$ssid"
}

connect_wifi_local() {
  log "Switching laptop Wi-Fi to ${WIFI_SSID}"

  sudo nmcli radio wifi on

  if nmcli -g NAME connection show | grep -Fxq "$WIFI_SSID"; then
    if ! sudo nmcli connection up "$WIFI_SSID"; then
      sudo nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD" || return 1
    fi
  else
    sudo nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD" || return 1
  fi

  sudo nmcli connection modify "$WIFI_SSID" connection.autoconnect yes || return 1
}

wait_for_ssh() {
  local host="$1"
  local max_attempts="${2:-$SSH_WAIT_ATTEMPTS}"
  local attempt

  for ((attempt = 1; attempt <= max_attempts; attempt++)); do
    if SSHPASS="$JETSON_PASSWORD" sshpass -e ssh "${SSH_OPTS[@]}" \
      "${JETSON_USER}@${host}" true >/dev/null 2>&1; then
      return 0
    fi

    printf 'Waiting for SSH on %s (%d/%d)\n' "$host" "$attempt" "$max_attempts"
    sleep "$SSH_WAIT_SLEEP_SECONDS"
  done

  return 1
}

confirm() {
  local prompt="$1"
  local reply=""

  if [ -r /dev/tty ]; then
    printf '%s [y/N] ' "$prompt" > /dev/tty
    IFS= read -r reply < /dev/tty || reply=""
  else
    printf '%s [y/N] ' "$prompt"
    IFS= read -r reply || reply=""
  fi

  case "$reply" in
    y|Y|yes|YES|Yes) return 0 ;;
    *) return 1 ;;
  esac
}

run_jetson_performance_mode() {
  local host="$1"

  log "Setting Jetson performance mode at ${host}"
  remote_exec "$host" <<'REMOTE'
set -euo pipefail

sudo_cmd() {
  printf '%s\n' "$JETSON_PASSWORD" | sudo -S -p '' "$@"
}

sudo_cmd nvpmodel -m 0
sudo_cmd jetson_clocks
REMOTE
}

open_interactive_ssh() {
  local host="$1"
  local label="$2"

  printf '\nJetson IP: %s\n' "$host"
  log "Opening interactive SSH to Jetson on ${label}"
  exec env SSHPASS="$JETSON_PASSWORD" sshpass -e ssh "${SSH_OPTS[@]}" "${JETSON_USER}@${host}" \
    -t "bash -lc 'cd $(shell_quote "$JETSON_REPO") && source devel/setup.bash && exec bash -i'"
}

need_cmd ssh
need_cmd sshpass
need_cmd nmcli
need_cmd sudo

log "Checking USB-C SSH to Jetson at ${JETSON_USB_IP}"
initial_jetson_host=""
initial_jetson_label=""

if wait_for_ssh "$JETSON_USB_IP" "$SSH_INITIAL_ATTEMPTS"; then
  initial_jetson_host="$JETSON_USB_IP"
  initial_jetson_label="USB-C"
else
  log "USB-C Jetson was not found at ${JETSON_USB_IP}"
  log "Checking current hotspot SSH to Jetson at ${JETSON_HOTSPOT_IP}"

  if wait_for_ssh "$JETSON_HOTSPOT_IP" "$SSH_INITIAL_ATTEMPTS"; then
    if confirm "Found Jetson on current hotspot at ${JETSON_HOTSPOT_IP}. Continue using this connection?"; then
      initial_jetson_host="$JETSON_HOTSPOT_IP"
      initial_jetson_label="current hotspot"
    else
      log "Stopped before running Jetson setup."
      exit 1
    fi
  else
    printf 'Could not reach Jetson over USB-C (%s) or hotspot (%s).\n' "$JETSON_USB_IP" "$JETSON_HOTSPOT_IP" >&2
    exit 1
  fi
fi

log "Using Jetson connection: ${initial_jetson_label} (${initial_jetson_host})"

log "Updating and building Jetson repo over ${initial_jetson_label}"
remote_exec "$initial_jetson_host" <<'REMOTE'
set -euo pipefail

cd "$JETSON_REPO"

if ! command -v catkin_make >/dev/null 2>&1; then
  for setup_file in /opt/ros/*/setup.bash; do
    [ -f "$setup_file" ] || continue
    # catkin_make is often only available after the ROS environment is sourced.
    # shellcheck disable=SC1090
    set +u
    . "$setup_file"
    set -u
    break
  done
fi

git pull
catkin_make
REMOTE

current_jetson_wifi_ip="$(remote_wlan_ip "$initial_jetson_host" || true)"
current_jetson_wifi_ip="${current_jetson_wifi_ip//$'\r'/}"

if [ -z "$current_jetson_wifi_ip" ] && [ "$initial_jetson_label" = "current hotspot" ]; then
  current_jetson_wifi_ip="$initial_jetson_host"
fi

if [ -n "$current_jetson_wifi_ip" ]; then
  log "Current Jetson Wi-Fi IP: ${current_jetson_wifi_ip}"
else
  log "Current Jetson Wi-Fi IP could not be detected"
fi

final_ssh_ip=""
final_ssh_label=""

if ! wifi_visible_local "$WIFI_SSID"; then
  log "${WIFI_SSID} network was not found from the laptop. Staying on the current hotspot."
else
  log "Switching Jetson Wi-Fi to ${WIFI_SSID} over ${initial_jetson_label}"
  jetson_wifi_result="$(
  remote_exec "$initial_jetson_host" <<'REMOTE' || true
set -euo pipefail

sudo_cmd() {
  printf '%s\n' "$JETSON_PASSWORD" | sudo -S -p '' "$@"
}

sudo_cmd nmcli radio wifi on >&2
sudo_cmd nmcli device wifi rescan >/dev/null 2>&1 || true

if ! nmcli -g SSID device wifi list | grep -Fxq "$WIFI_SSID"; then
  echo "NOT_FOUND"
  exit 0
fi

if nmcli -g NAME connection show | grep -Fxq "$WIFI_SSID"; then
  sudo_cmd nmcli connection up "$WIFI_SSID" >&2 || \
    sudo_cmd nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD" >&2
else
  sudo_cmd nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD" >&2
fi

sudo_cmd nmcli connection modify "$WIFI_SSID" connection.autoconnect yes >&2
ip_addr="$(ip -4 -o addr show wlan0 | awk '{ split($4, a, "/"); print a[1]; exit }')"
echo "CONNECTED:${ip_addr}"
REMOTE
)"

  jetson_wifi_result="${jetson_wifi_result//$'\r'/}"

  if [ "$jetson_wifi_result" = "NOT_FOUND" ]; then
    log "${WIFI_SSID} network was not found from the Jetson. Staying on the current hotspot."
  elif [[ "$jetson_wifi_result" == CONNECTED:* ]]; then
    observed_jetson_wifi_ip="${jetson_wifi_result#CONNECTED:}"

    if [ -n "$observed_jetson_wifi_ip" ]; then
      JETSON_WIFI_IP="$observed_jetson_wifi_ip"
    elif [ -z "$JETSON_WIFI_IP" ]; then
      JETSON_WIFI_IP="$JETSON_WIFI_IP_FALLBACK"
    fi

    log "Jetson ${WIFI_SSID} IP: ${JETSON_WIFI_IP}"

    if connect_wifi_local; then
      if wait_for_ssh "$JETSON_WIFI_IP"; then
        final_ssh_ip="$JETSON_WIFI_IP"
        final_ssh_label="$WIFI_SSID"
      else
        log "Could not reach Jetson at ${JETSON_WIFI_IP} on ${WIFI_SSID}. Falling back to ${initial_jetson_label} SSH."
        final_ssh_ip="$initial_jetson_host"
        final_ssh_label="$initial_jetson_label"
      fi
    else
      log "Laptop saw ${WIFI_SSID}, but could not connect to it. Falling back to ${initial_jetson_label} SSH."
      final_ssh_ip="$initial_jetson_host"
      final_ssh_label="$initial_jetson_label"
    fi
  else
    log "Could not switch Jetson to ${WIFI_SSID}. Staying on the current hotspot."
  fi
fi

if [ -z "$final_ssh_ip" ]; then
  if [ -n "$current_jetson_wifi_ip" ] && wait_for_ssh "$current_jetson_wifi_ip"; then
    final_ssh_ip="$current_jetson_wifi_ip"
    final_ssh_label="current hotspot"
  else
    log "Could not reach Jetson on the current hotspot. Falling back to ${initial_jetson_label} SSH."
    final_ssh_ip="$initial_jetson_host"
    final_ssh_label="$initial_jetson_label"
  fi
fi

run_jetson_performance_mode "$final_ssh_ip"
open_interactive_ssh "$final_ssh_ip" "$final_ssh_label"
