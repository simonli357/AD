#!/usr/bin/env bash

set -euo pipefail

# Run this on the laptop while the Jetson is connected over USB-C.

JETSON_USER="${JETSON_USER:-scandy}"
JETSON_PASSWORD="${JETSON_PASSWORD:-alex}"
JETSON_USB_IP="${JETSON_USB_IP:-192.168.55.1}"
JETSON_WIFI_IP="${JETSON_WIFI_IP:-}"
JETSON_WIFI_IP_FALLBACK="${JETSON_WIFI_IP_FALLBACK:-192.168.50.110}"
JETSON_REPO="${JETSON_REPO:-/home/${JETSON_USER}/AD}"

WIFI_SSID="${WIFI_SSID:-BoschFMC}"
WIFI_PASSWORD="${WIFI_PASSWORD:-bosch23581321}"

SSH_CONNECT_TIMEOUT="${SSH_CONNECT_TIMEOUT:-10}"
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

connect_wifi_local() {
  log "Switching laptop Wi-Fi to ${WIFI_SSID}"

  sudo nmcli radio wifi on

  if nmcli -g NAME connection show | grep -Fxq "$WIFI_SSID"; then
    sudo nmcli connection up "$WIFI_SSID" || \
      sudo nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD"
  else
    sudo nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD"
  fi

  sudo nmcli connection modify "$WIFI_SSID" connection.autoconnect yes
}

wait_for_ssh() {
  local host="$1"
  local attempt

  for ((attempt = 1; attempt <= SSH_WAIT_ATTEMPTS; attempt++)); do
    if SSHPASS="$JETSON_PASSWORD" sshpass -e ssh "${SSH_OPTS[@]}" \
      "${JETSON_USER}@${host}" true >/dev/null 2>&1; then
      return 0
    fi

    printf 'Waiting for SSH on %s (%d/%d)\n' "$host" "$attempt" "$SSH_WAIT_ATTEMPTS"
    sleep "$SSH_WAIT_SLEEP_SECONDS"
  done

  return 1
}

need_cmd ssh
need_cmd sshpass
need_cmd nmcli
need_cmd sudo

log "Checking USB-C SSH to Jetson at ${JETSON_USB_IP}"
wait_for_ssh "$JETSON_USB_IP"

log "Updating and building Jetson repo over USB-C"
remote_exec "$JETSON_USB_IP" <<'REMOTE'
set -euo pipefail

cd "$JETSON_REPO"

if ! command -v catkin_make >/dev/null 2>&1; then
  for setup_file in /opt/ros/*/setup.bash; do
    [ -f "$setup_file" ] || continue
    # catkin_make is often only available after the ROS environment is sourced.
    # shellcheck disable=SC1090
    . "$setup_file"
    break
  done
fi

git pull
catkin_make
REMOTE

log "Switching Jetson Wi-Fi to ${WIFI_SSID} over USB-C"
observed_jetson_wifi_ip="$(
  remote_exec "$JETSON_USB_IP" <<'REMOTE'
set -euo pipefail

sudo_cmd() {
  printf '%s\n' "$JETSON_PASSWORD" | sudo -S -p '' "$@"
}

sudo_cmd nmcli radio wifi on >&2

if nmcli -g NAME connection show | grep -Fxq "$WIFI_SSID"; then
  sudo_cmd nmcli connection up "$WIFI_SSID" >&2 || \
    sudo_cmd nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD" >&2
else
  sudo_cmd nmcli device wifi connect "$WIFI_SSID" password "$WIFI_PASSWORD" >&2
fi

sudo_cmd nmcli connection modify "$WIFI_SSID" connection.autoconnect yes >&2
ip -4 -o addr show wlan0 | awk '{ split($4, a, "/"); print a[1]; exit }'
REMOTE
)"

observed_jetson_wifi_ip="${observed_jetson_wifi_ip//$'\r'/}"

if [ -n "$observed_jetson_wifi_ip" ]; then
  JETSON_WIFI_IP="$observed_jetson_wifi_ip"
elif [ -z "$JETSON_WIFI_IP" ]; then
  JETSON_WIFI_IP="$JETSON_WIFI_IP_FALLBACK"
fi

log "Jetson BoschFMC IP: ${JETSON_WIFI_IP}"

connect_wifi_local

log "Checking Wi-Fi SSH to Jetson at ${JETSON_WIFI_IP}"
wait_for_ssh "$JETSON_WIFI_IP"

log "Setting Jetson performance mode over Wi-Fi"
remote_exec "$JETSON_WIFI_IP" <<'REMOTE'
set -euo pipefail

sudo_cmd() {
  printf '%s\n' "$JETSON_PASSWORD" | sudo -S -p '' "$@"
}

sudo_cmd nvpmodel -m 0
sudo_cmd jetson_clocks
REMOTE

log "Done"
