#!/bin/bash
#
# setup_ethernet.sh - configure the Raspberry Pi Ethernet port for a direct
# cable connection to the laptop while the laptop keeps WiFi internet.
#
# Network plan:
#   Laptop Ethernet: 192.168.10.1/24, no gateway
#   Raspberry Pi:    192.168.10.2/24, no gateway
#   FrED TCP port:   5005
#
# Usage on the Pi:
#   bash setup_ethernet.sh          # create/update + start the connection
#   bash setup_ethernet.sh status   # show active Ethernet state
#   bash setup_ethernet.sh down     # stop this Ethernet profile

set -euo pipefail

CON_NAME="FrED_Ethernet"
IFACE="eth0"
PI_IP="192.168.10.2"
PC_IP="192.168.10.1"
PORT="5005"

ACTION="${1:-up}"

if ! command -v nmcli >/dev/null 2>&1; then
  cat <<'MSG'
ERROR: nmcli (NetworkManager) was not found.

This script expects Raspberry Pi OS Bookworm with NetworkManager enabled.
Enable it with:
  sudo raspi-config
  Advanced Options -> Network Config -> NetworkManager
then reboot and re-run this script.
MSG
  exit 1
fi

show_status() {
  printf "\n--- FrED Ethernet status ---\n"
  nmcli -t -f NAME,TYPE,DEVICE connection show --active | grep -E "ethernet|${CON_NAME}" || true
  printf "\nThis Pi's IP address(es):\n"
  hostname -I || true
  printf "\nLaptop Ethernet should be: %s/24, no gateway\n" "$PC_IP"
  printf "Laptop app should connect to: %s:%s\n\n" "$PI_IP" "$PORT"
}

case "$ACTION" in
  down|stop)
    printf "Stopping Ethernet profile '%s'...\n" "$CON_NAME"
    sudo nmcli connection down "$CON_NAME" 2>/dev/null || true
    exit 0
    ;;
  status)
    show_status
    exit 0
    ;;
  up|start|"")
    ;;
  *)
    printf "Unknown option '%s'. Use: up | down | status\n" "$ACTION"
    exit 1
    ;;
esac

printf "\n=== Configuring FrED direct Ethernet ===\n"
printf "Pi: %s/24    Laptop: %s/24    Port: %s\n\n" "$PI_IP" "$PC_IP" "$PORT"

if ! nmcli -t -f NAME connection show | grep -qx "$CON_NAME"; then
  printf "Creating Ethernet connection profile '%s'...\n" "$CON_NAME"
  sudo nmcli connection add type ethernet ifname "$IFACE" con-name "$CON_NAME"
fi

# No gateway and no DNS here: this cable is only the FrED private link.
# The laptop's WiFi stays responsible for internet access.
sudo nmcli connection modify "$CON_NAME" \
  ipv4.method manual \
  ipv4.addresses "${PI_IP}/24" \
  ipv4.gateway "" \
  ipv4.dns "" \
  ipv4.never-default yes \
  ipv6.method disabled \
  connection.autoconnect yes

printf "Starting Ethernet profile...\n"
sudo nmcli connection up "$CON_NAME"

show_status

printf "On Windows, set the Ethernet adapter to static IPv4 %s / 255.255.255.0\n" "$PC_IP"
printf "Leave gateway and DNS blank so WiFi internet keeps working.\n\n"
