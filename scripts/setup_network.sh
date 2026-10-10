#!/usr/bin/env bash
# Give this PC the E1R's factory host address (192.168.1.102, mask 255.255.0.0) on the given
# ethernet interface, via a NetworkManager profile that comes up automatically with the cable.
#   scripts/setup_network.sh <interface>        e.g. enx00e04c5113c0 (see: ip -br link)
set -euo pipefail
IFACE=${1:?usage: $0 <ethernet-interface>   (list them with: ip -br link)}
NAME=robosense-e1r

if nmcli -t -f NAME con show | grep -qx "$NAME"; then
  nmcli con mod "$NAME" connection.interface-name "$IFACE" ipv4.method manual ipv4.addresses 192.168.1.102/16
else
  nmcli con add type ethernet ifname "$IFACE" con-name "$NAME" \
    ipv4.method manual ipv4.addresses 192.168.1.102/16 ipv4.never-default yes \
    ipv6.method disabled connection.autoconnect yes connection.autoconnect-priority 100
fi
nmcli con up "$NAME" || echo "Profile saved; it will activate when the cable has link."
ip -br addr show "$IFACE"
echo "Link speed: $(cat /sys/class/net/$IFACE/speed 2>/dev/null) Mbps (the E1R interface box needs 1000)"
