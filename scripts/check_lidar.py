#!/usr/bin/env python3
"""Check whether the RoboSense E1R is reaching this PC (no root needed).

  python3 scripts/check_lidar.py [interface]

Reports per-interface received packet counts, then listens on the MSOP/DIFOP ports
and prints the lidar's source IP and the packet sizes it sends.
"""
import collections
import os
import select
import socket
import sys
import time

MSOP_PORT, DIFOP_PORT = 6699, 7788


def rx(iface):
    with open(f"/sys/class/net/{iface}/statistics/rx_packets") as f:
        return int(f.read())


ifaces = sys.argv[1:] or [i for i in os.listdir("/sys/class/net") if i.startswith(("en", "eth"))]
before = {i: rx(i) for i in ifaces}

socks = {}
for port in (MSOP_PORT, DIFOP_PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", port))
        socks[s] = port
    except OSError as e:
        print(f"! cannot bind UDP {port}: {e} (is rslidar_sdk_node already running? stop it first)")

count, sizes, src = collections.Counter(), collections.defaultdict(set), {}
end = time.time() + 3
while time.time() < end and socks:
    ready, _, _ = select.select(list(socks), [], [], 0.2)
    for s in ready:
        data, addr = s.recvfrom(65535)
        p = socks[s]
        count[p] += 1
        sizes[p].add(len(data))
        src[p] = addr[0]

print("Ethernet interfaces (packets received in 3 s):")
for i in ifaces:
    print(f"  {i:20s} {rx(i) - before[i]:7d}")
for p, name in ((MSOP_PORT, "MSOP  (point data)"), (DIFOP_PORT, "DIFOP (device info)")):
    if count[p]:
        print(f"  {name} port {p}: {count[p] / 3:.0f} pkt/s from {src[p]}, sizes {sorted(sizes[p])}")
    else:
        print(f"  {name} port {p}: nothing received")

if count[MSOP_PORT]:
    print("\nOK - lidar data is arriving. Run:  ros2 launch e1r_person_detection e1r_view.launch.py")
elif any(rx(i) - before[i] > 100 for i in ifaces):
    print("\nPackets reach the NIC but not the ports above: the lidar is probably sending to a different "
          "destination IP/port. Give this PC the lidar's destination IP (default 192.168.1.102), e.g.\n"
          "  sudo ip addr add 192.168.1.102/24 dev <interface>")
else:
    print("\nNo traffic at all on the ethernet adapter: check lidar power (it needs ~10-20 s to boot), "
          "the cable / interface box, and that the link LED is lit.")
