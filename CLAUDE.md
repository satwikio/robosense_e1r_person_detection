# CLAUDE.md

RoboSense E1R solid-state lidar: stream + visualise the point cloud in RViz, then detect people in the
field of view. ROS 2 Humble, Ubuntu 22.04. See README.md for user-facing setup/usage.

## Layout

- `ros2_ws/src/rslidar_sdk`, `ros2_ws/src/rslidar_msg`: official RoboSense driver, **git submodules, do not
  edit**. Our driver config lives in `e1r_person_detection/config/rslidar_e1r.yaml` and is passed via the
  `config_path` parameter.
- `ros2_ws/src/e1r_person_detection` (ament_python), our code:
  - `detector.py`: pure numpy/sklearn, no ROS. ROI crop → voxel downsample → RANSAC floor → ground-aligned
    frame → remove floor + learned static background → DBSCAN → human-envelope test. `DetectorParams` fields
    are auto-declared as ROS parameters by the node, so adding a field there exposes it in `config/detector.yaml`.
  - `tracker.py`: constant-velocity tracks + Hungarian association on the floor plane.
  - `person_detector_node.py`: ROS wrapper. Publishes `~/markers`, `~/persons`, `~/person_present`, `~/foreground`;
    service `~/relearn_background`.
  - `synthetic.py` + `fake_lidar_node.py`: ray-cast fake E1R scene (floor, walls, pole, table, walking person),
    used by tests and by `launch ... fake:=true`.
- `scripts/check_lidar.py`: no-root check of NIC rx counters + UDP 6699/7788. `scripts/setup_network.sh <iface>`.

## Commands

```bash
cd ros2_ws && source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash
cd src/e1r_person_detection && python3 -m pytest -q test          # unit tests, no ROS needed
ros2 launch e1r_person_detection e1r_person_detection.launch.py fake:=true rviz:=false   # end-to-end without hardware
ros2 launch e1r_person_detection e1r_view.launch.py                # real lidar, raw cloud in RViz
```

## Hard-won facts (don't re-learn these)

- **Subscribe to `/rslidar_points` with RELIABLE QoS.** A cloud is ~0.5 MB; best-effort DDS dropped ~80% of
  frames (2 Hz instead of 10 Hz). RViz displays are set to Reliable for the same reason.
- **rclpy (Humble): assign `msg.data = array.array('B', buf)`, never raw `bytes`**. The bytes setter
  validates element-wise: ~100 ms per cloud vs 0.1 ms.
- `sensor_msgs_py.point_cloud2.create_cloud` is slow for big clouds; build `PointCloud2` manually (see fake_lidar_node).
- Background model: a 0.2 m voxel counts as background if occupied in ≥60% of learning frames. 0.15 m voxels
  caused false positives; a low ratio absorbed people walking through during learning. Anyone *standing still*
  during the first ~3 s becomes background (by design; use the relearn service).
- A person passing a learned-background object (pole) loses points for ~0.7 s; `coast_frames: 8` bridges it.
- Detector assumes a **static** lidar. On a moving platform set `use_background: false` (shape test only).
- Performance: ~35 ms/frame at 10 Hz on an Intel Core Ultra laptop.
- The rslidar_sdk builds fine when libpcap-dev is installed; `-DDISABLE_PCAP_PARSE=ON` does NOT propagate to
  the SDK node target (link errors), so just install libpcap-dev.

## E1R hardware (from the E1R User Guide)

Manual: https://1292444914-files.gitbook.io/~/files/v0/b/gitbook-x-prod.appspot.com/o/spaces%2FtgOfPnscWebs1eh82EKq%2Fuploads%2F2z498roff55q123beY4U%2FE1R%20User%20Guide.pdf?alt=media&token=49031fb7-d0e2-45eb-8408-d10eb0523d77

- Factory network: lidar 192.168.1.200 → host 192.168.1.102, MSOP 6699 (1200-byte packets), DIFOP 7788
  (256-byte), host mask 255.255.0.0. ~260k pts/s, 10 Hz, FOV 120°×90°, range 30 m, blind zone 0.1 m.
- Lidar body speaks only 1000BASE-T1; the **interface box RJ45 supports only gigabit ethernet**.
- Power 9–16 V via DC 5.5-2.1 into the interface box; box green LED steady = power OK.
- State machine: power on → sleep; wakes only when the wake-up pin (KL15) sees 9–16 V.
- Driver `lidar_type: RSE1`.

## Status (2026-10-10)

Software is complete and tested against the synthetic lidar. **No packet has been received from the real E1R
yet.** On the first laptop: two different USB ethernet adapters both linked at **100 Mbps** and received
0 packets (not even ARP); ping 192.168.1.200 and a full 192.168.1.0/24 sweep got no replies. Suspected cause:
the link negotiates 100 Mbps but the interface box is gigabit-only, so check the cable (Cat5e/6, all 4 pairs)
and confirm `/sys/class/net/<iface>/speed` reads 1000. Also check: box LED, supply 9–16 V, lidar plug seated,
lidar actually woken. Setup is being moved to another laptop.

When debugging connectivity: the NIC `rx_packets` counter is the decisive signal. A live E1R produces
thousands of packets/s regardless of IP config, so 0 means a physical/link problem, not an IP problem.
`tcpdump` needs sudo; ask the user to run it with `! sudo tcpdump -i <iface> -n -c 20` if needed.

## Next steps

1. Get packets flowing (above), run `e1r_view.launch.py`, check orientation (x forward, z up) in RViz.
2. Tune `config/detector.yaml` on real data (`show_rejected: true` shows rejected clusters).
3. Possible upgrades: learned detector (e.g. PointPillars) if geometric false positives are a problem.
