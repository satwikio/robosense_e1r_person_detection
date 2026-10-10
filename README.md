# RoboSense E1R: visualisation + person detection (ROS 2 Humble)

```
ros2_ws/src/
  rslidar_sdk/            official RoboSense driver (git submodule, supports E1R as "RSE1")
  rslidar_msg/            its message package (git submodule)
  e1r_person_detection/   this project
    config/rslidar_e1r.yaml   driver config (lidar_type RSE1, ports 6699/7788, 30 m max range)
    config/detector.yaml      detector tuning
    launch/e1r_view.launch.py              step 1: driver + RViz
    launch/e1r_person_detection.launch.py  step 2: driver + detector + RViz  (fake:=true for simulation)
scripts/setup_network.sh  gives the PC the E1R host address on an ethernet interface
scripts/check_lidar.py    checks whether lidar packets reach this PC
```

## Setup (Ubuntu 22.04 + ROS 2 Humble)

```bash
git clone --recursive https://github.com/satwikio/robosense_e1r_person_detection.git
cd robosense_e1r_person_detection
sudo apt install libpcap-dev libyaml-cpp-dev python3-sklearn python3-scipy
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

(Forgot `--recursive`? Run `git submodule update --init --recursive`.)

## 0. Network

E1R factory settings (user guide, table 4): lidar **192.168.1.200**, sends to the computer at **192.168.1.102**,
MSOP port **6699**, DIFOP port **7788**, subnet mask 255.255.0.0.

Connection: power adapter → interface box (DC 5.5-2.1, green LED steady = power OK) → lidar;
interface box RJ45 → **gigabit** ethernet adapter → PC. The box supports *only* gigabit: if the link
comes up at 100 Mbps no data flows (usually a bad / 2-pair cable).

```bash
ip -br link                                  # find the ethernet interface name
scripts/setup_network.sh <interface>         # static 192.168.1.102/16, profile "robosense-e1r"
cat /sys/class/net/<interface>/speed         # must be 1000
ping 192.168.1.200
python3 scripts/check_lidar.py               # should report ~2-3k pkt/s on 6699 and the lidar's source IP
```

## 1. Visualise

```bash
ros2 launch e1r_person_detection e1r_view.launch.py
```

RViz opens with fixed frame `rslidar` and shows `/rslidar_points` coloured by intensity.

## 2. Person detection

```bash
ros2 launch e1r_person_detection e1r_person_detection.launch.py           # real lidar
ros2 launch e1r_person_detection e1r_person_detection.launch.py fake:=true # synthetic scene, no hardware
```

**Keep the field of view empty for the first ~3 s.** The detector learns the static background during that time.
After moving the lidar or changing the room, relearn it:

```bash
ros2 service call /person_detector/relearn_background std_srvs/srv/Trigger
```

Outputs:

| topic | type | |
|---|---|---|
| `/person_detector/person_present` | `std_msgs/Bool` | true while anyone is in view |
| `/person_detector/persons` | `geometry_msgs/PoseArray` | person box centres (lidar frame) |
| `/person_detector/markers` | `visualization_msgs/MarkerArray` | green boxes + "person N  X m" labels in RViz |
| `/person_detector/foreground` | `sensor_msgs/PointCloud2` | points left after ground/background removal (debug) |

### How it works

1. Crop to range, 6 cm voxel downsample.
2. RANSAC floor fit, so the lidar can be mounted at any height and tilted (up to 35°).
3. Remove the floor and the learned static background (20 cm voxels, so walls, furniture and poles drop out).
4. DBSCAN clustering.
5. Human envelope test: top 1.0–2.2 m above the floor, ≥0.7 m tall, touching the floor, footprint ≤1.2 × 0.9 m.
6. Tracking: a person is reported after 3 consecutive frames and kept through ≤0.8 s occlusions.

Runs at ~35 ms per frame on this laptop (the E1R runs at 10 Hz).

### Tuning

Set `show_rejected: true` in `config/detector.yaml` to see rejected clusters as orange boxes. Common changes:
- children / sitting people: lower `min_height` (e.g. 0.6) and `min_vertical_extent`
- people only visible from the waist up (lidar mounted low and blocked by a desk): raise `max_bottom`
- **moving** lidar (robot): set `use_background: false`. Shape tests alone then decide, which gives more false positives
  on poles and similar person-sized objects.

### Tests

```bash
cd ros2_ws/src/e1r_person_detection && python3 -m pytest -q test
```
