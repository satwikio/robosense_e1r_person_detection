"""ROS 2 node: subscribes to the E1R point cloud and publishes person detections.

Subscribes:  /rslidar_points (sensor_msgs/PointCloud2)
Publishes:   ~/markers         (visualization_msgs/MarkerArray)  boxes + labels for RViz
             ~/persons         (geometry_msgs/PoseArray)         confirmed person positions
             ~/person_present  (std_msgs/Bool)
             ~/foreground      (sensor_msgs/PointCloud2)         points left after ground/background removal
Services:    ~/relearn_background (std_srvs/Trigger)
"""
from dataclasses import fields

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .detector import DetectorParams, PersonDetector
from .tracker import Tracker


def cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    arr = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
    return np.asarray(arr, dtype=np.float64).reshape(-1, 3)


class PersonDetectorNode(Node):
    def __init__(self):
        super().__init__("person_detector")
        self.declare_parameter("input_topic", "/rslidar_points")
        self.declare_parameter("show_rejected", False)
        self.declare_parameter("track_gate", 1.0)
        self.declare_parameter("confirm_frames", 3)
        self.declare_parameter("max_missed_frames", 15)
        self.declare_parameter("coast_frames", 8)
        defaults = DetectorParams()
        kw = {}
        for f in fields(DetectorParams):
            kw[f.name] = self.declare_parameter(f.name, getattr(defaults, f.name)).value
        self.params = DetectorParams(**kw)

        self.det = PersonDetector(self.params)
        self.tracker = Tracker(
            gate=self.get_parameter("track_gate").value,
            confirm_hits=self.get_parameter("confirm_frames").value,
            max_misses=self.get_parameter("max_missed_frames").value,
            coast_frames=self.get_parameter("coast_frames").value,
        )
        self.show_rejected = self.get_parameter("show_rejected").value

        topic = self.get_parameter("input_topic").value
        # Reliable on purpose: a cloud is ~0.5 MB, and best-effort DDS drops most fragmented large messages.
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=2)
        self.sub = self.create_subscription(PointCloud2, topic, self.on_cloud, qos)
        self.pub_markers = self.create_publisher(MarkerArray, "~/markers", 10)
        self.pub_poses = self.create_publisher(PoseArray, "~/persons", 10)
        self.pub_present = self.create_publisher(Bool, "~/person_present", 10)
        self.pub_fg = self.create_publisher(PointCloud2, "~/foreground", 5)
        self.create_service(Trigger, "~/relearn_background", self.on_relearn)

        self.present = False
        self.was_learning = None
        self.frames = 0
        self.proc_time = 0.0
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"Listening on {topic}. Keep the field of view clear while the background is learned.")

    def on_relearn(self, req, resp):
        self.det.reset_background()
        self.tracker.tracks = []
        resp.success = True
        resp.message = "Relearning background - keep the field of view clear"
        self.get_logger().info(resp.message)
        return resp

    def on_cloud(self, msg: PointCloud2):
        t0 = self.get_clock().now()
        pts = cloud_to_xyz(msg)
        res = self.det.process(pts)

        if res.learning != self.was_learning:
            if res.learning:
                self.get_logger().info(f"Learning background ({self.params.background_learn_frames} frames)...")
            elif self.was_learning:
                self.get_logger().info("Background learned. Detecting persons.")
            self.was_learning = res.learning

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        persons = [d for d in res.detections if d.is_person]
        tracks = self.tracker.update(persons, stamp_s)

        self.publish(msg.header, res, tracks)
        self.frames += 1
        self.proc_time += (self.get_clock().now() - t0).nanoseconds * 1e-9

    def publish(self, header, res, tracks):
        ma = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        pa = PoseArray()
        pa.header = header
        for tr in tracks:
            d = tr.det
            q = self.det.box_quaternion(d.yaw_ground)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, d.center)
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
            pa.poses.append(pose)

            box = Marker()
            box.header = header
            box.ns = "person"
            box.id = tr.id
            box.type = Marker.CUBE
            box.pose = pose
            box.scale.x, box.scale.y, box.scale.z = (float(max(d.size[0], 0.4)), float(max(d.size[1], 0.4)), float(d.size[2]))
            box.color.r, box.color.g, box.color.b, box.color.a = 0.1, 1.0, 0.2, 0.45
            ma.markers.append(box)

            txt = Marker()
            txt.header = header
            txt.ns = "label"
            txt.id = tr.id
            txt.type = Marker.TEXT_VIEW_FACING
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = map(float, d.center)
            txt.pose.position.z += float(d.size[2]) / 2 + 0.3
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.3
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = f"person {tr.id}  {d.range:.1f} m"
            ma.markers.append(txt)

        if self.show_rejected:
            for i, d in enumerate(x for x in res.detections if not x.is_person):
                m = Marker()
                m.header = header
                m.ns = "rejected"
                m.id = i
                m.type = Marker.CUBE
                q = self.det.box_quaternion(d.yaw_ground)
                m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, d.center)
                m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = map(float, q)
                m.scale.x, m.scale.y, m.scale.z = (float(v) for v in np.maximum(d.size, 0.05))
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.3, 0.1, 0.25
                ma.markers.append(m)

        self.pub_markers.publish(ma)
        self.pub_poses.publish(pa)

        present = len(tracks) > 0
        self.pub_present.publish(Bool(data=present))
        if present != self.present:
            if present:
                desc = ", ".join(f"#{t.id} at {t.det.range:.1f} m" for t in tracks)
                self.get_logger().warn(f"PERSON DETECTED: {desc}")
            else:
                self.get_logger().info("No person in field of view")
            self.present = present

        if self.pub_fg.get_subscription_count() and res.foreground is not None:
            self.pub_fg.publish(point_cloud2.create_cloud_xyz32(header, res.foreground.astype(np.float32)))

    def report(self):
        if self.frames == 0:
            self.get_logger().warn("No point clouds received yet - is the lidar driver running?")
            return
        self.get_logger().info(
            f"{self.frames / 5.0:.1f} Hz, {1000 * self.proc_time / self.frames:.0f} ms/frame, "
            f"persons: {sum(t.confirmed and t.misses <= self.tracker.coast_frames for t in self.tracker.tracks)}"
        )
        self.frames = 0
        self.proc_time = 0.0


def main():
    rclpy.init()
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
