"""Publishes a synthetic E1R-like point cloud on /rslidar_points (for testing without the sensor)."""
import array

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from .synthetic import _rays, render, walking_person


class FakeLidar(Node):
    def __init__(self):
        super().__init__("fake_e1r")
        self.declare_parameter("topic", "/rslidar_points")
        self.declare_parameter("frame_id", "rslidar")
        self.declare_parameter("rate", 10.0)
        self.frame_id = self.get_parameter("frame_id").value
        self.pub = self.create_publisher(PointCloud2, self.get_parameter("topic").value, 10)
        self.rays = _rays()
        self.rng = np.random.default_rng()
        self.t0 = self.get_clock().now()
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        self.create_timer(1.0 / self.get_parameter("rate").value, self.tick)
        self.get_logger().info("Publishing synthetic scene: empty for 4 s, then a person walks across (20 s loop)")

    def tick(self):
        now = self.get_clock().now()
        t = (now - self.t0).nanoseconds * 1e-9
        pts, inten = render(walking_person(t), rng=self.rng, rays=self.rays)
        data = np.column_stack([pts, inten]).astype(np.float32)
        msg = PointCloud2(
            header=Header(stamp=now.to_msg(), frame_id=self.frame_id), height=1, width=len(data),
            fields=self.fields, is_bigendian=False, point_step=16, row_step=16 * len(data),
            data=array.array("B", data.tobytes()), is_dense=True,
        )
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = FakeLidar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
