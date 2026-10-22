"""End-to-end check without Gazebo: fake camera, LiDAR and TF for a box 2.0 m ahead.

Publishes: static TF (map->odom->base_footprint->laser_link / cam_1_color_optical_frame),
camera_info (robot intrinsics), a 320x240 image with a red rectangle where the box projects,
and a 1080-beam scan with the box face at 1.875 m straight ahead (laser yaw = pi).
Listens to /jar/observaciones and /jar/candidatas and prints what arrives.
Expected center in map: (2.0, 0.0) = face 1.875 m + half side 0.125 m.
"""
import math
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from tf2_ros import StaticTransformBroadcaster

from equipo_jar_msgs.msg import Candidatas

FX, CX, CY = 271.8, 158.7, 120.0
BOX_X_FACE = 1.875           # face of the box, in base_footprint
BOX_HALF = 0.125
CAM_X, CAM_Z = 0.10, 0.107
LASER_Z = 0.18


def quat_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def tf(parent, child, x, y, z, rpy=(0, 0, 0)):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = float(x), float(y), float(z)
    q = quat_rpy(*rpy)
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = (float(v) for v in q)
    return t


class FakeSensors(Node):
    def __init__(self):
        super().__init__('fake_sensors')
        self.n_obs, self.last_cands = 0, None
        self.obs = []
        self.static = StaticTransformBroadcaster(self)
        self.static.sendTransform([
            tf('map', 'odom', 0, 0, 0),
            tf('odom', 'base_footprint', 0, 0, 0),
            tf('base_footprint', 'laser_link', 0, 0, LASER_Z, (0, 0, math.pi)),
            # optical frame: z forward, x right, y down  <=>  rpy(-pi/2, 0, -pi/2) from base
            tf('base_footprint', 'cam_1_color_optical_frame', CAM_X, 0, CAM_Z, (-math.pi / 2, 0, -math.pi / 2)),
        ])
        self.pub_info = self.create_publisher(CameraInfo, '/cam_1/color/camera_info', qos_profile_sensor_data)
        self.pub_img = self.create_publisher(Image, '/cam_1/color/image_raw', qos_profile_sensor_data)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, '/jar/observaciones', self.on_obs, 10)
        from equipo_jar.comun import QOS_ESTADO
        self.create_subscription(Candidatas, '/jar/candidatas', self.on_cands, QOS_ESTADO)
        self.create_timer(0.1, self.tick)

    def on_obs(self, m):
        self.n_obs += 1
        self.obs.append((m.pose.pose.position.x, m.pose.pose.position.y, math.sqrt(m.pose.covariance[0])))

    def on_cands(self, m):
        self.last_cands = m

    def tick(self):
        now = self.get_clock().now().to_msg()
        info = CameraInfo()
        info.header.stamp, info.header.frame_id = now, 'cam_1_color_optical_frame'
        info.width, info.height = 320, 240
        info.k = [FX, 0.0, CX, 0.0, FX, CY, 0.0, 0.0, 1.0]
        self.pub_info.publish(info)

        # Box face center relative to the camera, in optical coords.
        dx = BOX_X_FACE - CAM_X
        z_center = 0.15                              # box center height (0.30 tall)
        u = CX
        v = CY + FX * (-(z_center - CAM_Z)) / dx
        w_px = int(round(FX * 2 * BOX_HALF / dx))     # 0.25 m wide
        h_px = int(round(FX * 0.30 / dx))             # 0.30 m tall
        img = np.full((240, 320, 3), (170, 170, 170), np.uint8)
        x0, y0 = int(u - w_px / 2), int(v - h_px / 2)
        img[max(0, y0):y0 + h_px, max(0, x0):x0 + w_px] = (30, 30, 200)   # BGR red
        msg = Image()
        msg.header.stamp, msg.header.frame_id = now, 'cam_1_color_optical_frame'
        msg.height, msg.width, msg.encoding, msg.step = 240, 320, 'bgr8', 960
        msg.data = img.tobytes()
        self.pub_img.publish(msg)

        # Scan in laser_link (yaw pi): the box ahead of the robot is at bearing pi.
        n = 1080
        inc = 2 * math.pi / n
        angles = -math.pi + inc * np.arange(n)
        ranges = np.full(n, 4.0)
        d = (angles - math.pi + math.pi) % (2 * math.pi) - math.pi      # offset from bearing pi
        half = math.atan(BOX_HALF / BOX_X_FACE)
        sel = np.abs(d) <= half
        ranges[sel] = BOX_X_FACE / np.cos(d[sel])
        ranges[::4] = float('inf')                                       # ~25 % dropouts
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = now, 'laser_link'
        scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi - inc, inc
        scan.range_min, scan.range_max = 0.05, 12.0
        scan.ranges = ranges.tolist()
        self.pub_scan.publish(scan)


def main():
    rclpy.init()
    node = FakeSensors()
    end = node.get_clock().now().nanoseconds + int(float(sys.argv[1]) * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f'observations received: {node.n_obs}')
    if node.obs:
        xs = np.array(node.obs)
        print(f'  mean center ({xs[:, 0].mean():.3f}, {xs[:, 1].mean():.3f}) sigma {xs[:, 2].mean():.3f}  expected (2.000, 0.000)')
    if node.last_cands is not None:
        for c in node.last_cands.candidatas:
            cov = c.covarianza_xy
            print(f'  candidate id={c.id} center=({c.centro.x:.3f}, {c.centro.y:.3f}) n={c.observaciones} '
                  f'cov_diag=({cov[0]:.4f}, {cov[3]:.4f}) -> max std {math.sqrt(max(cov[0], cov[3])):.3f}')
    else:
        print('  no /jar/candidatas received')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
