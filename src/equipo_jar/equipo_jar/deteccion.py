"""Detection node (`deteccion`): red in the camera + LiDAR distance -> box center in `map`.

ROS wiring only; the math is in vision.py and is tested without ROS.

Inputs (QoS per the rules, docs/QOS.md):
  /cam_1/color/image_raw      Image           best effort   (320x240 @ 30 Hz on the robot)
  /cam_1/color/camera_info    CameraInfo      best effort   (intrinsics K)
  /scan                       LaserScan       best effort   (1080 rays @ ~7 Hz, primary range)
  /cam_1/depth/color/points   PointCloud2     best effort   (fallback range; unreliable on the robot)
  /tf, /tf_static             camera, LiDAR and base frames; map -> odom from localization

Output (team-internal, docs/team/INTERFACES.md lets the perception owners define it):
  /jar/observaciones          geometry_msgs/PoseWithCovarianceStamped, frame `map`
                              one message per box seen in a frame; position = estimated box
                              center; covariance[0] = covariance[7] = sigma^2 of this single
                              observation; header.stamp = time of the LiDAR scan it came from.
  /jar/deteccion/debug        Image with mask and boxes (only if publish_debug)

Per synchronized (image, scan) pair:
  1. red mask + box-shaped contours                                vision.red_mask, find_blobs
  2. camera rays through the blob's left/right edges -> LiDAR frame,
     at the scan time, through `odom` (robot motion between the two
     timestamps is compensated)                                     tf2 lookup_transform_full
  3. LiDAR points inside that wedge, nearest cluster = box face     vision.points_in_wedge, nearest_cluster
     (if none: depth cloud median, if a fresh cloud exists)         vision.robust_3d_point
  4. face -> robot frame -> half a side back = center               vision.center_from_face
  5. center -> `map` at the scan time, publish with sigma(distance) vision.observation_sigma
"""
import numpy as np
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from numpy.lib import recfunctions as rfn
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

import message_filters
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transforms in tf2)
import tf2_ros
from cv_bridge import CvBridge

from equipo_jar import vision
from equipo_jar.comun import correr, parametros_en_vivo

# Live-tunable parameters (section `deteccion` of params.yaml). Read them from self.p at use
# time, never cache them: a change made during a run applies to the next frame.
PARAM_DEFAULTS = {
    # Segmentation (see vision.VisionParams for ranges and reasoning)
    'hsv_red_low_1': [0, 100, 60],
    'hsv_red_high_1': [10, 255, 255],
    'hsv_red_low_2': [170, 100, 60],
    'hsv_red_high_2': [180, 255, 255],
    'morphology_kernel_px': 3,
    'min_area_px': 80,
    'aspect_min': 0.4,
    'aspect_max': 1.8,
    'min_solidity': 0.85,
    # Geometry
    'box_side_m': 0.25,                  # sim box 0.25 x 0.25 x 0.30; real one TO BE CONFIRMED
    'max_area_factor': 3.0,              # 0 disables the area-vs-distance check
    # LiDAR (primary range)
    'wedge_margin_rad': 0.03,
    'lidar_cluster_depth_m': 0.30,
    'lidar_min_points': 3,
    # Cloud (fallback range)
    'use_cloud_fallback': True,
    'cloud_max_age_s': 0.5,
    'erosion_px': 2,
    'z_min_m': 0.6,
    'z_max_m': 4.0,
    'min_3d_points': 15,
    # Observation uncertainty handed to fusion
    'obs_sigma_base_m': 0.05,
    'obs_sigma_per_m': 0.03,
    # Timing and frames
    'processing_rate_hz': 5.0,
    'sync_tolerance_s': 0.1,             # LiDAR period is ~0.14 s; read at startup only
    'robot_frame': 'base_footprint',
    'fixed_frame': 'odom',               # frame used to bridge image time and scan time
    'publish_debug': False,
}


class Detection(Node):
    def __init__(self):
        super().__init__('deteccion')
        self.p = parametros_en_vivo(self, PARAM_DEFAULTS)

        self.bridge = CvBridge()
        self.K = None                       # 3x3 intrinsics from camera_info
        self.last_cloud = None              # latest PointCloud2 (fallback range)
        self.t_last_proc = None

        #PARAM: cache_time = 10 s of TF history. Support: > the latency between capture and
        #       processing (tenths of a second). Plenty.
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Image and scan are processed in pairs with close timestamps. The LiDAR runs at
        # ~7 Hz with ~136 ms latency, so the tolerance is wider than for a 30 Hz camera.
        sub_img = message_filters.Subscriber(
            self, Image, '/cam_1/color/image_raw', qos_profile=qos_profile_sensor_data)
        sub_scan = message_filters.Subscriber(
            self, LaserScan, '/scan', qos_profile=qos_profile_sensor_data)
        #PARAM: queue_size = messages held per topic while waiting for a partner. Support:
        #       >= 1. 10 covers one LiDAR period of 30 Hz images.
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [sub_img, sub_scan], queue_size=10, slop=float(self.p['sync_tolerance_s']))
        self.sync.registerCallback(self.on_pair)
        self.create_subscription(
            CameraInfo, '/cam_1/color/camera_info', self.on_camera_info, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/cam_1/depth/color/points', self.on_cloud, qos_profile_sensor_data)

        # Observations are a stream, not a state: default reliable QoS, no transient local.
        self.pub_obs = self.create_publisher(PoseWithCovarianceStamped, '/jar/observaciones', 10)
        self.pub_debug = None
        if self.p['publish_debug']:
            self.pub_debug = self.create_publisher(Image, '/jar/deteccion/debug', 1)

        self.get_logger().info('Detection ready: red + LiDAR -> /jar/observaciones')

    # ------------------------------------------------------------------ simple callbacks

    def on_camera_info(self, msg):
        """camera_info carries K as a flat row-major list of 9."""
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)

    def on_cloud(self, msg):
        self.last_cloud = msg

    def _vision_params(self):
        """Rebuild the VisionParams dataclass from the live dict (cheap; once per frame)."""
        p = self.p
        return vision.VisionParams(
            hsv_low_1=tuple(int(v) for v in p['hsv_red_low_1']),
            hsv_high_1=tuple(int(v) for v in p['hsv_red_high_1']),
            hsv_low_2=tuple(int(v) for v in p['hsv_red_low_2']),
            hsv_high_2=tuple(int(v) for v in p['hsv_red_high_2']),
            kernel_px=int(p['morphology_kernel_px']), min_area_px=int(p['min_area_px']),
            aspect_min=float(p['aspect_min']), aspect_max=float(p['aspect_max']),
            min_solidity=float(p['min_solidity']), erosion_px=int(p['erosion_px']),
            z_min_m=float(p['z_min_m']), z_max_m=float(p['z_max_m']),
            min_points=int(p['min_3d_points']),
        )

    # ------------------------------------------------------------------ pipeline

    def on_pair(self, img_msg, scan_msg):
        """Main callback: one image and one scan with close timestamps."""
        img_stamp = Time.from_msg(img_msg.header.stamp)
        scan_stamp = Time.from_msg(scan_msg.header.stamp)
        t = scan_stamp.nanoseconds * 1e-9

        # Rate limit: the camera runs at 30 Hz; fusion needs far fewer observations.
        rate = float(self.p['processing_rate_hz'])
        if self.t_last_proc is not None and (t - self.t_last_proc) < 1.0 / rate:
            return
        self.t_last_proc = t
        if self.K is None:
            self.get_logger().warn('no camera_info yet', throttle_duration_sec=5.0)
            return

        vp = self._vision_params()
        bgr = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        mask = vision.red_mask(bgr, vp)
        blobs = vision.find_blobs(mask, vp)
        if self.pub_debug is not None:
            self._publish_debug(bgr, mask, blobs, img_msg.header)
        if not blobs:
            return

        cam_frame = img_msg.header.frame_id
        lidar_frame = scan_msg.header.frame_id
        robot_frame = str(self.p['robot_frame'])

        # Camera origin and ray directions expressed in the LiDAR frame AT THE SCAN TIME.
        # lookup_transform_full bridges the two timestamps through the fixed frame (odom), so
        # the robot's motion between image and scan is compensated.
        tr_cam_to_lidar = self._lookup_full(lidar_frame, scan_stamp, cam_frame, img_stamp)
        if tr_cam_to_lidar is None:
            return
        cam_origin_l = self._apply(tr_cam_to_lidar, np.zeros(3))
        scan_pts = vision.scan_to_points(
            np.asarray(scan_msg.ranges), scan_msg.angle_min, scan_msg.angle_increment,
            scan_msg.range_min, scan_msg.range_max)

        for blob in blobs:
            x, y, w, h = blob.bbox
            v_mid = y + h / 2.0
            # Rays through the left and right edges of the blob, in the LiDAR frame.
            d_left = self._apply(tr_cam_to_lidar, vision.pixel_ray(self.K, x, v_mid)) - cam_origin_l
            d_right = self._apply(tr_cam_to_lidar, vision.pixel_ray(self.K, x + w, v_mid)) - cam_origin_l

            # --- primary: LiDAR points inside the wedge ---
            face_robot = None
            source = 'lidar'
            in_wedge = vision.points_in_wedge(
                scan_pts, cam_origin_l[:2], d_left[:2], d_right[:2], float(self.p['wedge_margin_rad']))
            cluster = vision.nearest_cluster(
                scan_pts[in_wedge], cam_origin_l[:2],
                float(self.p['lidar_cluster_depth_m']), int(self.p['lidar_min_points']))
            if cluster is not None:
                face_xy, _ = cluster
                face_lidar = np.array([face_xy[0], face_xy[1], 0.0])
                face_robot = self._transform(face_lidar, lidar_frame, robot_frame, scan_stamp)

            # --- fallback: depth cloud median ---
            if face_robot is None and self.p['use_cloud_fallback']:
                face_robot = self._face_from_cloud(blob, bgr.shape, vp, robot_frame, scan_stamp)
                source = 'cloud'
            if face_robot is None:
                continue

            distance = float(np.linalg.norm(face_robot[:2]))
            factor = float(self.p['max_area_factor'])
            if factor > 0 and not vision.area_is_consistent(
                    blob.area, distance, self.K[0, 0], self.K[1, 1],
                    float(self.p['box_side_m']), factor):
                self.get_logger().debug(f'dropped: area {blob.area:.0f} px inconsistent at {distance:.2f} m')
                continue

            # Face -> center (half a side back), then to map at the scan time.
            center_robot = vision.center_from_face(face_robot, float(self.p['box_side_m']))
            center_map = self._transform(center_robot, robot_frame, 'map', scan_stamp)
            if center_map is None:
                continue

            sigma = vision.observation_sigma(
                distance, float(self.p['obs_sigma_base_m']), float(self.p['obs_sigma_per_m']))
            self._publish_observation(center_map, sigma, scan_msg.header.stamp)
            self.get_logger().debug(
                f'obs ({source}): ({center_map[0]:.2f}, {center_map[1]:.2f}) d={distance:.2f} sigma={sigma:.3f}')

    def _face_from_cloud(self, blob, shape, vp, robot_frame, stamp):
        """Fallback: median of the cloud points inside the blob, in the robot frame, or None."""
        cloud = self.last_cloud
        if cloud is None:
            return None
        cloud_stamp = Time.from_msg(cloud.header.stamp)
        if abs((cloud_stamp - stamp).nanoseconds) * 1e-9 > float(self.p['cloud_max_age_s']):
            return None
        points = self._cloud_as_image(cloud, shape)
        if points is None:
            return None
        m = vision.blob_mask(blob, shape, vp.erosion_px)
        p_cloud = vision.robust_3d_point(points, m, vp)
        if p_cloud is None:
            return None
        return self._transform(p_cloud, cloud.header.frame_id, robot_frame, cloud_stamp)

    def _cloud_as_image(self, cloud_msg, shape):
        """Cloud as a pixel-indexable (H, W, 3) array: direct reshape if organized, else projected with K."""
        structured = point_cloud2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=False)
        xyz = rfn.structured_to_unstructured(structured).astype(np.float32)
        height, width = shape[:2]
        if cloud_msg.height == height and cloud_msg.width == width:
            return xyz.reshape(height, width, 3)
        return vision.cloud_to_image(xyz.reshape(-1, 3), self.K, shape)

    # ------------------------------------------------------------------ tf helpers

    def _lookup_full(self, target_frame, target_time, source_frame, source_time):
        """Transform between two frames at two different times, bridged through fixed_frame."""
        try:
            #PARAM: timeout = 0.1 s to wait for a late transform. Support: >= 0. Longer blocks
            #       the callback; 0 drops more frames when tf runs just behind.
            return self.tf_buffer.lookup_transform_full(
                target_frame, target_time, source_frame, source_time,
                str(self.p['fixed_frame']), timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'no transform {source_frame} -> {target_frame}: {e}', throttle_duration_sec=5.0)
            return None

    def _transform(self, point, source_frame, target_frame, stamp):
        """Transform a (3,) point between frames at one time. None if tf2 cannot."""
        if source_frame == target_frame:
            return np.asarray(point, dtype=float)
        try:
            tr = self.tf_buffer.lookup_transform(
                target_frame, source_frame, stamp, timeout=Duration(seconds=0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'no transform {source_frame} -> {target_frame}: {e}', throttle_duration_sec=5.0)
            return None
        return self._apply(tr, point)

    @staticmethod
    def _apply(tr, point):
        """Apply a TransformStamped to a (3,) point."""
        ps = PointStamped()
        ps.point.x, ps.point.y, ps.point.z = (float(v) for v in point)
        out = tf2_geometry_msgs.do_transform_point(ps, tr)
        return np.array([out.point.x, out.point.y, out.point.z])

    # ------------------------------------------------------------------ output

    def _publish_observation(self, center_map, sigma, stamp_msg):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(center_map[0])
        msg.pose.pose.position.y = float(center_map[1])
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        # 6x6 row-major covariance; only x (index 0) and y (index 7) carry information.
        cov = [0.0] * 36
        cov[0] = cov[7] = float(sigma) ** 2
        msg.pose.covariance = cov
        self.pub_obs.publish(msg)

    def _publish_debug(self, bgr, mask, blobs, header):
        """Image for RViz: red-tinted mask, green boxes, cyan centroids."""
        import cv2
        dbg = bgr.copy()
        dbg[mask > 0] = (0.5 * dbg[mask > 0] + (0, 0, 127)).astype(np.uint8)
        for b in blobs:
            x, y, w, h = b.bbox
            cv2.rectangle(dbg, (x, y), (x + w, y + h), (0, 255, 0), 1)
            cv2.circle(dbg, (int(b.centroid[0]), int(b.centroid[1])), 3, (255, 255, 0), -1)
        msg = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
        msg.header = header
        self.pub_debug.publish(msg)


def main(args=None):
    correr(Detection, args)
