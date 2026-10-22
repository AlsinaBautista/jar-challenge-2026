"""Pure vision and geometry, no ROS: from a BGR image, a LiDAR scan and (optionally) a
point cloud to the 3D position of every visible red box.

Everything here is tested with numpy and OpenCV alone (test/test_vision.py). The node
(deteccion.py) only does the ROS wiring and calls these functions.

Pipeline (LiDAR is the primary range source; the depth cloud is the fallback):

    BGR image ──► red_mask ──► find_blobs ──► pixel_ray (bbox edges) ─┐
                                                                       ▼
    LiDAR scan ──► scan_to_points ──────────────► points_in_wedge ──► nearest_cluster ──► face point
                                                                                              │
    cloud (H,W,3) ──► blob_mask ──► robust_3d_point  (fallback when the LiDAR has nothing)    │
                                                                                              ▼
                                                                                    center_from_face

Why LiDAR first: on the real robot the depth cloud is valid only beyond 0.6 m, has ~38 %
valid pixels and gaps of up to a second (team CLAUDE.md). The LiDAR plane (0.18 m) cuts
the 0.30 m tall boxes, runs at ~7 Hz and has 0.05-12 m range, so it always sees them.

Frame conventions:
  - "camera optical frame": z forward, x right, y down (ROS *_optical_frame).
  - "robot frame" (base_footprint): x forward, y left, z up. center_from_face works here,
    where "horizontal" is the x-y plane with no ambiguity.
  - LiDAR frame: x-y plane of the scan. Mounted with yaw = π on this robot, so ray 0 points
    backwards: never assume, always go through TF.
"""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class VisionParams:
    """Thresholds for segmentation and depth reading.

    Defaults match params.yaml (section `deteccion`). The node rebuilds this dataclass from
    the live parameter dict on every frame, so a value changed during a run applies at once.
    """

    # --- Red ranges in OpenCV HSV --------------------------------------------------------
    # OpenCV uses H in [0, 180), S and V in [0, 255]. Red sits at both ends of the hue
    # circle, so two ranges are needed and the mask is their union. Distractors are blue,
    # yellow or green, never red: color is the main discriminator.
    #PARAM: hsv_low_1 / hsv_high_1 = first red range (H near 0).
    #       Support: H in [0, 180), S and V in [0, 255]. Upper H (10) sets how much orange is
    #       accepted; min S (100) rejects greys and washed-out pinks; min V (60) rejects
    #       near-black. First thing to retune under the venue's lighting.
    hsv_low_1: Tuple[int, int, int] = (0, 100, 60)
    hsv_high_1: Tuple[int, int, int] = (10, 255, 255)
    #PARAM: hsv_low_2 / hsv_high_2 = second red range (H near 180).
    #       Support: same as above. Lower H (170) sets how much magenta is accepted.
    hsv_low_2: Tuple[int, int, int] = (170, 100, 60)
    hsv_high_2: Tuple[int, int, int] = (180, 255, 255)

    # --- Morphological cleanup -----------------------------------------------------------
    #PARAM: kernel_px = structuring element (ellipse) size for opening and closing.
    #       Support: odd integer >= 1; 1 disables. At 320x240 a distant box is a few pixels:
    #       3 or 5 is right, 7 already erases boxes beyond ~3 m.
    kernel_px: int = 3

    # --- Contour shape filters -----------------------------------------------------------
    #PARAM: min_area_px = minimum contour area (pixels) to be considered a box.
    #       Support: integer > 0. At 320x240 with f ≈ 272 px, a 0.25 m box face covers
    #       (272*0.25/Z)^2 px: ~1150 px at 2 m, ~290 px at 4 m. 80 keeps boxes to ~5 m and
    #       drops most stains; raise it if distant false positives appear.
    min_area_px: int = 80
    #PARAM: aspect_min / aspect_max = allowed width/height of the bounding rectangle.
    #       Support: 0 < aspect_min < 1 <= aspect_max. The box is 0.25 wide x 0.30 tall:
    #       head-on gives 0.83; two faces in diagonal give up to ~1.2. [0.4, 1.8] leaves
    #       margin for partial occlusion by walls.
    aspect_min: float = 0.4
    aspect_max: float = 1.8
    #PARAM: min_solidity = minimum area / convex-hull area.
    #       Support: (0, 1]. A box is convex (≈ 1); a stain with notches scores lower.
    #       0.85 tolerates jagged mask edges at low resolution.
    min_solidity: float = 0.85

    # --- Point cloud fallback ------------------------------------------------------------
    #PARAM: erosion_px = radius (pixels) shaved off the blob before reading the cloud.
    #       Support: integer >= 0; 0 disables. Keeps the edge pixels (whose depth jumps to
    #       the wall behind) out of the median. At 320x240, 2 is enough.
    erosion_px: int = 2
    #PARAM: z_min_m / z_max_m = accepted cloud distance range (meters).
    #       Support: 0 < z_min < z_max. The real depth sensor is valid only from 0.6 m.
    z_min_m: float = 0.6
    z_max_m: float = 4.0
    #PARAM: min_points = minimum valid 3D points inside the blob to accept a cloud reading.
    #       Support: integer >= 1. The real cloud has ~38 % valid pixels; 15 is lenient.
    min_points: int = 15


@dataclass(frozen=True)
class Blob:
    """A red contour that passed the shape filters."""
    contour: np.ndarray                  # contour points, cv2.findContours format
    area: float                          # area in pixels (cv2.contourArea)
    bbox: Tuple[int, int, int, int]      # bounding rectangle: x, y, width, height in pixels
    centroid: Tuple[float, float]        # contour center of mass, (u, v) in pixels


# ============================================================================ image


def decode_image(data, height: int, width: int, step: int, encoding: str) -> np.ndarray:
    """sensor_msgs/Image fields -> (H, W, 3) uint8 BGR array, without cv_bridge.

    Only `bgr8` and `rgb8` are handled (the robot camera publishes rgb8 or bgr8). `step` is
    the row length in bytes and may include padding, so rows are cut to width * 3.
    Avoids a dependency on cv_bridge, which is not in the ROS base image.
    """
    if encoding not in ('bgr8', 'rgb8'):
        raise ValueError(f'unsupported image encoding: {encoding}')
    buf = np.frombuffer(data, dtype=np.uint8)
    img = buf.reshape(height, step)[:, :width * 3].reshape(height, width, 3)
    if encoding == 'rgb8':
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


def red_mask(bgr: np.ndarray, p: VisionParams) -> np.ndarray:
    """uint8 mask (0 or 255) of red pixels, cleaned with opening and closing.

        M(u,v) = 1  if  (H <= h1  or  H >= h2)  and  S >= s_min  and  V >= v_min

    HSV separates hue from intensity, so one threshold works in sun and shadow; in RGB
    "red" is an elongated region that shifts with lighting.
    """
    # OpenCV stores images as BGR, not RGB.
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    # inRange returns 255 where the pixel falls inside [low, high] in all three channels.
    m1 = cv2.inRange(hsv, np.array(p.hsv_low_1, np.uint8), np.array(p.hsv_high_1, np.uint8))
    m2 = cv2.inRange(hsv, np.array(p.hsv_low_2, np.uint8), np.array(p.hsv_high_2, np.uint8))
    # Red is "near 0" OR "near 180".
    mask = cv2.bitwise_or(m1, m2)
    # Opening (erode then dilate) removes isolated pixels; closing (dilate then erode)
    # fills holes such as a specular highlight in the middle of the box.
    if p.kernel_px > 1:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p.kernel_px, p.kernel_px))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask


def find_blobs(mask: np.ndarray, p: VisionParams) -> List[Blob]:
    """External contours of the mask that pass the box shape filters.

    Filters, cheapest first: minimum area (noise, very distant things), aspect ratio
    (stripes, cables, painted lines), solidity (irregular stains, two touching objects).
    """
    # RETR_EXTERNAL: outer contours only. CHAIN_APPROX_SIMPLE: straight runs compressed.
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < p.min_area_px:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if h == 0:
            continue
        aspect = w / h
        if not (p.aspect_min <= aspect <= p.aspect_max):
            continue
        hull_area = cv2.contourArea(cv2.convexHull(c))
        if hull_area <= 0 or area / hull_area < p.min_solidity:
            continue
        # Centroid from moments: (m10/m00, m01/m00) = center of mass.
        m = cv2.moments(c)
        if m['m00'] == 0:
            continue
        centroid = (m['m10'] / m['m00'], m['m01'] / m['m00'])
        blobs.append(Blob(contour=c, area=float(area), bbox=(x, y, w, h), centroid=centroid))
    # Largest first: usually the closest and the most reliable.
    blobs.sort(key=lambda b: b.area, reverse=True)
    return blobs


def blob_mask(blob: Blob, shape: Tuple[int, int], erosion_px: int) -> np.ndarray:
    """Boolean mask (H, W) of the blob interior, eroded to drop the edge pixels."""
    m = np.zeros(shape[:2], np.uint8)
    cv2.drawContours(m, [blob.contour], -1, 255, thickness=cv2.FILLED)
    if erosion_px > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * erosion_px + 1, 2 * erosion_px + 1))
        eroded = cv2.erode(m, k)
        # If erosion wiped the whole blob (small box), keep it un-eroded.
        if eroded.any():
            m = eroded
    return m.astype(bool)


# ============================================================================ camera rays


def pixel_ray(K: np.ndarray, u: float, v: float) -> np.ndarray:
    """Unit direction, in the camera optical frame, of the ray through pixel (u, v).

        d = K^-1 [u, v, 1]^T / ||.||        (pinhole model, no distortion)

    K is the 3x3 intrinsic matrix from camera_info (fx = fy ≈ 271.8 px on this robot).
    """
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    d = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
    return d / np.linalg.norm(d)


# ============================================================================ lidar


def scan_to_points(ranges: np.ndarray, angle_min: float, increment: float,
                   range_min: float, range_max: float) -> np.ndarray:
    """Valid LaserScan readings as (N, 2) points in the LiDAR frame.

    theta_i = angle_min + i * increment;  p_i = r_i (cos theta_i, sin theta_i).
    Non-finite readings (~25 % on this LiDAR) and those outside [range_min, range_max] are dropped.
    """
    ranges = np.asarray(ranges, dtype=float)
    if ranges.size == 0:
        return np.zeros((0, 2))
    angles = angle_min + increment * np.arange(ranges.size)
    ok = np.isfinite(ranges) & (ranges >= range_min) & (ranges <= range_max)
    return np.stack([ranges[ok] * np.cos(angles[ok]), ranges[ok] * np.sin(angles[ok])], axis=1)


def points_in_wedge(points_xy: np.ndarray, origin_xy: np.ndarray,
                    dir_left_xy: np.ndarray, dir_right_xy: np.ndarray,
                    margin_rad: float) -> np.ndarray:
    """Boolean mask of the points that lie, as seen from `origin_xy`, between two directions.

    The two directions are the camera rays through the left and right edges of the blob,
    projected onto the LiDAR plane and expressed in the LiDAR frame. The origin is the
    camera position in that frame, so parallax between camera and LiDAR is handled exactly.

        center = normalize(d_left + d_right)
        half   = angle(center, d_left) + margin
        keep   = angle(p - origin, center) <= half

    #PARAM: margin_rad = extra half-angle around the blob. Support: >= 0. One LiDAR step is
            2π/1080 ≈ 0.006 rad; 0.03 rad (~1.7°) absorbs the sync error between camera and
            scan without pulling in the wall next to the box.
    """
    if points_xy.shape[0] == 0:
        return np.zeros(0, dtype=bool)
    dl = dir_left_xy / max(np.linalg.norm(dir_left_xy), 1e-9)
    dr = dir_right_xy / max(np.linalg.norm(dir_right_xy), 1e-9)
    center = dl + dr
    n = np.linalg.norm(center)
    if n < 1e-9:                      # opposite directions: degenerate wedge
        return np.zeros(points_xy.shape[0], dtype=bool)
    center /= n
    half = math.acos(float(np.clip(dl @ center, -1.0, 1.0))) + margin_rad
    v = points_xy - origin_xy
    dist = np.linalg.norm(v, axis=1)
    ok = dist > 1e-6
    cosang = np.zeros(dist.shape)
    cosang[ok] = (v[ok] @ center) / dist[ok]
    ang = np.arccos(np.clip(cosang, -1.0, 1.0))
    return ok & (ang <= half)


def nearest_cluster(points_xy: np.ndarray, origin_xy: np.ndarray, depth_m: float,
                    min_points: int) -> Optional[Tuple[np.ndarray, int]]:
    """Median of the points closest to the origin, within `depth_m` of the nearest one.

    Inside the wedge the box is the closest thing; the wall behind it is farther. Taking
    everything within r_min + depth keeps both visible faces of the box and drops the wall.
    Returns (median_xy, count) or None if there are fewer than `min_points`.

    #PARAM: depth_m = how far behind the nearest point is still "the box". Support: > 0;
            the box side (0.25 m) plus a little: 0.30. Larger starts to include walls the
            box is resting against.
    #PARAM: min_points = minimum LiDAR points on the box. Support: integer >= 1. A 0.25 m
            box at 3 m spans ~14 beams of 1080; with 25 % dropouts ~10 survive. 3 is lenient.
    """
    if points_xy.shape[0] == 0:
        return None
    r = np.linalg.norm(points_xy - origin_xy, axis=1)
    sel = r <= r.min() + depth_m
    if int(sel.sum()) < min_points:
        return None
    return np.median(points_xy[sel], axis=0), int(sel.sum())


# ============================================================================ cloud fallback


def robust_3d_point(points_xyz: np.ndarray, mask: np.ndarray,
                    p: VisionParams) -> Optional[np.ndarray]:
    """Per-coordinate median of the valid cloud points inside the mask, or None.

        P_face = median{ P(u,v) : (u,v) in mask, P finite, z_min <= |P| <= z_max }

    The median tolerates up to half the points being contaminated (edge, wall behind, NaN).
    The norm is used instead of z so the cloud frame convention does not matter.
    """
    pts = points_xyz[mask]
    if pts.size == 0:
        return None
    pts = pts[np.isfinite(pts).all(axis=1)]
    if pts.shape[0] == 0:
        return None
    dist = np.linalg.norm(pts, axis=1)
    pts = pts[(dist >= p.z_min_m) & (dist <= p.z_max_m)]
    if pts.shape[0] < p.min_points:
        return None
    return np.median(pts, axis=0)


def project_points(points: np.ndarray, K: np.ndarray, shape: Tuple[int, int]
                   ) -> Tuple[np.ndarray, np.ndarray]:
    """Pinhole projection of (N, 3) optical-frame points: u = fx X/Z + cx, v = fy Y/Z + cy.

    Returns (valid_indices, uv) for the points inside the image with Z > 0.
    """
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    z = points[:, 2]
    ok = np.isfinite(points).all(axis=1) & (z > 1e-6)
    u = np.full(z.shape, -1.0)
    v = np.full(z.shape, -1.0)
    u[ok] = fx * points[ok, 0] / z[ok] + cx
    v[ok] = fy * points[ok, 1] / z[ok] + cy
    height, width = shape[:2]
    inside = ok & (u >= 0) & (u < width) & (v >= 0) & (v < height)
    uv = np.stack([u[inside], v[inside]], axis=1).astype(int)
    return np.flatnonzero(inside), uv


def cloud_to_image(points: np.ndarray, K: np.ndarray, shape: Tuple[int, int]) -> np.ndarray:
    """Rearrange an unorganized (N, 3) cloud into a pixel-indexable (H, W, 3) array.

    The closest point wins per pixel (z-buffer); empty pixels stay NaN.
    """
    height, width = shape[:2]
    out = np.full((height, width, 3), np.nan, np.float32)
    idx, uv = project_points(points, K, shape)
    if idx.size == 0:
        return out
    order = np.argsort(-points[idx, 2])   # far to near: near overwrites
    out[uv[order, 1], uv[order, 0]] = points[idx[order]]
    return out


# ============================================================================ geometry


def center_from_face(p_face: np.ndarray, side_m: float) -> np.ndarray:
    """Shift the visible-face point half a side backwards, in the horizontal plane.

    p_face is in the robot frame (x forward, y left, z up).
        c = p_face + (L/2) * n,     n = (x, y, 0) / ||(x, y)||

    The sensors see the front surface; the judge scores the geometric center, L/2 further
    back along the line of sight. For L = 0.25 m that is 0.125 m: half the 100-point radius.

    #PARAM: side_m = box side, meters. Support: > 0. Simulator box: 0.25 x 0.25 x 0.30 m.
            TO BE CONFIRMED for the real box (question pending to the organizers).
    """
    p_face = np.asarray(p_face, dtype=float)
    horizontal = p_face[:2]
    norm = np.linalg.norm(horizontal)
    if norm < 1e-6:
        return p_face.copy()
    n = np.array([horizontal[0] / norm, horizontal[1] / norm, 0.0])
    return p_face + 0.5 * side_m * n


def area_is_consistent(area_px: float, distance_m: float, fx: float, fy: float,
                       side_m: float, max_factor: float) -> bool:
    """Check that the apparent size matches a box of side L at that distance.

        A_expected = fx * fy * L^2 / Z^2        (one face seen head-on)

    Accepts if A_expected / factor <= A <= A_expected * factor. Rejects red things with the
    wrong size for their distance: a small label up close, a painted wall far away.

    #PARAM: max_factor = multiplicative tolerance. Support: > 1; 0 disables in the node.
            The face is 0.25 x 0.30 (1.2x a square) and two faces in diagonal add ~1.4x, so
            3 is comfortable; 2 is strict.
    """
    if distance_m <= 0 or side_m <= 0:
        return False
    expected = fx * fy * side_m ** 2 / distance_m ** 2
    if expected <= 0:
        return False
    ratio = area_px / expected
    return (1.0 / max_factor) <= ratio <= max_factor


def observation_sigma(distance_m: float, sigma_base_m: float, sigma_per_m: float) -> float:
    """Standard deviation (m) assigned to one observation, growing with distance.

        sigma = sigma_base + sigma_per_m * Z

    Fusion weighs each observation by 1/sigma^2 and reports the covariance of the mean, so
    these two numbers must be honest about the real error.

    #PARAM: sigma_base_m = error floor. Support: > 0. LiDAR noise plus the face-to-center
            approximation: 0.05 m.
    #PARAM: sigma_per_m = growth per meter. Support: >= 0. Bearing error from pixel
            quantization at 320 px (≈ 0.0035 rad/px) times a few pixels, plus sync error
            while turning: 0.03 m per meter.
    """
    return sigma_base_m + sigma_per_m * max(0.0, distance_m)
