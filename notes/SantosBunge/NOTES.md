# SantosBunge's notes

Module: deteccion and fusion (perception). Who keeps which one is still to be agreed
with Emiliano-Aguilera; this branch implements both so either split works.

Code: `src/equipo_jar/equipo_jar/vision.py` (image, LiDAR and geometry math),
`deteccion.py` (node), `candidate_fusion.py` (fusion math), `fusion.py` (node).
Tunables: sections `deteccion` and `fusion` of `src/equipo_jar/params/params.yaml`.
Every tunable has a `#PARAM:` comment with what it does and its valid range.

One entry per decision: the question, what was tried, what was decided.

## How does detection measure the distance to a box?

- Tried: the depth cloud as the main source (median of the cloud points inside the red
  blob). It works in Gazebo, but the real robot's cloud is valid only from 0.6 m, has
  ~38 % valid pixels and gaps of up to a second (team CLAUDE.md).
- Decided: LiDAR first. The rays through the left and right edges of the red blob are
  taken to the LiDAR frame, the beams inside that wedge are selected, and the nearest
  cluster (everything within 0.30 m of the closest beam) is the box face. The LiDAR
  plane (0.18 m) cuts the 0.30 m tall box, so it always sees it. The cloud is only a
  fallback when the wedge has no beams.

## How are the camera and the LiDAR aligned in time?

- Problem: the camera runs at 30 Hz, the LiDAR at ~7 Hz with ~136 ms latency. While the
  robot turns at 1 rad/s, 100 ms is 10 cm at 1 m.
- Decided: pair image and scan with `ApproximateTimeSynchronizer` (0.1 s), and take the
  camera rays to the LiDAR frame with `lookup_transform_full` through `odom`, which
  applies the robot's motion between the two timestamps. Everything downstream uses the
  scan's timestamp.

## Face or center?

- The sensors see the front face; the judge scores the center, 0.125 m behind it for a
  0.25 m box. Half the 100-point radius.
- Decided: in the robot frame, shift the face point half a side along the line of
  sight (`center_from_face`). `box_side_m` = 0.25 from the simulator; the real size is
  a pending question to the organizers.

## What does detection hand to fusion?

- Options: a new message in `equipo_jar_msgs` (needs an interface PR) or a standard one.
- Decided: `/jar/observaciones` as `geometry_msgs/PoseWithCovarianceStamped` in `map`,
  one per box per frame. Position = center estimate; `covariance[0] = covariance[7] =
  sigma^2`, with `sigma = 0.05 + 0.03 * distance`; stamp = scan time. Fusion looks up
  the robot pose at that stamp itself. No new message type needed.

## What covariance does fusion report?

- Mission (AlsinaBautista) filters on `observaciones >= 5` and on the std along the
  worst direction (largest eigenvalue of `covarianza_xy`) `<= 0.10 m`, and wants every
  candidate, doubtful ones too, with an honest covariance.
- Decided: `cov = S / n + I / W`, where `I / W` is the covariance of the weighted mean
  (weights `1 / sigma^2`) and `S` the weighted sample covariance of the observations
  (so an elongated cloud gives an elongated ellipse). Plus a floor: while the robot poses
  the candidate was seen from span less than 0.3 m, the diagonal is at least 0.15^2.
  Ten frames of a reflection from one spot would otherwise look very precise; the floor
  keeps them above mission's 0.10 m until a second viewpoint confirms them.

## Do candidates ever disappear or merge?

- Decided: two candidates closer than 0.3 m merge and the lower id survives (as
  `Candidata.msg` promises). A candidate with fewer than 5 observations and nothing new
  for 20 s is dropped. Candidates with 5 or more observations are never dropped.

## Parameters

- Decided: all tunables go through `comun.parametros_en_vivo` and are read from the
  dict at use time, so they apply during a live session. Two exceptions, read at
  startup: `sync_tolerance_s` (the synchronizer is built once) and `publish_debug`.
- HSV bounds are lists of 3 (the CI rejects numeric lists of 4 or more).

## Which dependencies?

- Tried: cv_bridge and python3-opencv as rosdep (apt) dependencies. The first run on the
  simulation server died at container start right after the build (batch
  20261022-130550-3826108, `rc docker`, 0 s), most likely because those apt packages are
  not in the server's base image and the install failed.
- Decided: no cv_bridge (images decoded with numpy in `vision.decode_image`, bgr8/rgb8)
  and OpenCV from pip (`opencv-python-headless==4.9.0.80` in `requirements.txt`, the
  path the template documents). The remaining `package.xml` additions
  (tf2_geometry_msgs, sensor_msgs_py, message_filters, numpy) ship with ros-base.

## Robot facts checked in the simulator repo (yahboom_rosmaster @ jar2026-v1, 2026-10-22)

- Camera: `cam_1_color_optical_frame`, 320x240 `rgb8`, fx = fy = 271.809, cx = 158.719,
  cy = 120.092, mounted level (rpy 0 0 0) at x = 0.057 m, z = 0.0376 m above base_link
  (0.107 m above the floor). The macro default pitch of 0.5 rad is not used.
- LiDAR: `laser_link` at x = 0.0435, z = 0.11 m above base_link (0.18 m above the floor),
  yaw = pi. base_link is 0.0714 m above base_footprint.
- Depth cloud: frame `cam_1_depth_frame` (x forward, not an optical frame), unorganized,
  valid from 0.6 m, ~38 % valid points. The fallback therefore moves the cloud into the
  color optical frame with TF before projecting it with K (`_cloud_as_image`).
- Boxes in the practice worlds: 0.25 x 0.25 x 0.30 m, red diffuse (0.8, 0.05, 0.05),
  blue distractors (0.05, 0.05, 0.8). Pure hue, so the HSV ranges hold.

## Next

- Run on the simulation server and compare `/jar/observaciones` against the box
  positions of the practice world (`tools/mundos.py`); tune `obs_sigma_*` so the
  reported covariance matches the real error.
- Confirm with AlsinaBautista that the 0.15 m single-view floor fits mission's policy.

## Testing without the simulator

- `notes/SantosBunge/fake_sensors.py`: publishes static TF, camera_info, a 320x240 image
  with a red rectangle and a 1080-beam scan for a box 2.0 m ahead, then prints the
  observations and candidates it gets back. Run `deteccion` and `fusion` with
  `params.yaml`, then `python3 notes/SantosBunge/fake_sensors.py 12`. Expected center
  (2.0, 0.0); with the robot still, the candidate std stays at the 0.15 m floor.
- Unit tests (no ROS): `src/equipo_jar/test/test_vision.py`, `test_candidate_fusion.py`.
