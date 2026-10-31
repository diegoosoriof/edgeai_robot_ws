# RGB-D robot surface tracking experiment

This package offers two configurable surface detectors on a common flat floor:
`green_seeds` (the original/default RGB + depth method) and `geometry` (native depth
only). Both reuse the same single-target tracker, timestamped observer TF and outputs.
There are no target pose,
Gazebo identity, segmentation, fiducial or learned-model inputs. `target_ns` selects
the suggested keyboard-control command; it is **not** a detector input or an identity
guarantee. Plausible green objects can fool the original method; similarly sized
non-robot objects can fool geometry mode. The output is a visible-surface centroid,
not the robot's base centre or verified identity.

## Build and launch

From the workspace root:

```bash
pixi install
pixi run colcon build --symlink-install --packages-up-to edgeai_robot_tracking
pixi shell
source install/setup.bash
```

If the two-robot simulation is already running:

```bash
ros2 launch edgeai_robot_tracking detection_tracking.launch.py
```

Select either method explicitly with the same launcher:

```bash
ros2 launch edgeai_robot_tracking detection_tracking.launch.py \
  observer_ns:=robot1 target_ns:=robot2 detection_method:=green_seeds
```

```bash
ros2 launch edgeai_robot_tracking detection_tracking.launch.py \
  observer_ns:=robot1 target_ns:=robot2 detection_method:=geometry
```

`detection_method` is a validated startup setting; unsupported values fail clearly.
Runtime switching is not implemented. Compare sequentially: stop the detector with
Ctrl+C before launching the other method. Running both with the default namespace
would mix positions, diagnostics and marker deletions; simultaneous comparison is
not a supported launcher workflow. Keep the simulation running between detector runs.

Or start the **existing** multi-robot simulation and detector together:

```bash
ros2 launch edgeai_robot_tracking detection_tracking.launch.py \
  start_simulation:=true num_robots:=2 observer_ns:=robot1 target_ns:=robot2 \
  detection_method:=geometry
```

Use `detection_method:=green_seeds` in the same command for the original method.

Do not start a second simulation on top of the first. `start_simulation` defaults to
`false`. The experiment includes `edgeai_robot_description/multi_robot_sim.launch.py`;
it does not duplicate the world or bridge setup. The original single-robot and
multi-robot launches do not start this detector. `gui:=false` optionally runs the
included simulation server with headless sensor rendering and omits RViz; the
default `gui:=true` keeps normal graphical behavior. Headless rendering still needs
a working EGL/OGRE2 driver.

Keyboard control is independent of perception. Open **another interactive terminal**:

```bash
cd /path/to/edgeai_robot_ws
pixi shell
source install/setup.bash
ros2 run edgeai_robot_tracking keyboard_teleop --namespace robot2 \
  --linear-speed 0.15 --angular-speed 0.6 --input-timeout 0.25
```

| Key | Action |
| --- | --- |
| `w` / `s` | Forward / backward |
| `a` / `d` | Turn left / right in place |
| `j` / `l` | Forward left / right arc |
| Space or `x` | Stop |
| `q` or Ctrl+C | Stop and quit |

Repeat keys to keep moving. The wall-time watchdog sends zeros after 0.25 seconds
without input, including while simulation time is paused. A desktop keyboard's
initial repeat delay can cause a brief stop; repeated taps work, or adjust
`--input-timeout` deliberately. Zeros are sent at startup, on quit, Ctrl+C and SIGTERM;
terminal settings are restored in `finally`. Force-killing a process cannot run its
cleanup. `--rate` defaults to 20 wall Hz. `--topic /robot2/cmd_vel` overrides namespace
resolution. Commands use **geometry_msgs/msg/Twist**, matching this branch's DiffDrive
bridge, rather than TwistStamped.

There is no command mux or navigation controller in `multi_robot_sim.launch.py`.
Stop other teleop, `ros2 topic pub`, exploration or Nav2 processes publishing to the
same `cmd_vel` before this test. Inspect with:

```bash
ros2 topic info /robot2/cmd_vel --verbose
```

`enable_detector:=false` disables perception without changing the optional simulation
or separately running keyboard control. The launcher never assumes it owns a TTY.
`target_ns` only changes the printed teleop command; pass that namespace to the separate
keyboard script. Run `--help` on the script or `--show-args` on the launch command for
all options.

## Camera and frame contract

Inspected workspace: ROS 2 Jazzy under Pixi; Gazebo Harmonic, gz-sensors 8.2.2 and
ros_gz_bridge. For observer `robot1`:

| Input | Topic | Representation |
| --- | --- | --- |
| RGB (green-seed mode only) | `/robot1/camera/image` | RGB8 from Gazebo; converted to BGR8 for OpenCV |
| Depth | `/robot1/camera/depth_image` | 32FC1 **metres**, forward-axis depth |
| RGB and depth calibration | `/robot1/camera/camera_info` | Runtime K/D and image dimensions |
| Mounting/observer motion | `/tf`, `/tf_static` | Timestamped TF; no target TF is read |

The URDF defines one 640×480 RGB-D sensor at 30 Hz, HFOV 1.047 rad and clipping
0.10–10 m. RGB and depth come from the same rendering camera with the same dimensions,
intrinsics and timestamps, so the simulation's depth is registered to RGB. This follows
the [Gazebo 8 RGB-D implementation](https://github.com/gazebosim/gz-sensors/blob/gz-sensors8/src/RgbdCameraSensor.cc),
with floating-point image encoding preserved as 32FC1 by the
[Jazzy bridge](https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_bridge/src/convert/sensor_msgs.cpp).
Intrinsics are read from CameraInfo rather than approximated from HFOV.

The current URDF places `optical_frame_id` under `<sensor>` rather than `<camera>`.
Consequently Gazebo uses a scoped header such as
`robot1/base_footprint/rgbd_camera` even for its image/depth stream. The model already
has the correctly rotated `robot1/camera_optical_frame` with ROS optical axes
(x right, y down, z forward). The simulation defaults **explicitly** accept the
scoped image/calibration header and reconstruct rays in that optical TF frame.
The header is not used as if it had optical axes. The earlier point-cloud TF fix
connects the Gazebo sensor frame to `camera_link`; this detector does not consume
that point cloud or reinterpret its coordinate convention.

`base_footprint → base_link` is 0.049 m high and `base_link → camera_link` is
(0.175, 0, 0.051) m. The camera is therefore 0.10 m above the floor. Floor rejection
uses the actual camera-to-base TF at each image timestamp, not a hardcoded optical
pixel row. It assumes a common flat floor at `detection.floor_height` in observer
base coordinates. Observer tilt greater than 15 degrees relative to the tracking
frame is rejected. Slopes, steps and different target/observer floor elevations need
a better ground-plane model.

In green-seed mode, RGB and depth use best-effort sensor QoS and approximate timestamp synchronization
(25 ms tolerance, queue 10). Only the newest synchronized pair is processed at up to
10 wall Hz; older work is dropped to avoid backlog. Pairs older than 0.5 simulation
seconds are rejected. CameraInfo is cached, checked against image dimensions, frame
and supported calibration, and expires after 2 seconds; zero-stamped static calibration
is allowed. Missing calibration, missing TF, unmatched streams, unsupported encodings
and stale messages produce diagnostics rather than uncalibrated measurements.
TF waits are nonblocking and bounded by image age. No latest-TF fallback is used.

In geometry mode there is no RGB/depth synchronizer. When debug images are enabled,
an optional RGB subscription and calibration cache supply a normal-colour display
background only; neither supplies detection or tracking evidence. Only the
newest native depth message is queued for processing, using the same rate, age checks,
calibration validation, clock-reset handling and bounded TF retry. Depth timestamps
drive measurement updates and all camera/observer TF lookups. The image header
adaptation uses `expected_depth_frame` and `depth_optical_frame`, including the
existing Gazebo alias above. The depth stream's own `CameraInfo` and optical frame
are required. RGB calibration, RGB messages and depth-to-RGB registration are not
required for detection; `depth_registered` is irrelevant in this mode. Geometry
detection and depth debugging continue when RGB is absent. No colours influence candidates, quality,
initialization or association.

For a real camera, use a separate YAML file and `use_sim_time:=false`, for example:

```bash
ros2 launch edgeai_robot_tracking detection_tracking.launch.py \
  use_sim_time:=false config_file:=/absolute/path/real_camera.yaml \
  rgb_topic:=/camera/color/image_raw depth_topic:=/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  depth_camera_info_topic:=/camera/color/camera_info \
  camera_optical_frame:=camera_color_optical_frame \
  depth_optical_frame:=camera_color_optical_frame \
  expected_rgb_frame:=camera_color_optical_frame \
  expected_depth_frame:=camera_color_optical_frame \
  base_frame:=base_footprint tracking_frame:=odom output_frame:=base_footprint
```

These example real-camera topics are placeholders to replace with that camera's
actual interfaces. Confirm its registered-depth contract first. With
`depth_registered: true`, the node checks dimensions, K/D, optical frame and image
header agreement; matching sizes alone do not prove registration, so the setting is
an explicit source contract. For **unregistered** depth set `depth_registered: false`,
provide the depth stream's own CameraInfo, expected header and optical TF frame.
The node unprojects depth with its calibration, looks up the transform between
depth and RGB optical frames **at their respective timestamps** through the
continuous tracking frame, and projects into RGB with a nearest-surface z-buffer.
Occlusion holes remain invalid; images are never merely resized or blindly indexed.
Supported depth encodings are 32FC1 metres and 16UC1 with configurable
`uint16_depth_scale` (default 0.001 m/unit). Zero, negative, NaN, infinity and
out-of-range values are rejected. Only plumb-bob/rational pinhole distortion is
supported; cropped/binned grids requiring adjusted intrinsics and fisheye models
are rejected rather than silently misused.

## Detection, tracking and output semantics

### Original green-seed method

HSV components seed the detector; opening is off by default so small distant wheels
survive. One green wheel is sufficient. A seed uses multiple valid depths from the
densest bounded depth interval and a median 3D point. It must sit at plausible wheel
height. A sampled depth-connected component then adds nearby non-green surfaces,
with metric neighbor distance, physical search radius, pixel-count cap, height
bounds, floor removal and visible-size limits. Geometry touching the search/height
boundary is rejected, so a cropped wall is not treated as a complete robot.
Overlapping components from multiple green seeds merge. Partial observations are
allowed; a complete chassis or two wheels are not required.
Disjoint wheel patches without connecting depth geometry can remain ambiguous;
the detector does not force them together across an unsupported gap.

The model's plate is 0.375×0.265 m, wheel separation 0.300 m, wheel width 0.020 m and
wheel diameter 0.098 m. Default bounds include margins for visible robot geometry.
Loose appearance and partial-geometry checks cannot uniquely identify a robot, and
floor-connected or touching objects can still cause rejection or biased clusters.

### Geometry-only method

`geometry_detection.py` accepts validated native depth, its calibration and the
camera-to-base transform. It shares the candidate representation with the green
detector; colour-specific fields are `None`, with no invented green support.

The pipeline samples at stride 2 and unprojects points through the shared camera
utilities. It excludes the floor below 0.012 m and a configurable self-structure box.
The broad acquisition region defaults to base x = 0.25–5.0 m and |y| ≤ 3.0 m. This
region is independent of a target track or simulator identity and is reused for
reacquisition. The self box spans x = -0.24–0.23 m, |y| ≤ 0.19 m and height ≤ 0.23 m.

Four-neighbour depth-grid edges connect only when their 3D distance is ≤ 0.065 m.
The graph is embedded in a grid with explicit edge pixels and labelled once with
OpenCV connected components. Components are gathered by one label sort, avoiding
per-component whole-cloud flood fills or all-pairs distance comparisons. The sampled
grid is capped at 100,000 points, each component at 16,000 points, and the frame at
128 components having at least 6 points. Sample/component-count overflow suppresses
all candidates; an oversized individual component is rejected. Events appear in
diagnostic rejection counts. Increase stride for larger depth images.

Crucially, clustering retains points **above the accepted robot-height band**.
Height is checked after component extraction, so a tall obstacle cannot pass just
because its upper portion was cut away by the detector's height filter. Pre-crop
points are also retained to detect metric-connected continuation across horizontal
search/self-exclusion boundaries. Components touching those boundaries, the image
edge, or the configurable 0.02 m search margin are rejected conservatively. If an
obstacle's upper structure is already invisible to the camera, these checks cannot
recover that missing evidence.

The URDF has a 0.300 × 0.265 × 0.070 m lower chassis, a 0.375 × 0.265 m upper plate,
0.098 m diameter wheels, and a 0.300 m wheel separation. Its LiDAR support/body reach
approximately 0.175 m above `base_footprint`; the camera top is about 0.1125 m high.
Geometry defaults allow top height up to 0.23 m, top height at least 0.04 m, visible
vertical extent at least 0.025 m, and lowest visible point no higher than 0.11 m.
These margins allow partial wheel/chassis views without requiring a complete robot.

An oriented minimum-area rectangle in the horizontal plane bounds the visible
footprint. Its longer side must be 0.025–0.55 m, its shorter side at most 0.40 m,
and its diagonal at most 0.65 m. These are orientation-independent bounds, rather
than assuming base-frame x/y extents equal the robot's length/width. At least 25%
of points must lie in the 0.025–0.12 m chassis/wheel height band. Horizontal surfaces
are retained; there is no blanket planar-surface removal. A flat isolated patch
with insufficient vertical extent still fails the component-size checks.

Quality is exclusively geometric:

```text
q = 0.30 * min(1, point_count / support_saturation)
  + 0.25 * chassis_band_fraction
  + 0.25 * min(1, visible_height / 0.07 m)
  + 0.20 * min(1, horizontal_long_side / 0.265 m)
```

`support_saturation` defaults to 60 points; the minimum accepted quality is 0.45.
This is heuristic support, not a calibrated probability or a unique shape match.
Point spacing/connectivity, component support, size, and low-height evidence all
affect acceptance. Small cuboids, low furniture or other robot-sized objects can
pass. Nearby objects separated by sufficient depth gaps remain separate components;
touching objects may merge and be rejected or bias the centroid.

Disconnected wheels/chassis patches are **not grouped** by proximity. Unsupported
grouping could merge a robot with an obstacle. This version instead keeps separate
candidates, which can cause conservative ambiguous initialization or association.
Exactly one sufficiently plausible candidate must remain to start a track; it does
not choose the nearest/largest obstacle to force initialization. Existing tracker
ambiguity gates, confirmation and timeout are unchanged.

### Shared measurement and tracking

The measurement is the **mean of the selected visible surface points**, not the
robot's base center. It may be biased toward the near chassis face or a visible
wheel; its offset changes with viewpoint, occlusion and which parts are connected.
No center correction or target orientation is fabricated. Marker orientation is
only the identity required to draw a sphere/point set, not a robot heading.

A 3D constant-velocity Kalman filter tracks this surface measurement in
`robot1/odom` by default. Each camera measurement is transformed at its own timestamp
before association. Output is transformed back to `robot1/base_footprint` at the
output timestamp. This accounts for observer translation and rotation if its
odometry/TF is accurate. The repository's known collision/slip odometry drift still
affects tracking. The detector does not read `robot2` poses or use simulator ground
truth. A different continuous frame can be selected with `tracking_frame`.

The filter uses actual elapsed time, covariance and physical-distance/speed gates,
plus support quality and visible extent. Two hits confirm initialization. Ambiguous
initialization or similarly plausible associations produce no new measurement;
the existing track briefly predicts instead of silently selecting a new candidate.
Quality is a **heuristic support score**, not a probability or calibrated accuracy.
Predictions decay in confidence; after 1 second without an accepted measurement,
position publication stops and markers are deleted. Reacquisition creates a new
track ID after expiration and does not establish identity continuity. Delayed
frames cannot rewind/refresh a track. Simulation clock rewind resets it; paused
simulation time also pauses its expiry. Keyboard expiry always uses wall time.

All output topics are relative to the observer namespace:

| Topic under `/robot1` | Type | Meaning |
| --- | --- | --- |
| `tracking/position` | PointStamped | Filtered surface point in configured output frame, normally observer base |
| `tracking/position_continuous` | PointStamped | Same filtered surface point in continuous tracking frame |
| `tracking/status` | DiagnosticArray | State, validity, reason, age, track ID, quality, covariance std, method, processing duration, candidate details, input and rejection counts |
| `tracking/debug_image` | Image (BGR8) | RGB/mask view in green-seed mode; normal RGB only in geometry mode; yellow candidate boxes, green selected surface |
| `tracking/debug_depth_image` | Image (BGR8) | Geometry-mode native-depth view with candidate boxes and scores, independent of RGB availability |
| `tracking/green_mask` | Image (mono8) | Full-resolution threshold mask, published/advertised only in green-seed mode |
| `tracking/markers` | MarkerArray | Surface sphere (green measured/orange predicted) and optional selected cluster |

Measured outputs carry the RGB timestamp in green-seed mode and the depth timestamp
in geometry mode. Watchdog predictions carry an explicit
prediction timestamp normally 60 ms behind the current clock to allow timestamped
odometry TF to arrive, and never precede the last processed frame. Missing output
TF suppresses position publication and sets `valid=false`. Do not reuse an old
PointStamped indefinitely: check its age and `tracking/status`. A debug image carries
its original image timestamp; if the camera stops, that last image can remain visible
while status/marker outputs continue to expire the track.

Both position topics contain the same filtered surface estimate: `position` in
`output_frame`, `position_continuous` in `tracking_frame`. Both can be measured or
predicted. Output-frame TF failure suppresses both publications as before.

The debug overlay includes the active method and, for geometry mode, `RGB` or
`depth` to identify the displayed background. The two backgrounds have separate
topics and never alternate on the same topic. Geometry mode uses the normal RGB
image closest to the depth measurement, within `sync_slop` (25 ms by default).
Candidate surface points are projected from the depth-time base frame into the
RGB-time optical frame using timestamped TF and RGB calibration; this supports
different image resolutions and separate cameras without using RGB for detection.
The debug image carries the background RGB timestamp/frame in this case, while
tracking measurements and position outputs continue to use the depth timestamp.

If RGB is missing, stale, uncalibrated, unsupported, or its display TF is unavailable,
no new `debug_image` is published for that depth frame. A viewer may retain its
last RGB image; inspect its timestamp to check freshness. Tracking continues and
`debug_depth_image` is still published for every successfully processed depth frame.
There is no automatic RGB-to-depth fallback on `debug_image`. This avoids colour
flickering when RGB arrives just outside the 25 ms matching tolerance or when
display TF is temporarily unavailable. Valid depths in `debug_depth_image`
use a fixed 0–`max_depth` Turbo colour scale (nearer points have higher colour-map
indices), with invalid rays black. With `debug_image=false`, no optional RGB display
subscription is created in geometry mode. A bounded cache retains at most
`sync_queue` RGB messages and is cleared on clock rewind.

Each geometry candidate is labelled `#0 score=0.78`, for example: `0` is the candidate
index in this frame (not a persistent robot ID), and `0.78` is geometric quality on
a 0–1 scale (not distance or a calibrated probability). Green outlines show the
candidate accepted for the current tracking update; yellow outlines show other
detected candidates. Candidate indices may change between frames. Debug boxes do
not identify the robot's centre or pose.

Diagnostic `candidate_details` includes point count,
top/bottom heights, oriented footprint dimensions and chassis-band fraction.
`processing_duration_ms` measures the last successful preparation/detection/tracking
pass, excluding publishing and debug drawing. It is not an end-to-end latency
measurement. Counts/details remain from the last processed frame if input stops.
Geometry rejection keys include `floor_points`, `self_points`,
`outside_search_points`, `small_components`, `sample_limit`, `component_limit`,
`component_point_limit`, `search_or_image_boundary`, `tall_component`,
`height_dimensions`, `footprint_dimensions`, `chassis_support`, and `low_quality`.
The point counters and component counters have different units.

Inspect outputs in a prepared terminal:

```bash
ros2 topic echo /robot1/tracking/status
ros2 topic echo /robot1/tracking/position
ros2 topic hz /robot1/tracking/debug_image
ros2 run tf2_ros tf2_echo robot1/odom robot1/camera_optical_frame
```

In the existing RViz, keep Fixed Frame `world` (or choose `robot1/odom`), add **Image**
for `/robot1/tracking/debug_image` and **MarkerArray** for
`/robot1/tracking/markers`. Perception itself does not require RViz. Disable image
or marker work with `debug_image:=false`, `debug_markers:=false` and/or
`debug_cluster:=false` on the experiment launch command.

For geometry depth visualization, add a separate **Image** display for
`/robot1/tracking/debug_depth_image`. Keep `/robot1/tracking/debug_image` selected
for normal RGB colours; RGB availability never changes that topic into a depth view.

## Tuning and manual test procedure

### Configuration precedence

`tracking.yaml` remains unchanged and supplies the original shared/tracker and
`detection.*` green-seed defaults. `geometry.yaml` supplies separate `geometry.*`
defaults without changing shared noise, timing, gates or timeout for comparison.

Node built-in defaults are overridden by launch frame defaults, then
`geometry_config_file` **only in geometry mode**, then `config_file`, then explicit
topic/frame/debug launch overrides. The launcher always applies its
`detection_method` and `use_sim_time` arguments last. `config_file` defaults to the
existing `tracking.yaml`; a custom file can contain shared and either method's
settings, and its geometry settings override `geometry_config_file`. Unspecified
settings use earlier files or node defaults. This preserves real-camera YAML frame
overrides and the original configuration workflow.

For a focused geometry override, create a YAML file such as:

```yaml
/**:
  ros__parameters:
    geometry:
      max_forward: 4.0
      cluster_stride: 2
      connect_distance: 0.065
      min_quality: 0.45
```

Then run:

```bash
ros2 launch edgeai_robot_tracking detection_tracking.launch.py \
  detection_method:=geometry config_file:=/absolute/path/geometry_override.yaml
```

All geometry defaults are listed in
`src/edgeai_robot_tracking/config/geometry.yaml`. Tune depth calibration and floor
reference first, followed by the self box/search volume, sampling/connectivity,
support/visible dimensions, and geometric quality. `min_component_points` and
`support_saturation` operate on sampled points, so changing stride changes support.
Farther targets have fewer/wider-spaced samples; lower support thresholds or adjusted
connectivity may help but also admit clutter. Do not widen gates or remove height
checks merely to force a track. Parameters are validated at startup and require a
restart to take effect.

### Original green-seed tuning and basic checks

Copy `src/edgeai_robot_tracking/config/tracking.yaml` to a local file and pass
`config_file:=/absolute/path/to/your.yaml`. Restart perception after changes; parameters
are startup settings. Launch arguments override YAML only when explicitly supplied.
Tune HSV limits against the debug mask first, then `min_green_area` and
`min_seed_depths` for distance. Next tune `seed_depth_band`, `connect_distance`,
`cluster_stride` and geometry bounds against the selected cluster, followed by
`tracking.measurement_std`, association gates and timeout. Increasing clustering
radius/connectivity can admit obstacles; increasing timeout extends uncertain
predictions. `floor_height` and TF must be correct before loosening geometry checks.

1. Start two robots at the default spawns: robot1 at (-2,0), robot2 at (0,0), both
   facing +x. Robot1 looks toward robot2. Confirm RGB/depth/CameraInfo and TF before
   tuning thresholds. Wait for two accepted frames and a measured green sphere.
2. Drive robot2 with `w` and `s` toward/away from robot1, staying clear of arena walls
   and collisions. Check that the relative forward position decreases/increases and
   the selected points stay on robot surfaces. Use Space to stop.
3. Turn robot2 with `a`/`d` until one green wheel is visible. Check partial detection;
   only a few wheel pixels at long range may give initializing/predicted/lost states.
4. Briefly occlude the wheel/camera with an object in Gazebo or steer behind an
   obstacle for less than the timeout. Expect predicted (orange), then measured
   on unambiguous return. Nearby gray objects should remain separate across depth
   gaps; touching objects may invalidate the cluster.
5. Drive the target out of the camera's field of view. Expect lost and no new target
   points after the configured timeout. Return it; initialization confirms a new
   track ID. Stop/unsubscribe depth separately to verify missing input also expires.
6. In another terminal run keyboard control with `--namespace robot1`, and move the
   observer slowly while the target stays still. The relative point should change
   while `position_continuous` stays approximately fixed, apart from visible-surface
   changes and odometry error. Avoid contacts/slip during this check.
7. Add a green floor patch/high object/large wall, then a small green object at wheel
   height using Gazebo's GUI. Check seed-height and visible-dimension rejection in
   diagnostics, and verify depth separation near a gray obstacle. A small green
   object with robot-compatible dimensions can pass; two plausible candidates
   should cause ambiguous initialization/association rather than an identity claim.
8. Release input, press Space, quit and use Ctrl+C in the teleop terminal. Confirm
   `/robot2/cmd_vel` becomes zero and the terminal remains usable. While simulation
   is paused, keyboard input must still expire; no target updates should acquire
   advancing simulation timestamps.

## Automated verification

```bash
pixi run tracking-lint
pixi run tracking-test
pixi run env PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 colcon test \
  --packages-select edgeai_robot_tracking --event-handlers console_direct+
pixi run colcon test-result --test-result-base build/edgeai_robot_tracking --verbose
```

The test task disables automatic third-party pytest plugin loading because this
Pixi environment combines pytest 9 with a ROS launch_testing plugin that still
declares an obsolete pytest hook. These tests use plain pytest and do not need
that plugin. Unit tests cover units/invalid depth, calibration, registration and
z-buffer occlusion, partial green seeds and gray geometry, floor/wall distractors,
obstacle separation, duplicate wheels, timing/gating/ambiguity/expiration,
reacquisition, observer transforms and the keyboard watchdog.

An opt-in ROS transport test publishes synthetic images and moving-observer TF,
checks actual subscription/filter/output/marker behavior, then stops depth and
checks prediction and expiration. Run it in a separate domain:

```bash
pixi run env ROS_DOMAIN_ID=89 ROS_LOG_DIR=/tmp/edgeai_tracking_ros_test \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 RUN_ROS_TESTS=1 \
  PYTHONPATH=src/edgeai_robot_tracking \
  python -m pytest -q src/edgeai_robot_tracking/test/test_ros_pipeline.py \
  src/edgeai_robot_tracking/test/test_teleop_ros.py
```

The PTY tests exercise actual keyboard commands, input expiry, explicit stop,
quit/Ctrl+C zero publication and terminal restoration over a test-only Twist topic.
These synthetic ROS tests are distinct from validating rendered Gazebo images and
physical camera geometry. Real-scene performance, difficult lighting, long range,
occlusion and robot-like green distractors require the manual checks above.

New tests exercise native-depth geometry without green pixels, RGB colour/no-RGB
independence, native depth calibration/frame/timestamps with incompatible RGB setup,
floor-only scenes, tall pre-crop obstacles, partial views, separated obstacles,
disconnected-candidate ambiguity, rotated horizontal footprints, cropped walls,
metric connectivity and processing caps. Launcher tests check both methods, default
selection, invalid values and configuration precedence. The ROS transport test is
parameterized for both methods, with geometry cases both without RGB and with RGB
used only for display. They verify no geometry green masks are published while
tracking, predicting and expiring normally. RGB display tests cover original pixel
colours, projected boxes on a different image grid, frame/timestamp handling and
independent depth output for missing/stale/invalid RGB, missing calibration or
missing display TF. An intermittent RGB/TF regression test verifies that the RGB
topic never contains false-colour depth frames, and ROS tests check both topic
contracts, including no RGB debug output when no RGB publisher exists.

The original green-seed implementation report: both packages built, lint passed, and 17 unit tests
plus 3 opt-in ROS/PTY integration cases passed. The dedicated launcher also started
the two-robot server and detector
with the intended parameters. Rendered-camera validation could not complete:
the environment has no `/dev/dri` device, and the software-rendering attempt crashed
inside EGL/OGRE2 camera initialization before any images arrived. The detector
continued reporting `lost` with zero image inputs and published no target positions.
This does not establish detection performance on real Gazebo camera images; run the
manual checks on the working graphical simulator.

### Geometry extension verification (2026-10-31)

Checks actually performed for this extension:

- `pixi run tracking-test`: **43 passed, 5 skipped**. The skipped cases are the
  three opt-in ROS transport cases and two keyboard PTY cases.
- `pixi run tracking-lint`: **passed**.
- `pixi run colcon build --symlink-install --packages-up-to edgeai_robot_tracking`:
  **both `edgeai_robot_description` and `edgeai_robot_tracking` built successfully**.
- Explicit opt-in ROS/PTY run: **5 passed**, including geometry detection and
  debug/position outputs without an RGB publisher, optional normal-colour RGB
  display, moving-observer compensation, prediction/expiry, absence of a geometry
  green mask, and keyboard cleanup.
- The RGB-debug update also rebuilt `edgeai_robot_tracking` successfully with
  `pixi run colcon build --symlink-install --packages-select edgeai_robot_tracking`.
- Installed launcher `--show-args`: **passed**, with both methods listed and
  `green_seeds` as default.

The restricted coding sandbox initially prevented DDS interface enumeration. The
successful integration run used local socket access and this loopback-only DDS
configuration, with multicast disabled. To reproduce that exact configuration:

```bash
cat > /tmp/edgeai_geometry_dds.xml <<'XML'
<CycloneDDS>
  <Domain id="any">
    <General>
      <Interfaces><NetworkInterface name="lo"/></Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery><Peers><Peer address="127.0.0.1"/></Peers></Discovery>
  </Domain>
</CycloneDDS>
XML

pixi run env CYCLONEDDS_URI=file:///tmp/edgeai_geometry_dds.xml \
  ROS_DOMAIN_ID=89 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  ROS_LOG_DIR=/tmp/edgeai_tracking_geometry_ros_test \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 RUN_ROS_TESTS=1 \
  PYTHONPATH=src/edgeai_robot_tracking \
  python -m pytest -q --tb=short \
  src/edgeai_robot_tracking/test/test_ros_pipeline.py \
  src/edgeai_robot_tracking/test/test_teleop_ros.py
```

This requires permission to create local DDS sockets. Pixi was invoked as
`/home/diego/.pixi/bin/pixi` during the checks because it was not on the coding
shell's PATH. The commands above assume Pixi is on PATH. Pytest emitted two
dependency deprecation warnings from the installed Lark parser; there were no
test failures in the successful runs.

## Repeatable sequential comparison

Use one running two-robot simulation, one detector at a time, the same observer,
the same sensor/TF configuration, and the same shared tracking settings. Keep a
record of any method-specific tuning rather than adjusting it between individual
trials. `target_ns` remains keyboard-only throughout.

1. Start the multi-robot simulation once in a prepared terminal:

   ```bash
   ros2 launch edgeai_robot_description multi_robot_sim.launch.py num_robots:=2
   ```

2. Launch the green-seed detector with the explicit command near the top of this
   guide, and start keyboard control in another terminal. Record status, both point
   topics and debug images. A ROS bag can preserve observer sensor/TF input and
   `/clock` for repeating the same input sequence; replay in an isolated domain
   without live simulation publishers and without recording/replaying duplicate
   detector outputs as input. Use this input-topic set if bag recording is available:

   ```text
   /robot1/camera/image /robot1/camera/depth_image /robot1/camera/camera_info
   /tf /tf_static /clock
   ```

3. Run the following scene sequence and log detection counts, accepted quality,
   measurement age, measured/predicted/lost transitions, processing duration and
   selected visible surface. Use the same simulator placements, target travel and
   durations for the second method. Reset robot poses/odometry between manually
   repeated trials, or replay the identical input bag to remove motion differences.

   | Trial | Motion/scene | Compare |
   | --- | --- | --- |
   | Distance | Start at default spawns, then approach/retreat in clear space | Acquisition range, sampled support, centroid stability, loss distance |
   | Rotation | Target turns through front/side/rear views, including one visible wheel | Partial observations, split components, candidate count and ambiguity |
   | Brief occlusion | Hide the target for less than 1 s, then longer than 1 s | Prediction, expiry, confirmation and new track ID after expiry |
   | Observer motion | Stationary target, observer translates and rotates slowly | Relative output changes; continuous output approximately fixed, subject to odometry/surface changes |
   | Nearby wall | Move target near a wall first with a visible gap, then touching | Separation, merged-component rejection, centroid bias; avoid prolonged wheel slip |
   | Similar-sized object | Put an ordinary low box beside the robot, then remove robot | Geometry false positives and conservative multi-candidate ambiguity; colour-method differences |
   | Colour/RGB independence | Recolour target surfaces or stop only RGB while depth/calibration/TF continue | Geometry candidates continue; green method loses colour support/input and expires |

4. Stop only the detector, restart it with `detection_method:=geometry`, and repeat
   the identical scene/motion sequence. With geometry active, display its debug
   topic and selected-cluster marker. Do not compare a stale RGB image with a fresh
   depth result without checking timestamps.
5. Distinguish tracking a visible surface from tracking the target centre. If manual
   simulator reference measurements are used to evaluate error, keep them entirely
   outside detector inputs and search-region configuration.

This procedure describes manual comparisons still to perform on a working rendered
simulator; it does not claim those trials have already passed. In the current coding
environment `/dev/dri` is unavailable. Rendered end-to-end comparison was not run.
