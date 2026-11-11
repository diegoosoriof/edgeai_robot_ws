# edgeai_aruco

ROS 2 / OpenCV ArUco detection using `/camera/image` and `/camera/camera_info`.
The Gazebo test marker on `obstacle_3` uses `DICT_4X4_50`, ID **1**, and a **0.20 m** black square.

That marker belongs to the `start_sim.sh` sensor sandbox. The midterm uses the
shared rescue arena with **0.16 m** S1/ID 1 and C1/ID 2 stands, configured in
`edgeai_robot_description/config/rescue_targets.yaml`. Use
[`start_part1.sh` and the workspace guide](../../README.md#part-1-explore-map-and-detect)
for autonomous mapping/detection; its topics have the `/robot1` prefix. Part 2
uses known targets and does not require another camera detection pass.

## Build and launch

Run from the workspace root:

```bash
pixi install
pixi run colcon build --symlink-install
./start_sim.sh enable_aruco:=true
```

ArUco detection is disabled when `enable_aruco` is omitted.

## Inspect detections

In another terminal, from the workspace root:

```bash
pixi shell
source install/setup.bash
ros2 topic echo /aruco/detections --once
ros2 topic echo /aruco/ids --once
ros2 topic echo /aruco/known_markers --once --qos-durability transient_local
```

To start only the detector with an existing simulation, if it is not already running:

```bash
ros2 launch edgeai_aruco detection.launch.py use_sim_time:=true
```

## Topics and RViz

| Topic | Content |
| --- | --- |
| `/aruco/detections` | `Detection3DArray`: IDs, poses, timestamps, and frame together |
| `/aruco/ids` | `Int32MultiArray`: IDs corresponding to `/aruco/poses` |
| `/aruco/poses` | `PoseArray`: current marker poses |
| `/aruco/debug_image` | Image with corners, IDs, and pose axes |
| `/aruco/known_markers` | `Detection3DArray`: remembered markers in `registry_frame` |
| `/aruco/known_markers_visualization` | `MarkerArray`: saved marker panels and ID labels for RViz |

Use the **ArUco Debug Image** display in RViz. For a manual Image display, select
`/aruco/debug_image`, **Reliability Policy: Best Effort**, **Durability Policy: Volatile**.
The **Saved ArUco Markers** display shows green panels and ID labels that remain visible out of view.
To add it manually: **Add → MarkerArray**, topic `/aruco/known_markers_visualization`,
**Reliable**, **Transient Local**. Use **Fixed Frame: odom** (or your configured `registry_frame`).

Live detection poses use metres and `camera_optical_frame` (X right, Y down, Z forward), with image timestamps.
Detection runs independently per image: markers leaving view are removed from the output.
The registry retains each dictionary/ID while invisible and updates its fixed-frame pose on reappearance.
Its array header gives the snapshot time; each detection's header stamp is its **last-seen image time**.
Storage lasts for the running session; restarting the registry or resetting ROS time clears it.
IDs must be unique per physical marker. No spatial clustering or pose smoothing is applied.
In Part 1 the registry also waits for `/part1/run_enabled`, clears when the run
starts and ignores observations captured before Start. Panels are green and ID
labels are black for contrast against the map.

## Configuration

Edit [config/aruco.yaml](config/aruco.yaml) for dictionary, marker size, topics, frames, and optional TF.
`marker_size` excludes the white margin; `marker_size_ids` / `marker_sizes` allow per-ID sizes.
Keep `input_frame` as the camera's actual header frame and `optical_frame` as its optical TF frame.
Restart the detector after configuration changes.
`registry_frame` defaults to `odom`; select `map` when that TF is available. Poses are transformed
at image timestamps, with brief retries for delayed TF. Odometry drift also affects stored poses.
The launch starts the registry automatically; use `enable_registry:=false` to disable it.

Use a custom configuration with:

```bash
./start_sim.sh enable_aruco:=true aruco_params_file:=/absolute/path/aruco.yaml
```

## Generate a Gazebo marker

With Pixi active and the workspace sourced:

```bash
ros2 run edgeai_aruco generate_marker \
  --output src/edgeai_robot_description/models/edgeai_aruco_marker \
  --dictionary DICT_4X4_50 --id 1 --size 0.20
pixi run colcon build --symlink-install
```

Keep the generated marker's dictionary and size consistent with the detector configuration.

## Check the package

```bash
env -u PYTHONPATH pixi run colcon test --packages-select edgeai_aruco --event-handlers console_direct+
pixi run colcon test-result --verbose
```

For the shared arena's Gazebo camera, marker-pose and sensor smoke test (EGL
rendering required), with the workspace environment sourced:

```bash
python src/edgeai_robot_description/test/verify_rescue_arena.py --output-dir /tmp/edgeai_aruco_validation
```
