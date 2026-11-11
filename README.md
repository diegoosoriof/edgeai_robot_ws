# EdgeAI Robot Workspace

A ROS 2 workspace for a realistic differential-drive robot in Gazebo, with RViz visualization, LiDAR, an RGB-D camera, and SLAM toolbox mapping.

## Environment

- Ubuntu 24.04 (x86-64).
- Gazebo Harmonic.
- ROS 2 Jazzy managed by **Pixi**, using the RoboStack Jazzy and conda-forge channels in `pixi.toml`.
- Pixi installed and available on `PATH`; a graphical desktop for Gazebo and RViz.

The manifest and lockfile define the workspace dependencies. Use the Pixi environment for ROS commands and builds; the instructions below do not require sourcing `/opt/ros/jazzy/setup.bash`.

Pixi uses ROS domain `0` and local-only DDS. For communication between computers,
change `CYCLONEDDS_URI` in `pixi.toml`.

## Install and build

```bash
git clone https://github.com/dongjw-hit/edgeai_robot_ws.git
cd edgeai_robot_ws
pixi install
pixi run colcon build --symlink-install
```

Run these commands from the repository root. Rebuild after changing package resources or adding configuration files.

## Start the simulation

From the repository root:

```bash
./start_sim.sh
```

The script resolves the workspace relative to its own location, activates Pixi, sources `install/setup.bash`, launches `simulation_real.launch.py`, and opens RViz with `realistic_robot.rviz`. It requires a completed build and does not depend on a fixed home-directory path. Press **Ctrl+C** in its terminal to stop the launched session.

The launch file loads `edgeai_robot_real.urdf.xacro`, starts the world in `worlds/empty.sdf`, spawns `edgeai_robot_real`, and starts the ROS–Gazebo bridge and robot transforms. The old simple model and its launch script have been removed; `start_sim.sh` is the current entry point.

The single-robot startup does not start localization or navigation. Part 2 uses the dedicated launch below.

## Part 2: autonomous rescue

```bash
pixi install
pixi run colcon build --symlink-install --packages-select edgeai_robot_description edgeai_rescue
source setup_part2.bash
ros2 launch edgeai_rescue rescue_part2.launch.py
```

Starts two robots, a shared map, independent AMCL/Nav2, the rescue FSM, verification and RViz.
Use `autostart_mission:=false` for debugging or `map_yaml:=/path/map.yaml` for a team-prepared map.
Commands, configuration and evidence: [Part 2 README](src/edgeai_rescue/README.md).

## Optional ArUco perception

After rebuilding, run `./start_sim.sh enable_aruco:=true` to detect targets using the existing RGB-D camera. Also, the world includes a camera-visible placeholder `DICT_4X4_50` marker, ID `1`, with a 0.20 m black-square side on `obstacle_3`. Moreover, the detector publishes timestamped ID–pose observations on `/aruco/detections`,
matching `/aruco/ids` and `/aruco/poses` arrays, and `/aruco/debug_image`. Detection is disabled by default. 

Configuration, namespace support, asset generation, and verification commands are in [the ArUco package README](src/edgeai_aruco/README.md).  The RGB-D topics retain Gazebo's original sensor frame so `/camera/points` stays aligned with the robot; ArUco observations use `camera_optical_frame`.

## Rescue challenge arena

### Midterm Part 1: fresh exploration, mapping, and target evidence

This entry point uses the existing `rescue_arena.sdf`, robot1, ArUco detector,
SLAM Toolbox, Nav2, MRTSP frontier explorer, recorder, and an RViz map view.
From the workspace root, install the Pixi environment, build once, then launch:

```bash
pixi install
pixi run colcon build --symlink-install --packages-select edgeai_aruco mrtsp_exploration_ros2 edgeai_robot_description
./start_part1.sh
```

If the installed Pixi binary is too old to read `pixi.lock` but
`.pixi/envs/default` already exists, use the installed environment directly:

```bash
export PATH="$PWD/.pixi/envs/default/bin:$PATH"
source .pixi/envs/default/local_setup.bash
colcon build --symlink-install --packages-select edgeai_aruco mrtsp_exploration_ros2 edgeai_robot_description
```

For a headless check, use `./start_part1.sh enable_rviz:=false
gz_args:='-r -s --headless-rendering'`. The launch loads the final arena but
leaves SLAM unconfigured and exploration paused. In another terminal, from the
workspace root:

```bash
./part1_control.sh start
./part1_control.sh status
./part1_control.sh save
```

`start` waits for robot TF, activates fresh SLAM, starts Nav2, then releases
automatically generated frontier goals and starts the simulation-time record.
Call it only once per launch. `save` writes `map.yaml` and `map.pgm` into the
current run directory. To reset, stop the launch with Ctrl+C, relaunch it, and
call `start` again. This creates new SLAM, Nav2, exploration, and detection
processes and a separate run directory; old output is retained only as evidence
and is never loaded. The initial robot1 pose is (0.45, 0.45) facing north in
the arena's 0–5 m x/y convention, inside the 0–1.4 m by 0–1.0 m start area.
By default, each Part 1 launch gets its own Gazebo transport partition, so another
simulation on the machine cannot mix its `/clock`, scans, or odometry into this
run. Stop any Part 1 launch started before this partition change once.

Output defaults to `runs/part1/<UTC timestamp>_<unique id>/` with
`detections.jsonl`, `timing_summary.json`, `images/S1_*.png`,
`images/C1_*.png`, and saved map files. `runs/part1/latest_run.txt` points to
the active output directory. Pass `output_root:=/absolute/path` to the launch
and set `PART1_OUTPUT_ROOT=/absolute/path` for the save command to move output.
The live map is `/map`; generated frontiers are `/explore/frontiers` and
`/explore/selected_frontier`. Calibrated camera observations are on
`/robot1/aruco/detections`; `/part1/target` publishes JSON with ID, S1/C1
type, image timestamp, and visual position in
`robot1/camera_optical_frame` (x right, y down, z forward, metres).
`/part1/summary` publishes the current detection and timing report.
Part 1 omits the arena visualization's `world` to `robot1/odom` anchor so
SLAM alone owns the map-to-odometry TF link.

Detected targets also appear in RViz's **Robot 1 Saved Markers** display as
green panels and ID labels, retained when they leave the camera view. The
registry publishes `/robot1/aruco/known_markers_visualization` and stores poses
in `robot1/odom`; RViz transforms them into its `map` fixed frame once SLAM is
active. If adding the display manually, use **MarkerArray**, that topic,
**Reliable**, and **Transient Local**. Enable **Robot 1 ArUco Debug** to see
the annotated camera images as well. Restarting the launch clears the registry;
markers appear again when observed during the new run.

The printed marker size used by the detector and stand models is 0.16 m,
excluding the white margin. Both stand centers remain at S1 (1.0, 4.1) and
C1 (4.1, 1.0), with marker centers 0.35 m above the floor. The marker's local
surface normal is +Z; its 90° roll points S1 toward world −Y (lower/open
arena), and C1's additional 180° world yaw points it toward +Y
(upper/open northern approach). A literal east/right (+X) orientation would
conflict with the competition directions, so the specified south/north faces
are used. No known target coordinate enters visual pose estimation.

Part 1 has no mandatory cutoff. The suggested duration is about six minutes;
the competition's 13-minute timer covers Parts 1 and 2 together. Mapping
depends on wheel odometry and LiDAR, so contact with walls can still cause
odometry drift. The camera view is forward-facing and marker evidence depends
on the explorer reaching a visible approach angle.

Starts Gazebo, RViz and ArUco. Run from the workspace root:

```bash
eval "$(pixi shell-hook --shell bash)"
source install/setup.bash
ros2 launch edgeai_robot_description rescue_arena.launch.py robot_count:=2
```

For only robot1:

```bash
ros2 launch edgeai_robot_description rescue_arena.launch.py robot_count:=1
```

`robot_count` defaults to `2`; RViz follows the selection. Topics use `/robot1/...`
and `/robot2/...`; RViz's fixed frame is `world`.
Optional flags: `enable_aruco:=false`, `enable_rviz:=false`.
For headless use, add `enable_rviz:=false gz_args:="-r -s --headless-rendering"`.

Teleoperate in another activated terminal (replace `robot1` with `robot2` as needed):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -r cmd_vel:=/robot1/cmd_vel -p speed:=0.1 -p turn:=0.5
```

If topics are missing: `ros2 topic list --no-daemon --spin-time 2`.

Markers: `DICT_4X4_50`, S1 ID **1**, C1 ID **2**, printed size **0.16 m**.
Edit [rescue_targets.yaml](src/edgeai_robot_description/config/rescue_targets.yaml),
then regenerate, rebuild and restart:

```bash
ros2 run edgeai_robot_description generate_rescue_targets.py \
  --config src/edgeai_robot_description/config/rescue_targets.yaml \
  --models-dir src/edgeai_robot_description/models
pixi run colcon build --symlink-install --packages-select edgeai_robot_description
```

[Arena reference](docs/arena_layout_reference.png) · [Gazebo view](docs/rescue_arena_top_down.png).
C1 currently has one 0.70 m zone; the official two-zone positions remain undefined.

## ROS commands in another terminal

In each additional terminal, enter the cloned repository and activate its environment:

```bash
cd /path/to/edgeai_robot_ws
eval "$(pixi shell-hook --shell bash)"
source install/setup.bash
```

Replace `/path/to/edgeai_robot_ws` with your clone location. The following commands assume this setup and a running simulation.

### Main topics

| Topic | ROS message type | Purpose |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Robot velocity commands |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo DiffDrive odometry |
| `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic transforms, including `odom` → `base_footprint` |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Fixed robot and sensor transforms |
| `/joint_states` | `sensor_msgs/msg/JointState` | Simulated joint states |
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR scan |
| `/camera/image` | `sensor_msgs/msg/Image` | RGB image |
| `/camera/depth_image` | `sensor_msgs/msg/Image` | Depth image |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera calibration |
| `/camera/points` | `sensor_msgs/msg/PointCloud2` | RGB-D point cloud |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation time |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Occupancy map when SLAM toolbox is running |

The saved RViz configuration uses `odom` as its fixed frame. To inspect available topics:

```bash
ros2 topic list
ros2 topic echo /odom --once
```

### Move and stop

In clear space, publish a slow forward command:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

Stop the publisher with **Ctrl+C**, then explicitly send a zero velocity command:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'
```

Stopping the publisher alone should not be treated as a robot stop command.

## Mapping with SLAM toolbox

Keep the simulation running. In another prepared terminal at the repository root:

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$PWD/src/edgeai_robot_description/config/slam_toolbox.yaml" \
  use_sim_time:=true
```

The repository configuration uses `/scan`, `base_footprint`, `odom`, and `map`, with mapping mode enabled and a map resolution of 0.05 m. Simulation time comes from the bridged `/clock` topic. The simulation startup script does not start SLAM automatically.

In RViz, set **Fixed Frame** to `map` after SLAM starts and add a **Map** display for `/map`. Drive slowly through free space to collect scans and revisit previously observed areas. Avoid sustained contact with walls or obstacles.

### Save the occupancy map

While SLAM is running and `/map` is available, use another prepared terminal at the repository root:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/my_map" \
  --ros-args -p use_sim_time:=true
```

This writes `maps/my_map.yaml` and its associated map image. Choose a new basename for each map you want to preserve. These files are occupancy-map exports, not a serialized SLAM toolbox pose graph for resuming a mapping session.

The package also contains `src/edgeai_robot_description/maps/edgeai_arena.yaml` and its map image. Saving `maps/my_map` does not overwrite the packaged map.

## Known limitation: odometry drift during collisions

In the reported movement tests, normal straight driving stayed synchronized between Gazebo and RViz, and an explicit stop command stopped the robot. When the robot contacted a wall or obstacle, its physical motion in Gazebo stopped while `/odom` continued changing and the robot in RViz continued moving.

The current `/odom` source is the Gazebo DiffDrive plugin. Treat this odometry as an estimate rather than collision-corrected ground truth: wheel motion under contact or slip can diverge from actual displacement. The observed collision drift remains unresolved. Stop the robot when it becomes blocked, avoid driving into obstacles during mapping, and do not interpret continued RViz movement as physical motion in Gazebo. SLAM is not a guarantee that this behavior is corrected.

## Repository layout

```text
pixi.toml / pixi.lock                 Pixi environment and dependency lock
start_sim.sh                         Simulation + RViz entry point
src/edgeai_aruco/                    Optional camera-based ArUco perception and tests
src/edgeai_robot_description/
  launch/simulation_real.launch.py   Realistic simulation and bridges
  urdf/edgeai_robot_real.urdf.xacro   Current robot model
  worlds/empty.sdf                   Simulation world
  models/edgeai_aruco_marker/         Reusable visual-only ArUco panel
  rviz/realistic_robot.rviz           RViz configuration
  config/slam_toolbox.yaml           Mapping parameters
  maps/                             Packaged map resources
```

Start Gazebo, the robot, SLAM Toolbox, Nav2, and RViz:

```bash
ros2 launch edgeai_robot_description bringup.launch.py
```

To enable autonomous exploration:

```bash
ros2 launch edgeai_robot_description bringup.launch.py explore:=true
```
