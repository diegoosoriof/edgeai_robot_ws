# KTH EdgeAI Robot Workspace

One ROS 2 workspace for the KTH EdgeAI midterm: autonomous exploration, SLAM and
visual target detection in Part 1; autonomous single and cooperative rescue in
Part 2. Both use the **same arena, target stands and robot URDF**. The two parts
run independently, as permitted by the task description. Part 2 uses a prepared
map and does not need the map or detections produced by Part 1.

The implementation was reviewed against *KTH EdgeAI Robotics Challenge — Midterm
Check, Task Description*, proposed for November 12, 2026. See the
[requirements review](docs/midterm_requirements_review.md) for evidence, checks
and remaining demonstration requirements.

## Install and build

Use Ubuntu 24.04 x86-64 with a graphical desktop for Gazebo/RViz. Pixi supplies
ROS 2 Jazzy, Gazebo Harmonic and the dependencies in `pixi.toml`; do not mix these
commands with a system ROS installation.

```bash
git clone --recurse-submodules https://github.com/dongjw-hit/edgeai_robot_ws.git
cd edgeai_robot_ws
# Also run this after cloning without --recurse-submodules:
git submodule update --init --recursive
pixi install
pixi run colcon build --symlink-install
```

Run commands below from the workspace root. In each additional Bash terminal:

```bash
source setup_workspace.bash
```

All entry scripts use this same setup, local-only Cyclone DDS and ROS domain
`0` (or your existing `ROS_DOMAIN_ID`). `setup_part2.bash` remains a compatibility
alias. Each Part 1/2 launch has its own Gazebo partition. Stop one launch before
starting the other: Gazebo partitioning does not isolate ROS topics on the same
domain.

If an older Pixi executable cannot read the lockfile but the environment is
already installed, build directly with it:

```bash
export PATH="$PWD/.pixi/envs/default/bin:$PATH"
source .pixi/envs/default/local_setup.bash
colcon build --symlink-install
```

Rebuild after adding files or changing installed resources. Restart affected
nodes after changing Python code or configuration.

## Shared arena and targets

The authoritative arena resources are in `src/edgeai_robot_description/`:

| Resource | Used by |
| --- | --- |
| `worlds/rescue_arena.sdf` | Both midterm launches and the prepared-map generator |
| `models/rescue_target_S1/`, `models/rescue_target_C1/` | Both parts; one copy of each stand |
| `config/rescue_robots.yaml` | Robot starts in both parts |
| `config/rescue_targets.yaml` | Marker IDs/size, detection types and rescue centers/radii/holds |
| `urdf/edgeai_robot_real.urdf.xacro` | Robot geometry and sensors in both parts |
| `config/rescue_simulation_dds.xml` | ROS communication for all entry scripts |

The arena spans x/y = 0–5 m; +X is east and +Y is north. The start area is
x = 0–1.4 m, y = 0–1.0 m. Robot1 starts at (0.45, 0.45) and robot2 at
(1.00, 0.65), both facing north. Exact starts are team choices permitted by the
brief; both footprints fit without touching walls or one another.

| Target | ArUco ID | Center (m) | Printed face | Rescue condition |
| --- | --- | --- | --- | --- |
| S1 | 1 | (1.0, 4.1) | East, +X, toward the opening around W1 | One robot within 0.5 m for 10 consecutive simulation seconds |
| C1 | 2 | (4.1, 1.0) | North, +Y, toward the upper approach | Both robots simultaneously within 0.7 m for 10 consecutive simulation seconds |

Both markers use `DICT_4X4_50`, a **0.16 m black square**, a white margin and a
center height of **0.35 m**. C1 has one shared rescue circle; the brief does not
specify separate circles for each robot. The marker panel's local +Z is its
printed-face normal. Its 90° roll points south before the world yaw is applied;
S1's include yaw is π/2 (east) and C1's is π (north). These orientations are
identical in both parts. S1's east-facing orientation is the team's requested
choice; the task brief describes S1 facing the lower/open arena.

After changing marker dictionary, size or IDs, regenerate the shared stands:

```bash
ros2 run edgeai_robot_description generate_rescue_targets.py \
  --config src/edgeai_robot_description/config/rescue_targets.yaml \
  --models-dir src/edgeai_robot_description/models
```

Centers and rescue circles in the SDF must agree with the target YAML; regression
tests check that agreement. Regenerate the prepared map after any collision
geometry change, then rebuild and restart. The current reference map is checked
against the shared world.

[Arena reference](docs/arena_layout_reference.png) ·
[Gazebo view](docs/rescue_arena_top_down.png).

## Part 1: explore, map and detect

```bash
./start_part1.sh
```

This starts one robot, the camera detector, SLAM Toolbox, Nav2, the MRTSP frontier
explorer, the evidence recorder and RViz. SLAM is initially unconfigured and the
explorer is paused. In another terminal:

```bash
./part1_control.sh start
./part1_control.sh status
# Save before stopping the launch:
./part1_control.sh save
```

`start` activates fresh SLAM and Nav2, then starts recording and releases the
explorer. Motion goals come from live frontiers; Nav2 camera sweeps follow reached
frontiers. No saved map, scripted route or supplied target coordinate is used for
exploration or visual pose estimation. Call Start once per launch. Stop with
Ctrl+C and relaunch for a fresh run; old output directories are never loaded.
The saved-marker registry is also gated by Start and ignores pre-start detections.

RViz uses **Fixed Frame: `map`**. The map comes from `/map`. Enable
**Robot 1 ArUco Debug** for annotated camera images. **Robot 1 Saved Markers**
shows green panels with black ID labels that remain visible after leaving view.
For a manual display, add **MarkerArray** on
`/robot1/aruco/known_markers_visualization`, with **Reliable / Transient Local**.
Remembered positions are in `robot1/odom` and transformed into the live map by
RViz; they are estimates affected by odometry and SLAM.

Evidence goes to `runs/part1/<UTC timestamp>_<id>/`:

- `detections.jsonl`: every recorded successful target detection, ID, type, image
  timestamp and visually estimated camera-relative position in metres.
- `images/S1_*.png`, `images/C1_*.png`: annotated detection evidence.
- `timing_summary.json`: run timing and per-target counts/first/last observations.
- `map.yaml`, `map.pgm`: written by the Save command for ROS map loading.
- `map.png`: a lossless image copy of the saved map, written by the same command
  for viewing or sharing.

`runs/part1/latest_run.txt` points to the current run. To relocate output, launch
with `output_root:=/absolute/path` and use the same `PART1_OUTPUT_ROOT` for the
control script. Live evidence is also available on `/part1/target`,
`/part1/summary` and `/robot1/aruco/detections`. Visual observations use
`robot1/camera_optical_frame`: x right, y down, z forward.

## Part 2: autonomous rescue

Stop Part 1 before launching Part 2:

```bash
./start_part2.sh elapsed_before_part2:=360.0
```

**Replace 360.0 with the actual elapsed time, including the transition.** The
midterm allows 13 minutes combined, plus 2 minutes of verification; the roughly
6/7 minute split is guidance. Keep a single external stopwatch from the start of
Part 1 through the transition and Part 2. The launch argument supplies the
already-used portion of the 780-second budget; it does not automatically read
Part 1's timing file or measure the switch. Part 2's FSM counts simulation time
from mission initialization. Pauses and startup/transition overhead must also be
accounted for on the external timer. `elapsed_before_part2:=0.0` is for independent
Part 2 testing.

Part 2 starts two robots, a shared prepared `/map`, separate AMCL/Nav2 stacks,
peer obstacle handling, collision monitors, the mission FSM, verifier and RViz.
Robot2 goes to S1 while robot1 goes toward its C1 slot. After S1's verified hold,
robot2 uses Nav2's collision-checked backup and then navigates to C1. Rescue
completion comes from distance/continuous-time checks, independently of Nav2's
success result. Leaving the radius or losing fresh localization resets an
unfinished hold; C1 requires both robots at the same time.

The same URDF supplies geometry in both parts. Part 2 adds a wheel-axle
`drive_frame` in memory for Nav2 control, disables its unused RGB-D stream, and
measures rescue distances from the original `base_footprint` as required.

Useful options:

```bash
# Prepare without releasing the mission:
./start_part2.sh autostart_mission:=false
# Use a different team-prepared, arena-aligned map:
./start_part2.sh map_yaml:=/absolute/path/map.yaml
# Headless (also supported by start_part1.sh):
./start_part2.sh enable_rviz:=false gz_args:='-r -s --headless-rendering'
```

In an activated terminal:

```bash
ros2 service call /rescue/start std_srvs/srv/Trigger '{}'
ros2 topic echo /rescue/state
ros2 topic echo /rescue/verification
ros2 topic echo /rescue/progress
ros2 service call /rescue/stop std_srvs/srv/Trigger '{}'
```

Evidence goes to `runs/part2/run_<UTC timestamp>_<id>/`: `mission_events.jsonl`,
`mission_summary.json`, `verification.jsonl`, `verification_summary.json`, the
resolved mission configuration and map metadata. Verification logs contain robot
poses, target distances, continuous hold durations and completion flags. These
are AMCL/TF estimates; **physical collision status is not recorded by this
verifier**. Observe Gazebo for contact and complete a full run before claiming a
collision-free demonstration.

Part 2's prepared map is generated offline from the shared arena collision
geometry. Its resolution is 0.025 m and origin is (−0.25, −0.25); its `map` frame
matches the arena coordinates. Part 1's SLAM map has its own estimated origin and
orientation, so passing it to Part 2 without alignment is insufficient.

```bash
ros2 run edgeai_rescue generate_reference_map \
  --output src/edgeai_rescue/maps/rescue_reference
pixi run colcon build --symlink-install
```

Mission assignments/goals and tuning:
[Part 2 configuration](src/edgeai_rescue/config/mission.yaml),
[Nav2 configuration](src/edgeai_rescue/config/nav2.yaml),
[package guide](src/edgeai_rescue/README.md).

## Verification and limitations

After building and sourcing `setup_workspace.bash`:

```bash
python -m pytest -q src/edgeai_aruco/test src/edgeai_rescue/test \
  src/edgeai_robot_description/test/test_rescue_arena.py
```

These tests check marker patterns/orientations, arena dimensions, safe starts and
rescue slots, prepared-map consistency, start gating, mission transitions and
continuous rescue holds. They do not demonstrate complete autonomous coverage or
collision-free travel. The separate arena sensor smoke test is:

```bash
python src/edgeai_robot_description/test/verify_rescue_arena.py \
  --output-dir /tmp/edgeai_arena_check
```

That test moves/teleports only its own test robots to check sensors and marker
visibility. It is not a scored autonomous run. There is currently no automated
full Part 2 Gazebo mission test in this repository.

Part 1 wheel odometry may drift during collisions or slip; stop/reset a blocked
run. Detecting both targets depends on autonomous coverage and camera approach
angles. Before the demonstration, confirm both targets are observed, save the
map, inspect both continuous holds and check Gazebo for collisions. See the
[requirements review](docs/midterm_requirements_review.md) for the full assessment.

## Other development modes

`./start_sim.sh` launches the older single-robot `empty.sdf` sensor sandbox;
`enable_aruco:=true` enables its 0.20 m placeholder marker. This is separate from
the shared 0.16 m midterm arena. See the [ArUco guide](src/edgeai_aruco/README.md).

For arena sensor debugging, without a scored mission:

```bash
ros2 launch edgeai_robot_description rescue_arena.launch.py robot_count:=2
```

This uses a `world` fixed frame and static odometry anchors. The Part 1/2 launches
manage their own SLAM/localization transforms instead. Manual teleoperation and
operator-selected exploration goals are only for development; neither is allowed
during the assessed autonomous demonstrations.

## Repository layout

```text
setup_workspace.bash            Shared Pixi/ROS/DDS setup for every terminal
start_part1.sh / part1_control.sh   Part 1 launch, start, status and map save
start_part2.sh                  Part 2 launch
setup_part2.bash                Compatibility alias for shared setup
src/edgeai_robot_description/  Shared arena, stands, robot, starts and target specs
src/edgeai_aruco/              Camera perception, marker registry, Part 1 adapters
src/mrtsp_exploration_ros2/    Frontier exploration submodule
src/edgeai_rescue/             Part 2 mission, localization/Nav2 config, prepared map
runs/part1/ / runs/part2/       Separate run evidence (not loaded by Part 1)
docs/                          Arena references and midterm requirements review
```
