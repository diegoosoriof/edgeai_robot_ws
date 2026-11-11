# Part 2: autonomous rescue

Part of the [KTH EdgeAI workspace](../../README.md). Build the workspace once and
use the same environment and arena as Part 1:

```bash
pixi install
pixi run colcon build --symlink-install
./start_part2.sh elapsed_before_part2:=360.0
```

Replace `360.0` with the actual time already used by Part 1 and the transition.
For an independent Part 2 test, omit it. Stop Part 1 before starting Part 2.
All additional terminals use `source setup_workspace.bash`;
`source setup_part2.bash` is a compatibility alias.

This package owns the rescue logic, prepared map and Part 2 Nav2 tuning. It loads
`edgeai_robot_description/worlds/rescue_arena.sdf` and that package's target models,
start poses and target specifications. There are no separate Part 2 arena assets.
The reference map generator uses the same collision geometry.

## Launch options and commands

- `map_yaml:=/path/map.yaml map_image:=/path/map.pgm`: use an aligned team-prepared
  map; the image override is optional.
- `autostart_mission:=false`: prepare, then call `/rescue/start`.
- `elapsed_before_part2:=360.0`: account for prior task/transition time in the
  780-second budget. There is no automatic timer handoff; keep an external shared
  stopwatch, including any startup or pauses during the transition.
- `evidence_dir:=/path/output`: output root, default `runs/part2`.
- `initial_scan_angle:=0.35`: optional sequential collision-checked localization
  scans, disabled by default and limited to 0.5 rad.
- `enable_rviz:=false gz_args:='-r -s --headless-rendering'`: no GUI.
- `gz_partition:=name`: override the unique per-launch Gazebo partition.

```bash
source setup_workspace.bash
ros2 topic echo /rescue/state
ros2 topic echo /rescue/verification
ros2 topic echo /rescue/progress
ros2 service call /rescue/start std_srvs/srv/Trigger '{}'
ros2 service call /rescue/stop std_srvs/srv/Trigger '{}'
```

## Mission and verification

`config/mission.yaml` defines assignments, approach goals, timeouts and localization
thresholds. `configuration.py` merges it with shared
`edgeai_robot_description/config/rescue_robots.yaml` and `rescue_targets.yaml`.
A custom mission file may explicitly override those defaults. Such overrides must
still satisfy the midterm arena and rescue requirements.

Robot2 goes to S1 while robot1 goes to its C1 slot. Robot2 holds at S1 until the
verifier confirms 10 consecutive simulation seconds within 0.5 m; it then backs
out 0.4 m using Nav2 collision checks and plans its route to C1. C1 requires both
robots within 0.7 m simultaneously for 10 consecutive simulation seconds.
Peer footprints enter both costmaps and Collision Monitor. Velocity locks stop
robots on invalid localization, during holds, or while idle/failed.

Nav2 tuning is in `config/nav2.yaml`. Footprints include the URDF's caster
clearance. Part 2 adds a wheel-axle `drive_frame` to the shared URDF in memory;
the original `base_footprint` remains the rescue-distance reference. The 8.75 cm
offset is derived from the URDF, not added to the target coordinates.

The verifier uses fresh AMCL/LiDAR/TF and simulation time, independently of Nav2
results. An unfinished hold resets when a required robot leaves the radius or
observations become invalid. Pausing simulation adds no hold time. A backward
clock jump clears verification and aborts the mission. Restart after a clock reset.
RViz uses `map`; `/rescue/markers` shows rescue circles, centers and black target
labels. These are predefined rescue goals, rather than camera detections.

Evidence is written under a new `runs/part2/run_.../` directory:
`verification.jsonl`, `verification_summary.json`, `mission_events.jsonl`,
`mission_summary.json`, resolved mission config and map metadata. Distance and
hold evidence uses AMCL/TF estimates, not Gazebo ground truth. Physical contact
status is not recorded by this verifier.

## Prepared map and checks

The default map is generated offline from the shared arena, with resolution
0.025 m and origin (−0.25, −0.25). Regenerate after changes to collision geometry:

```bash
ros2 run edgeai_rescue generate_reference_map \
  --output src/edgeai_rescue/maps/rescue_reference
pixi run colcon build --symlink-install
python -m pytest -q src/edgeai_rescue/test
```

Tests check the FSM, continuous holds, shared configuration, map consistency,
collision-free starts and separated in-radius rescue goals. A full autonomous
Gazebo mission and collision evidence still need demonstration; no automated full
Part 2 simulation test is currently included. See the
[requirements review](../../docs/midterm_requirements_review.md).
