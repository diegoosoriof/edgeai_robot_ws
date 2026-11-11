# Part 2 — autonomous rescue

```bash
pixi install
pixi run colcon build --symlink-install --packages-select edgeai_robot_description edgeai_rescue
source setup_part2.bash
ros2 launch edgeai_rescue rescue_part2.launch.py
```

Gazebo + shared `/map` + independent AMCL/Nav2 + collision monitoring + rescue FSM + verifier + RViz.
Part 1 and `start_sim.sh` are independent. This branch's arena assets are reused
from `markers_detection` (commit `6f6d590`); official midterm marker recommendations
are 0.16 m square at 0.35 m height, IDs 1 and 2.

Useful launch arguments:

- `map_yaml:=/path/map.yaml map_image:=/path/map.pgm` — use the team's prepared map.
- `autostart_mission:=false` — prepare the system, then call `/rescue/start`.
- `elapsed_before_part2:=360.0` — charge Part 1/transition against the shared 780 s budget.
- `initial_scan_angle:=0.35` — optional sequential, collision-checked localization scans (default disabled).
- `enable_rviz:=false gz_args:="-r -s --headless-rendering"` — no GUI.

The default reference map is generated offline from the arena's collision geometry,
not by SLAM. Its origin is (-0.25,-0.25), resolution 0.025 m; arena/target coordinates
are unchanged in `map`. Replace it with a geometrically aligned saved map as needed.

In other terminals, run `source setup_part2.bash` first:

```bash
ros2 topic echo /rescue/state
ros2 topic echo /rescue/verification
ros2 service call /rescue/start std_srvs/srv/Trigger '{}'
ros2 service call /rescue/stop std_srvs/srv/Trigger '{}'
```

Goals, thresholds, timers and spawn poses: `config/mission.yaml`.
Nav2 tuning: `config/nav2.yaml`. Footprints come from URDF collisions, including caster
clearance. Nav2 controls the wheel-axle `drive_frame`; goals and rescue distances use
the original `base_footprint`. Their 8.75 cm offset is derived from the URDF.
C1 slots are approached sequentially.
After S1, Nav2 backs out 0.4 m with collision checks before turning toward C1.
Peer footprints enter both costmaps and Nav2 Collision Monitor; twist-mux locks stop
robots on invalid localization, while holding, or when the mission is idle/failed.

The verifier uses AMCL `map -> robotN/odom -> robotN/drive_frame -> robotN/base_footprint`, fresh
LiDAR/AMCL/TF, and simulation time. Leaving a radius or losing valid observations
resets its continuous timer. A pause adds no hold time; a backward clock jump clears
verification and aborts the mission. Start a new mission after correcting/resetting.

Evidence: a new `rescue_evidence/run_.../` directory with `verification.jsonl`,
`verification_summary.json`, `mission_events.jsonl` and `mission_summary.json`.
These estimates are independent of Nav2 success but are not Gazebo ground truth.

```bash
env -u PYTHONPATH pixi run colcon test --packages-select edgeai_rescue
pixi run colcon test-result --test-result-base build/edgeai_rescue --verbose
python src/edgeai_rescue/test/verify_part2_simulation.py --rviz
```

The simulation test uses its own ROS domain/Gazebo partition and independently
checks Gazebo poses, ten-second holds, map alignment and sampled footprint overlaps.
