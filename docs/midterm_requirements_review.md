# Midterm requirements review

Source: *KTH EdgeAI Robotics Challenge — Midterm Check, Task Description*, PDF
provided by the team, proposed date November 12, 2026. Section references below
refer to that document. This is an implementation review and regression-test
record, not a certificate that an autonomous simulation run has completed.

## Changes made after merging the two parts

- Both launches now use `edgeai_robot_description/worlds/rescue_arena.sdf` and
  its target models. Duplicate Part 2 worlds/models were removed. The offline
  prepared-map generator also reads these shared assets.
- S1's printed face points east (+X) in both parts, as requested by the team.
  C1 remains north (+Y). The brief describes S1 facing the lower/open arena;
  the east-facing orientation is a deliberate project choice.
- Robot starts and target IDs/types/centers/radii/hold durations have shared
  configuration. Robot2 uses the Part 2 start (1.00, 0.65) in both modes;
  robot1 remains (0.45, 0.45). Part 2 retains its own assignments, goals and
  navigation tuning.
- The terminal setup and DDS profile are shared. Part 1/2 have matching entry
  scripts and evidence roots (`runs/part1`, `runs/part2`). Each has a separate
  Gazebo partition; run them sequentially on the same ROS domain.
- Part 1's saved-marker registry waits for Start, clears its state at that point,
  and rejects delayed images from before Start. The evidence recorder also
  ignores pre-start observations. Live image previews may run before Start.
- Target labels are black in both RViz views. The Part 2 package guide no longer
  refers to an absent full-mission simulation test or to an old branch/commit.

## Requirement-by-requirement assessment

| Requirement | Implementation/evidence | Assessment |
| --- | --- | --- |
| §2–3: approximately 5 × 5 m arena; specified W1/W1b, W2, O2/O3 and start area | Shared SDF; `test_rescue_arena.py` checks bounds, dimensions, connected walls and absence of extra colliding obstacles | Checked statically |
| §3: freestanding vertical targets, IDs 1/2, `DICT_4X4_50`, recommended 0.16 m size and 0.35 m height | Shared generated stand models, YAML and marker generator; tests decode both patterns and check combined world/panel face normals | Dimensions and configured directions checked statically. S1 faces east by team request, differing from the brief's lower/open direction; visibility along the actual autonomous route still needs demonstration |
| §2, §7: collision-free starts within lower-left area | Shared starts; tests project URDF collision envelopes and check map clearance/separation | Checked statically for both robots |
| §4.1: at least one robot autonomously explores using onboard sensors | Part 1 uses live SLAM/Nav2 and MRTSP frontiers, with collision-checked Nav2 camera sweeps; no supplied target coordinates enter frontier planning | Implemented; full coverage and detection of both targets in one fresh run remain runtime checks |
| §4.1–4.2: clear earlier mapping, exploration and target results | New processes per launch, initially inactive SLAM and gated explorer/recorder/registry; no map deserialization or prior evidence loading | Implemented; restart the launch for each scored run, call Start only once |
| §4.1: record ID, type, visually estimated relative pose and timestamp for both S1/C1 | `part1_record.py`; `/part1/target`, `/robot1/aruco/detections`, JSONL and annotated PNGs | Implemented; inspect both targets' evidence in the current demonstration |
| §4.1, §6.1: display and save the resulting map | RViz `/map`; `part1_control.sh save` writes map YAML/PGM and a PNG image to the current run | Implemented; saving is an explicit operator action before shutdown |
| §5: Part 2 uses two robots, a team-prepared map and known target coordinates; no repeat vision required | Two AMCL/Nav2 stacks, shared prepared geometry-derived map, explicit S1/C1 assignments | Implemented; a prepared map independent of Part 1 is allowed |
| §5: S1, one robot center within 0.5 m for 10 consecutive simulation seconds | `RescueVerifier`, continuous-hold tests and FSM; distance uses `base_footprint` | Logic tested, including leaving the radius and the inclusive distance boundary |
| §5: C1, both robot centers simultaneously within 0.7 m for 10 consecutive simulation seconds | Joint eligibility and one continuous timer; separated configured C1 goals | Logic and static slot geometry tested; joint arrival and physical holds need a full run |
| §5: reset unfinished hold when a required robot leaves | Verifier also resets on stale/missing localization or a long observation gap | Logic tested; a simulation pause contributes zero hold time and a backward clock reset clears results |
| §5, §7: autonomous navigation, task-dependent assignment and transitions; no teleoperation/playback | Nav2 actions and explicit mission FSM; peer obstacles, recovery and mission locks | Logic tested; successful collision-free travel is not established by unit tests |
| §5, §7: avoid contact and provide collision status | Nav2 costmaps, footprint checks, peer point clouds and Collision Monitor help prevent contact | **Incomplete evidence:** runtime verifier records AMCL/TF distances, not physical contact status; inspect/record Gazebo and add a contact/ground-truth trace if machine-readable collision evidence is needed |
| §4.2, §7: shared 13-minute task time, including switch and initialization; 10-second holds use simulation time | Part 1 timing summary; Part 2 780 s budget and `elapsed_before_part2` input; hold clocks use simulation time | **Manual handoff:** keep one external stopwatch and supply the actual prior elapsed time. No automatic timer spans launch shutdown/restart; startup during the switch and paused simulation must still count on the external timer |
| §6: present map, target evidence, rescue flags and distance/time history | Saved map/images/JSONL; `/rescue/verification`, `/rescue/progress`, mission and verification summaries | Implemented outputs; collect a complete demonstration record and collision evidence |
| §7: LLM inference and a fully integrated explore–rescue pipeline are optional at midterm | Parts remain independent, sharing physical resources rather than forcing a SLAM-map handoff | Consistent with the brief |

## What the automated checks establish

Run from the workspace root after building and sourcing `setup_workspace.bash`:

```bash
python -m pytest -q src/edgeai_aruco/test src/edgeai_rescue/test \
  src/edgeai_robot_description/test/test_rescue_arena.py
```

The suite checks arena/marker geometry, decoded IDs, start separation, matching
prepared-map pixels, free rescue goals, shared defaults and custom overrides,
pre-start registry behavior, mission transitions/recovery and the simultaneous
continuous hold rules. Map equality is checked by regenerating into a temporary
directory rather than changing the packaged map during a test.

Validation of this merge: all three affected packages built with symlink install;
**32 tests passed**; both installed launch descriptions and their node parameters
loaded successfully. The two-robot Gazebo arena smoke test also passed, checking
all sensor streams, independent physical movement, TF, correct shared start poses,
and both rendered IDs with camera-relative pose checks, before the subsequent
east-facing S1 change. Its S1 test approach was
moved beyond W1b so the actual low-mounted camera sees the whole printed pattern;
the old close pose clipped it. Expected camera positions now come from the actual
mount transform. Artifacts from that run are in
`/tmp/edgeai_shared_arena_check/` (`result.json`, camera images and top-down view).

The existing `verify_rescue_arena.py` is an isolated sensor/visibility smoke test.
It teleports its own test robots and is not valid autonomous mission evidence.
There is no full autonomous Part 2 Gazebo test in this repository. Do not infer
successful localization/navigation, collision-free trajectories or a compliant
13-minute demonstration from static tests alone.

## Demonstration procedure still required

1. Start Part 1 with a fresh launch; keep robot1 in the start area. Start a single
   shared task stopwatch when autonomous exploration is released.
2. Allow only automatic exploration goals. Confirm both S1 and C1 have current-run
   visual relative-position evidence and annotated images. Confirm the map shows
   recognizable perimeter/internal walls and obstacles; save it before shutdown.
3. Keep the stopwatch running through the switch. Stop Part 1, start Part 2 on
   the aligned prepared map and supply the actual elapsed Part 1/transition time.
   Budget calculations in the FSM use simulation time; the external stopwatch is
   the overall timing record when there are pauses or additional startup delays.
4. Observe contact/collision status in Gazebo. Confirm S1's verified hold is at
   least 10 continuous simulation seconds, then confirm both robots' C1 distances
   remain within 0.7 m simultaneously for at least 10 continuous simulation seconds.
5. Finish within 13 minutes combined; use the separate 2-minute verification
   window to show the saved Part 1 map, both detections, distance histories,
   completion flags, hold times, overall timing and collision evidence.

Known limitations: wheel odometry can drift during contact/slip, AMCL/TF estimates
are not Gazebo ground truth, Part 1's SLAM frame is not automatically aligned with
Part 2's arena frame, and the external timer/collision demonstration steps remain
necessary. Arena geometry and nominal target centers were preserved; S1's
orientation is east in both parts by team request.
