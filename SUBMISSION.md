# Assignment 4: submission tracker

Hospital robot in a new ward: build a map (SLAM), plan on it (PRM), localise
and follow the path, integrate everything in one launch file. Brief:
`b31ys_kalman/b31ys_kalman/INSTRUCTIONS.md`.

All package paths below are relative to
`b31ys-kalman-filter--2026-11-07--ccca035c1bcf/ros2_ws/src/b31ys_kalman/`.
How to run everything in Docker: `docker/README.md`.

## Marking scheme

| Part | Weight | Status |
|---|---|---|
| 1. SLAM: does the map represent the environment well? | 15% | **Done**: `maps/my_map` built with teleop, complete and clean |
| 2. PRM on the SLAM map: finds a solution, is PRM, keeps a safe distance | 25% | **Done and tested** on `my_map`, in simulation. Report figures/table to do |
| 3. Localisation and path following: reaches goals without collisions | 25% | **Done and tested**: 6/6 goals reached, no collisions. Screenshot to do |
| 4. Integration and code quality: runs from one launch file, documented | 15% | Not started |
| Report (max 4 pages PDF) | 20% | Not started. Material collected below |

## Deliverables checklist

- [x] Saved map `maps/my_map.yaml` + `.pgm`, built with teleop
- [x] PRM planning on that map (Part 2)
- [x] Localisation in the saved map: AMCL (Part 3)
- [x] Path following to the goal without collisions (Part 3)
- [ ] One launch file: localisation + planner + path follower + RViz (Part 4)
- [ ] Demo: robot reaches at least 3 goals (patient rooms) one after the other (Part 4)
- [ ] Video of the robot working (or live demo)
- [ ] Group report, max 4 pages PDF (contents below)
- [ ] One submission per group
- [ ] Individually: peer assessment form
- [ ] Individually: statement of own contribution, max 200 words

## Report contents checklist

- [ ] Diagram of the system architecture (nodes, topics, TF tree)
- [ ] SLAM package used and the parameters changed (see Part 1)
- [ ] What was changed in the PRM to work on the SLAM map (see Part 2)
- [ ] Results: success rate, path length, planning time, number of nodes
      (collected automatically in `prm_results.csv`, see Part 2)
- [ ] What failed and how it was fixed (see the log at the end)
- [ ] Table with each member's contribution

---

## Part 1: Mapping (slam_toolbox)

**How to build the map** (inside the container):
```bash
ros2 run b31ys_kalman build_map.sh husarion_world_map
```
Drive with the keyboard (`i` `,` `j` `l` `k`), cover every room, come back to
the start to close the loop, Ctrl+C to save. Then copy
`docker/output/husarion_world_map.{yaml,pgm}` into the package's `maps/`
folder and rebuild the image.

**Files**

| File | Purpose |
|---|---|
| `scripts/build_map.sh` | Starts sim + SLAM + RViz, runs keyboard teleop, saves with the Nav2 map saver |
| `launch/mapping.launch.py` | Simulation + slam_toolbox (online async) + RViz |
| `config/slam_params.yaml` | slam_toolbox parameters |
| `config/mapping.rviz` | RViz view: map, laser, robot |

**Parameters changed from slam_toolbox's defaults** (`mapper_params_online_async.yaml`):

| Parameter | Default | Ours | Why |
|---|---|---|---|
| `base_frame` | `base_footprint` | `base_link` | The ROSbot has no `base_footprint` frame; with the default, slam_toolbox gets no robot pose and builds nothing |
| `scan_topic` | `/scan` | `/scan_filtered` | Lidar scan with the robot's own body removed by `laser_filters`, so the robot does not map itself |
| `use_sim_time` | (unset) | `true` | Gazebo clock, otherwise TF lookups fail on timestamps |

Map saved with the map saver's defaults: resolution 0.05 m, trinary mode,
occupied threshold 0.65, free threshold 0.25.

**Map used:** `maps/my_map.yaml` + `.pgm` (built with teleop, 24 x 24 m,
481 x 481 cells). `docker/output/husarion_world_map` is an earlier partial
attempt, not used.

**To do**
- [x] Build the real map with teleop
- [ ] Screenshot of the final map for the report

## Part 2: PRM on the SLAM map

**How to run**
```bash
ros2 launch b31ys_kalman prm_planner.launch.py sim:=true results_csv:=/output/prm_results.csv
```
Then click **2D Goal Pose** in RViz. Optional: `follow:=true` also runs the
CW1 path follower; `goal_x:=... goal_y:=...` plans one goal at start-up.

**Files**

| File | Purpose |
|---|---|
| `b31ys_kalman/prm_core.py` | PRM algorithm (CW1 code, changes marked `CW2:`) |
| `b31ys_kalman/prm_planner.py` | ROS node around it |
| `launch/prm_planner.launch.py` | Map server + planner + RViz (+ sim, + follower) |
| `config/prm.rviz` | Map, safe area, roadmap, path, 2D Goal Pose tool |

**What was changed in the CW1 PRM, and why** (for the report)

| # | Change | Why it is needed |
|---|---|---|
| 1 | Import from `b31ys_kalman` instead of `b31ys_path_planning` | The code now lives in this package |
| 2 | Map read from a saved SLAM map through `map_server` | CW1 used a hand-made map of a different world. The PRM must plan on the map we built |
| 3 | Three kinds of cells handled separately: walls inflated by robot radius + safety margin; unknown areas inflated by robot radius; tiny unknown specks removed | A SLAM map is not clean. In CW1 every non-free cell was an obstacle. On the SLAM map, thin "fingers" of free space where single laser rays leaked through gaps were treated as usable: samples landed in them and could not connect. **26% of test queries failed; after the change 0 of 200.** Inflating unknown areas also keeps the whole robot on explored ground |
| 4 | Safety margin in metres (`safety_margin`, default 0.10 m) on top of the robot radius (0.18 m), instead of half a cell | Marking criterion "keeps a safe distance from obstacles". Verified: no allowed position is closer than 0.28 m to a wall |
| 5 | Start = robot's current pose from TF (`map` to `base_link`) instead of fixed (0, 0) | Part 4 visits several rooms in a row: each plan must start where the robot is |
| 6 | Goals from RViz "2D Goal Pose" (`/goal_pose`) or launch arguments, instead of a goal generated from the student name | Goals are now chosen patient rooms |
| 7 | Multi-query PRM: roadmap built once per map, reused for every goal | This is how PRM is meant to be used (build once, query many times), and it makes each new goal fast |
| 8 | If no path: rebuild with 2x then 4x samples and a new seed | Raises the success rate in narrow areas |
| 9 | Start/goal snapped to the nearest safe cell (within 0.6 m) | The robot can stop close to a wall, or a clicked goal can be near furniture; without this the query fails |
| 10 | Path shortcutting: skip waypoints when the straight line is collision-free | PRM paths zig-zag between random samples. Shorter, smoother path for the follower, same safety distance |
| 11 | k-nearest neighbours with numpy | The SLAM map is much larger than the CW1 map; the pure-Python loops were slow |
| 12 | Statistics logged and saved to CSV (`results_csv`) | Report needs success rate, path length, planning time and number of nodes |
| 13 | `/prm_free_space` published | Shows the safe area in RViz: a figure for the report on the safety distance |

Unchanged from CW1: free-space sampling, edge collision check (every half
cell), k-nearest roadmap construction, A* search.

**Test results so far** (800 samples, k = 12, radius 0.18 m + margin 0.10 m)

On `my_map`, core only: roadmap built in 0.13 s (800 nodes, about 5400
edges; 4916 edges in the simulation run, which loads the map through
map_server); 200/200 random start/goal pairs solved; mean path 14.2 m; mean
query time 3 ms. No allowed robot position closer than 0.28 m to a wall.

In simulation (planner + CW1 path follower, static map to odom):

| Goal | From | Path length (shortcut / roadmap) | Waypoints (shortcut / roadmap) | Planning time | Reached |
|---|---|---|---|---|---|
| (-8, -8) | spawn (0, 0) | 12.96 m / 15.62 m | 3 / 11 | 1.7 ms | yes |
| (-8, 8) | (-7.71, -7.82) | 15.91 m / 16.42 m | 3 / 15 | 1.9 ms | yes |
| (3, -3) | (-8.04, 7.66) | 22.27 m / 23.26 m | 5 / 19 | 6.3 ms | yes |

Shortcutting shortened the paths by 2 to 17%. Each plan started from the
robot's actual position.

**To do**
- [x] Test the full node in simulation on a complete map
- [ ] Results table for the report from `prm_results.csv` (several goals, several seeds)
- [ ] Screenshot: roadmap + path + safe area in RViz

## Part 3: Localisation and path following

**How to run**
```bash
ros2 launch b31ys_kalman navigation.launch.py sim:=true results_csv:=/output/prm_results.csv
```
In RViz: if the robot did not start at the map origin, set its pose with
**2D Pose Estimate**; then click **2D Goal Pose**. The PRM plans from the
localised pose and the follower drives there.

**Files**

| File | Purpose |
|---|---|
| `launch/localization.launch.py` | map_server (saved map) + AMCL + lifecycle manager |
| `config/amcl_params.yaml` | AMCL parameters |
| `launch/navigation.launch.py` | Sim (optional) + localisation + PRM planner + path follower + RViz |
| `config/navigation.rviz` | prm.rviz + AMCL pose, 2D Pose Estimate tool |
| `b31ys_kalman/path_follower.py` | CW1 follower, with CW2 additions |

**Localisation: AMCL (nav2_amcl).** Particle filter on the saved map, using
the lidar and the robot's odometry (`odom` to `base_link` from the ROSbot's
EKF); publishes `map` to `odom`. Replaces the static transform used in Part 2.

AMCL parameters changed from the Nav2 defaults:

| Parameter | Default | Ours | Why |
|---|---|---|---|
| `base_frame_id` | `base_footprint` | `base_link` | The ROSbot has no `base_footprint` |
| `scan_topic` | `scan` | `/scan_filtered` | Scan without the robot's own body |
| `set_initial_pose` + `initial_pose` | false | true, (0, 0, 0) | The map was started at the spawn point, so the robot starts at the map origin: no manual pose needed |
| `laser_max_range` | 100 | 12 m | Reliable lidar range |
| `update_min_d` / `update_min_a` | 0.25 m / 0.2 rad | 0.10 m / 0.10 rad | More frequent updates, smoother pose while following |

**Path follower: CW1 proportional controller**, kept: rotate towards the next
waypoint, drive when the heading error is below 0.35 rad, speed
`min(0.25, 0.5 x distance)`, waypoint tolerance 0.15 m.

CW2 additions:

| Change | Why |
|---|---|
| Lidar safety stop: no forward motion if an obstacle is closer than 0.30 m within +/-35 deg ahead; slow down from 0.60 m | "Reaches the goal without collisions", also if localisation drifts or something is not on the map |
| `/path_follower/status` (idle / following / blocked / reached) | Part 4 needs to know when a room is reached to send the next goal |
| No `/cmd_vel` messages while idle (CW1 sent stop commands continuously) | Teleop and the follower would fight over the robot |
| An empty path (planner failed) stops the robot | Otherwise it would keep following the old path |
| TF listener on its own thread | Lookups inside callbacks could stall |

**Test results** (simulation, map `my_map`, two runs of the same 3 goals in a
row, starting at the spawn point. "True" positions from Gazebo, converted to
the map frame: the robot spawns at world (-1, -2), which is the map origin.)

| Goal | Reached | Time (run 1 / run 2) | Final true distance to goal | AMCL pose vs true pose* | Closest lidar range** | Safety stops |
|---|---|---|---|---|---|---|
| (-8, -8) | 2/2 | 63 s / 64 s | 0.066 m | 0.10 m | 0.27 m / 0.31 m | 0 |
| (-8, 8) | 2/2 | 75 s / 77 s | 0.037 m | 0.17 m | 0.31 m / 0.26 m | 0 |
| (3, -3) | 2/2 | 110 s / 109 s | 0.034 m | 0.11 m | 0.35 m / 0.34 m | 0 |

\* Last `/amcl_pose` message. AMCL only publishes after moving 0.10 m or
turning 0.10 rad, so this estimate is from slightly before the robot
stopped and overstates the error. The follower uses the continuously
updated `map` to `base_link` TF, which is why the robot ends 3 to 7 cm from
the goal (tolerance 0.15 m).

\*\* Closest obstacle seen by the lidar during the trip. The lidar is at the
robot's centre and the robot's half-length is about 0.12 m, so 0.26 m is
roughly 0.14 m between the robot body and the nearest obstacle: no
collisions. Path clearance planned: 0.28 m (robot radius 0.18 + margin
0.10).

Success rate: 6/6 goals reached, 0 collisions, 0 safety stops.

**To do**
- [x] Results table for the report (above)
- [ ] Screenshot: AMCL pose + path in RViz

## Part 4: Integration

- [ ] Single launch file: localisation + planner + path follower + RViz
- [ ] Sequence of at least 3 goals (patient rooms), one after the other
- [ ] Record the video

---

## What failed and how we fixed it (for the report)

| Problem | Cause | Fix |
|---|---|---|
| Workspace would not build from scratch | `setup.py` listed README files that did not exist (`students/bsc/README.md` never existed) | Removed those entries |
| Gazebo did not start in Docker (`ros_gz_bridge` not found) | Husarion's packages only declare Gazebo dependencies when `HUSARION_ROS_BUILD_TYPE=simulation` | Set that variable in the Dockerfile |
| `rosbot_controller` would not build | Needs `mecanum_drive_controller` from `husarion_controllers` | Added it to the build |
| slam_toolbox default config would not work | Expects `base_footprint`, which the ROSbot does not have | `base_frame: base_link` |
| 26% of PRM queries failed on the SLAM map | Thin free-space "fingers" from laser rays leaking into unexplored space; samples there could not connect | Unknown areas inflated by the robot radius, tiny unknown specks removed (change 3). 0/200 failures |
| AMCL error looked like 10-17 cm while the robot stopped 3-7 cm from the goal | `/amcl_pose` is only published after 0.10 m / 0.10 rad of motion, so the last message is stale | Measure with the `map` to `base_link` TF (what the follower uses) and Gazebo's true pose |

## Not part of the submission

- `kalman_filter_template.py` / `kalman_filter_template 1.py`, `fake_sensors`,
  `student_trajectory`: from the previous (Kalman filter) assignment
- Test map and explorer script in Claude's scratch folder: testing only
