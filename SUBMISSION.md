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
| 3. Localisation and path following: reaches goals without collisions | 25% | CW1 path follower works (3 goals reached with a static map to odom TF). Localisation not started |
| 4. Integration and code quality: runs from one launch file, documented | 15% | Not started |
| Report (max 4 pages PDF) | 20% | Not started. Material collected below |

## Deliverables checklist

- [x] Saved map `maps/my_map.yaml` + `.pgm`, built with teleop
- [x] PRM planning on that map (Part 2)
- [ ] Localisation in the saved map: AMCL or slam_toolbox localisation mode (Part 3)
- [ ] Path following to the goal without collisions (Part 3)
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

- [ ] Localisation: AMCL (nav2_amcl) or slam_toolbox in localisation mode, on the saved map
- [ ] Replace the static `map` to `odom` transform (`static_map_odom:=false`)
- [ ] Path follower: CW1 `path_follower.py` is copied in; check it works on this robot, tune speeds/tolerances
- [ ] Collision-free check over several runs

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

## Not part of the submission

- `kalman_filter_template.py` / `kalman_filter_template 1.py`, `fake_sensors`,
  `student_trajectory`: from the previous (Kalman filter) assignment
- Test map and explorer script in Claude's scratch folder: testing only
