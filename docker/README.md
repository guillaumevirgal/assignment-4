# Running CW2 in Docker

ROS 2 Humble + Gazebo + the ROSbot simulation + the `b31ys_kalman` package,
with slam_toolbox and the nav2 map server also installed.

## First time

If the repo was cloned from GitHub, fetch the Husarion packages too:

```bash
git clone --recursive https://github.com/guillaumevirgal/assignment-4.git
# or, in an existing clone:
git submodule update --init
```

Then build the image (10-20 minutes the first time):

```bash
cd docker
docker compose build
```

## Each time

```bash
cd docker
xhost +local:docker            # lets RViz / Gazebo open windows
docker compose up -d           # start the container
docker compose exec ros bash   # one shell per terminal you need
```

Inside the container (ROS is already sourced):

```bash
# Terminal 1: simulation
ros2 launch rosbot_gazebo simulation.launch.py robot_model:=rosbot rviz:=false gz_headless_mode:=True

# Terminal 2: assignment sensors
ros2 launch b31ys_kalman robot.launch.py

# Terminal 3: Kalman filter + trajectory
ros2 launch b31ys_kalman kalman_execution.launch.py student_name:=YourName level:=beng   # or level:=msc

# Terminal 4: RViz (fixed frame: odom)
rviz2 -d /ros2_ws/src/b31ys_kalman/config/b31ys_assignment.rviz --ros-args -p use_sim_time:=true

# Terminal 5: Gazebo GUI (attaches to the running simulation)
ign gazebo -g

# Plot the results (writes kalman_plot.pdf and kalman_data.csv to /output)
python3 /ros2_ws/src/b31ys_kalman/scripts/plot_trajectories.py

# Optional: mapping
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true
ros2 run nav2_map_server map_saver_cli -f /output/husarion_world_map
```

Stop with `docker compose down`.

## Files

- Anything saved in `/output` inside the container appears in `docker/output/` on
  your machine (CSV files, plots, saved maps). The shell starts in `/output`.
- The code is copied into the image at build time. After editing code, run
  `docker compose build` again, then `docker compose up -d`.
