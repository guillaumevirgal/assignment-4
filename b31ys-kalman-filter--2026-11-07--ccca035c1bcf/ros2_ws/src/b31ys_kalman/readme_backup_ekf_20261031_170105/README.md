# B31YS Kalman Filter Assignment

**ROS 2 Humble | The Construct | ROSBOT 2**

In this assignment, you will implement a Kalman filter to estimate the position and orientation of ROSBOT 2 using velocity commands and noisy sensor measurements.

## Choose your assignment

| Degree | Instructions | Required measurements |
|---|---|---|
| BSc / BEng | [BSc/BEng Assignment](students/bsc/README.md) | GPS |
| MSc | [MSc Assignment](students/msc/README.md) | GPS and noisy odometry |

Both groups use the same simulation and Python template. Read the instructions corresponding to your degree.

## Available topics

| Topic | Description |
|---|---|
| `/cmd_vel` | Linear and angular velocity commands |
| `/gps` | Simulated GPS position |
| `/odom1` | Noisy and drifting odometry |
| `/odom` | Reference wheel odometry |
| `/kalman/odom` | Student Kalman estimate |
| `/imu/data` | IMU measurements |

The GPS provides coordinates in metres in the local odometry frame.

The `/odom` reference is derived from simulated wheel odometry and is not independent Gazebo ground truth.

## Launch instructions

Open separate terminals for each process.

### Terminal 1: ROSBOT

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch rosbot_gazebo simulation.launch.py \
  robot_model:=rosbot \
  rviz:=false \
  gz_headless_mode:=True
```

### Terminal 2: Assignment sensors

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch b31ys_kalman robot.launch.py
```

### Terminal 3: Kalman assignment

**BSc/BEng:**

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch b31ys_kalman kalman_execution.launch.py \
  student_name:=YourName \
  level:=beng
```

**MSc:**

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch b31ys_kalman kalman_execution.launch.py \
  student_name:=YourName \
  level:=msc
```

Replace `YourName` with your own name.

### Terminal 4: RViz

```bash
source ~/ros2_ws/install/setup.bash

rviz2 \
  -d ~/ros2_ws/src/b31ys_kalman/config/b31ys_assignment.rviz \
  --ros-args -p use_sim_time:=true
```

The fixed frame should be `odom`.

### Terminal 5: Gazebo GUI

Keep the simulator running in Terminal 1, then open the graphical client:

```bash
ign gazebo -g
```

Do not start a second simulation.

## Kalman filter template

Edit:

`b31ys_kalman/kalman_filter_template.py`

Implement the prediction and correction operations required by your degree.

Rebuild after editing:

```bash
cd ~/ros2_ws
colcon build --packages-select b31ys_kalman
source install/setup.bash
```

The trajectory generator follows a deterministic, continuously repeating trajectory based on your name.

## Submission requirements

All students must submit:

1. **A PDF plot** comparing GPS, Kalman estimate and reference trajectory. Use an X-Y plot or a position-versus-time plot.
2. **An RViz screenshot** showing the estimation running.
3. **The completed Python code** implementing the Kalman filter.

## Detailed instructions

- [BSc/BEng README](students/bsc/README.md)
- [MSc README](students/msc/README.md)
