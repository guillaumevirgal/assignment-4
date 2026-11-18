# B31YS Extended Kalman Filter (EKF) Assignment: MSc


> **Why an Extended Kalman Filter?**
>
> The robot's motion model is nonlinear because its position
> depends on `cos(theta)` and `sin(theta)`.
> An Extended Kalman Filter (EKF) handles this nonlinearity
> by linearising the motion model around the current state
> estimate using a Jacobian matrix.
>
> You must implement the state prediction, covariance
> propagation and the required measurement corrections.
> The EKF equations are not provided as executable solutions.


[Return to the main README](../../README.md)

**Your task is to implement prediction and correction using both GPS and noisy odometry.**

This assignment uses ROSBOT 2 in ROS 2 Humble on The Construct.

The objective is to estimate the robot state:

    state = [x, y, theta]

where `x` and `y` are positions in metres and `theta` is heading in radians.

## Start the simulation

Open separate terminals.

**Terminal 1: ROSBOT**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch rosbot_gazebo simulation.launch.py \
  robot_model:=rosbot rviz:=false gz_headless_mode:=True
```

**Terminal 2: Sensors**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch b31ys_kalman robot.launch.py
```

**Terminal 3: Assignment**

Use the launch command given in your degree-specific section below.

**Terminal 4: RViz**

```bash
source ~/ros2_ws/install/setup.bash
rviz2 \
  -d ~/ros2_ws/src/b31ys_kalman/config/b31ys_assignment.rviz \
  --ros-args -p use_sim_time:=true
```

Set Fixed Frame to `odom`.

**Terminal 5: Gazebo graphical interface**

```bash
ign gazebo -g
```

Keep the original headless simulation running. Do not start another one.

## Available measurements

| Topic | Description |
|---|---|
| `/cmd_vel` | Commanded linear and angular velocities |
| `/gps` | Low-frequency GPS position, in local metres |
| `/odom1` | High-frequency noisy and drifting odometry |
| `/odom` | Reference wheel odometry |
| `/kalman/odom` | Student's estimate |

The reference `/odom` is based on simulated wheel odometry, not independent Gazebo ground truth.

## Prediction model

Use the velocity commands to predict:

    x_next = x + v*cos(theta)*dt
    y_next = y + v*sin(theta)*dt
    theta_next = theta + omega*dt

Implement both the state prediction and covariance propagation using the motion Jacobian.

Use NumPy for matrix operations.

The file to modify is:

`b31ys_kalman/kalman_filter_template.py`

After editing, rebuild:

```bash
cd ~/ros2_ws
colcon build --packages-select b31ys_kalman
source install/setup.bash
```

## Run your assignment

```bash
ros2 launch b31ys_kalman kalman_execution.launch.py \
  student_name:=YourName level:=msc
```

Replace `YourName` with your own name.

## Required implementation

1. Predict `[x, y, theta]` from `/cmd_vel`.
2. Propagate the covariance using the motion Jacobian.
3. Correct the prediction using `/gps`.
4. Implement an additional correction using `/odom1`.
5. Tune the process covariance `Q`.
6. Tune the measurement covariances `R_gps` and `R_odom`.

The GPS measurement is:

    z_gps = [x_gps, y_gps]

The noisy odometry measurement is:

    z_odom = [x_odom, y_odom, theta_odom]

GPS is low-frequency and comparatively accurate. Noisy odometry is higher-frequency and contains drift.

Your filter must handle the different measurement frequencies correctly.

Apply each correction only when a new measurement arrives. Do not repeatedly use the same measurement.

Handle angular differences correctly when updating orientation.

## Submission

Submit:

1. A PDF plot comparing GPS, Kalman estimate and reference odometry. Use X versus Y or position versus time.
2. An RViz screenshot showing the estimation.
3. Your completed Extended Kalman Filter (EKF) Python code.

Do not modify the simulated sensors.

