# B31YS CW2 (Kalman filter) - ROS 2 Humble + Gazebo + ROSbot simulation
# Build context: the CW2 folder (see compose.yaml)

FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]
# HUSARION_ROS_BUILD_TYPE=simulation makes rosdep install the Gazebo dependencies
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    HUSARION_ROS_BUILD_TYPE=simulation

ARG WS_SRC=b31ys-kalman-filter--2026-11-07--ccca035c1bcf/ros2_ws/src

# Extra tools: SLAM, map server (for the PRM map), teleop, plotting
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-dev-tools \
        python3-pip \
        python3-numpy \
        python3-matplotlib \
        ros-humble-slam-toolbox \
        ros-humble-nav2-map-server \
        ros-humble-nav2-lifecycle-manager \
        ros-humble-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ${WS_SRC} src

# Packages built on The Construct, plus mecanum_drive_controller (needed by rosbot_controller)
ARG PKGS="b31ys_kalman husarion_components_description husarion_gz_worlds \
mecanum_drive_controller rosbot_bringup rosbot_controller rosbot_description rosbot_gazebo \
rosbot_joy rosbot_localization rosbot_utils"

# Dependencies of those packages only (skips hardware/manipulator extras)
RUN apt-get update \
    && rosdep update --rosdistro $ROS_DISTRO \
    && paths=$(colcon list --paths-only --packages-select $PKGS) \
    && rosdep install -i -y -r --rosdistro $ROS_DISTRO --from-paths $paths \
       --skip-keys "micro_ros_agent stm32flash python3-pyftdi-pip python3-libgpiod \
                    open_manipulator_x_description open_manipulator_x_moveit \
                    kortex_description robotiq_description ur_description \
                    depthai_descriptions realsense2_description zed_msgs" \
    && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --packages-select $PKGS --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc \
    && echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Plots / CSVs written here end up in CW2/docker/output on the host
WORKDIR /output
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
