# Standalone ROS 2 Jazzy image for recording the Go2's native (stock-controller)
# DDS topics. Deliberately separate from the Quad-SDK dev image so recording
# never depends on, or disturbs, the driver/planner/MPC container.
#
#   docker build -t quad-sdk/go2-bag-recorder:jazzy quad_utils/scripts/go2_bag_recorder
#
# The Unitree message packages are required: rosbag2 can only record a topic
# whose type support it can load, and silently skips the rest. Without
# unitree_go / unitree_api, /lowstate, /sportmodestate, /wirelesscontroller,
# /api/* ... would all be dropped from the bag.
FROM ros:jazzy-ros-base

ARG UNITREE_ROS2_COMMIT=668d1ec5a05d1c38d3306bdca7d59f2ba3581a88

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-rosidl-generator-dds-idl \
        ros-jazzy-sensor-msgs \
        ros-jazzy-nav-msgs \
        ros-jazzy-geometry-msgs \
        ros-jazzy-tf2-msgs \
        ros-jazzy-visualization-msgs \
        git \
    && rm -rf /var/lib/apt/lists/*

# Only the message packages; the unitree_ros2 examples are not needed. Two
# firmware types missing from unitree_ros2 are added from extra_msgs/ (see the
# .msg headers). Remaining unknown types -- unitree_arm/*, unitree_interfaces/*
# -- have no public definition and cannot be recorded.
COPY extra_msgs /tmp/extra_msgs
RUN mkdir -p /opt/unitree_ws/src \
    && git clone https://github.com/unitreerobotics/unitree_ros2.git /tmp/unitree_ros2 \
    && git -C /tmp/unitree_ros2 checkout ${UNITREE_ROS2_COMMIT} \
    && cp -r /tmp/unitree_ros2/cyclonedds_ws/src/unitree/unitree_go \
             /tmp/unitree_ros2/cyclonedds_ws/src/unitree/unitree_api \
             /tmp/unitree_ros2/cyclonedds_ws/src/unitree/unitree_hg \
             /opt/unitree_ws/src/ \
    && cp /tmp/extra_msgs/unitree_go/*.msg /opt/unitree_ws/src/unitree_go/msg/ \
    && sed -i 's|"msg/WirelessController.msg"|"msg/WirelessController.msg" "msg/VoxelMapCompressed.msg" "msg/ConfigChangeStatus.msg"|' \
           /opt/unitree_ws/src/unitree_go/CMakeLists.txt \
    && rm -rf /tmp/unitree_ros2 \
    && cd /opt/unitree_ws \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# The recorder runs as the host user (so bags are owned by `unitree`), which
# has no home directory inside the image.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    HOME=/tmp \
    ROS_LOG_DIR=/tmp/ros_log

RUN printf '%s\n' '#!/bin/bash' \
        'source /opt/ros/jazzy/setup.bash' \
        'source /opt/unitree_ws/install/setup.bash' \
        'exec "$@"' > /usr/local/bin/go2_env && chmod +x /usr/local/bin/go2_env
ENTRYPOINT ["/usr/local/bin/go2_env"]
CMD ["bash"]
