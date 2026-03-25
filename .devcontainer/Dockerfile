FROM osrf/ros:jazzy-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        vim \
        tmux \
        wget \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

WORKDIR /root/ros2_ws

COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
