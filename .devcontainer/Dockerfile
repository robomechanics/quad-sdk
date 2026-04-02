FROM arm64v8/ros:jazzy
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        git \
        python3-colcon-common-extensions \
        python3-rosdep \
        vim \
        tmux \
        wget \
        chrony \
    && rm -rf /var/lib/apt/lists/*

RUN printf "server 192.168.8.103 iburst\nmakestep 0.1 3\nrtcsync\n" > /etc/chrony/chrony.conf

RUN rosdep init || true && rosdep update

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc

WORKDIR /root/ros2_ws

COPY ./ros_entrypoint.sh /
RUN chmod 755 /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
