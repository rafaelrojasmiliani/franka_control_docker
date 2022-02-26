FROM moveit/moveit:melodic-release

SHELL ["bash", "-c"]
WORKDIR /
RUN  apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
            build-essential cmake git libpoco-dev libeigen3-dev python3-catkin-tools \
    && git clone --recursive --branch 0.7.1 https://github.com/frankaemika/libfranka /libfranka && cd /libfranka \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && make -j 16  \
    && cpack -G DEB \
    && dpkg -i libfranka*deb \
    && rosdep update \
    && mkdir -p franka_ws/src \
    && git clone --recursive --branch 0.7.0 https://github.com/frankaemika/franka_ros /franka_ws/src/franka_ros \
    && cd /franka_ws && rosdep install -r -q  --from-paths src --ignore-src --rosdistro melodic -y \
    && source /opt/ros/melodic/setup.bash \
    && catkin config -j 8 \
    && catkin build \
    && rm -rf /var/lib/apt/lists/*

RUN echo $'\
#!/bin/bash\n\
main(){\n\
source /franka_ws/devel/setup.bash \n\
export ROS_MASTER_URI=$1\n\
export ROS_IP=$2\n\
export ROS_NAMESPACE=$3 \n\
roslaunch franka_control franka_control.launch robot_ip:=$4\n\
bash \n\
}\n\
main $@' > /entrypoint.bash
ENTRYPOINT ["bash", "/entrypoint.bash"]
