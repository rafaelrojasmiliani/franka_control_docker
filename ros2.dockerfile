ARG ROSDISTRO
FROM ros:${ROSDISTRO}-ros-base
ARG ROSDISTRO
ARG LIBFRANKA_VERSION
ARG FRANKAROS_REPO
ARG FRANKAROS_BRANCH

SHELL ["bash", "-c"]
WORKDIR /
RUN set -x && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
            build-essential cmake git libpoco-dev libeigen3-dev \
			screen vim iproute2 iputils-* \
			ros-${ROSDISTRO}-joint-trajectory-controller ros-${ROSDISTRO}-control-toolbox \
            ros-${ROSDISTRO}-xacro ros-${ROSDISTRO}-controller-manager ros-${ROSDISTRO}-joint-state-broadcaster \
            ros-${ROSDISTRO}-joint-state-publisher \
            ros-${ROSDISTRO}-ros2-control-test-assets ros-${ROSDISTRO}-xacro ros-${ROSDISTRO}-ament-clang-format \
    && git clone --recursive --branch ${LIBFRANKA_VERSION} https://github.com/frankaemika/libfranka /libfranka && cd /libfranka \
    && [ -f /libfranka/src/control_types.cpp ] && sed -i '1 i\#include <stdexcept>' /libfranka/src/control_types.cpp || true \
    && [ -f /libfranka/include/franka/control_tools.h ] && sed -i '1 i\#include <string>' /libfranka/include/franka/control_tools.h || true \
    && mkdir build \
    && cd build \
    && cmake  -DBUILD_TESTS=0 -DBUILD_EXAMPLES=0 -DCMAKE_BUILD_TYPE=Release .. \
    && make -j $(nproc)  \
    && cpack -G DEB \
    && dpkg -i libfranka*deb \
    && rosdep update \
    && mkdir -p franka_ws/src \
    && git clone --recursive --branch ${FRANKAROS_BRANCH} ${FRANKAROS_REPO} /franka_ws/src/franka_ros \
    && [ -d /franka_ws/src/franka_ros/franka_moveit_config ] && rm -rf /franka_ws/src/franka_ros/franka_moveit_config || true \
    && cd /franka_ws \
    && rosdep install -r -q  --from-paths src --skip-keys libfranka --skip-keys rviz2 --skip-keys joint_state_publisher_gui --skip-keys ament_cmake_clang_format --ignore-src  --rosdistro ${ROSDISTRO} -y \
    && source /opt/ros/${ROSDISTRO}/setup.bash \
    && colcon build \
    && rm -rf /var/lib/apt/lists/*
# ros-galactic-ros2-control-test-assets (galactic or foxy) ros-galactic-xacro ros-foxy-ament-clang-format
#RUN echo $'\
#termcapinfo xterm* ti@:te@ \n\
#hardstatus alwayslastline \n\
#hardstatus string \'%{= kG}[%{G}%H%? %1`%?%{g}][%= %{= kw}%-w%{+b yk} %n*%t%?(%u)%? %{-}%+w %=%{g}][%{B}%m/%d %{W}%C%A%{g}]\' \n\
#attrcolor b ".I" \n\
#defbce on \n\
#startup_message off \n\
#mousetrack on \n\
#screen -t franka_control 1 bash \n\
#screen -t control_manager_and_bash  2 bash \n\
#select 1 \n\
#stuff "sleep 2; roslaunch franka_control franka_control.launch robot_ip:=ROBOT_IP" \n\
#split -v \n\
#focus next \n\
#select 2 \n\
#stuff "sleep 3; rosrun controller_manager spawner position_joint_trajectory_controller" \n\
#layout save default' > /screen_conf_base \
#&& echo $'\
##!/bin/bash\n\
#main(){\n\
#source /franka_ws/devel/setup.bash \n\
#export ROS_MASTER_URI=$1\n\
#export ROS_IP=$2\n\
#export ROS_NAMESPACE=$3 \n\
#screen -S roscore -d -m roscore \n\
#cat screen_conf_base | sed "s/ROBOT_IP/$4/" > /screen_conf \n\
#screen -c /screen_conf \n\
#}\n\
#main $@' > /entrypoint.bash
#ENTRYPOINT ["bash", "/entrypoint.bash"]
