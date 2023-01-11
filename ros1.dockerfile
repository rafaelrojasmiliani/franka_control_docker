ARG ROSDISTRO
FROM ros:${ROSDISTRO}-ros-base
ARG ROSDISTRO
ARG LIBFRANKA_VERSION
ARG FRANKAROS_VERSION

SHELL ["bash", "-c"]
WORKDIR /
RUN  apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
            build-essential cmake git libpoco-dev libeigen3-dev \
			python3-catkin-tools screen vim iproute2 iputils-* \
			ros-${ROSDISTRO}-joint-trajectory-controller \
    && git clone --recursive --branch ${LIBFRANKA_VERSION} https://github.com/frankaemika/libfranka /libfranka && cd /libfranka \
    && [ -f /libfranka/src/control_types.cpp ] && sed -i '1 i\#include <stdexcept>' /libfranka/src/control_types.cpp || true \
    && [ -f /libfranka/include/franka/control_tools.h ] && sed -i '1 i\#include <string>' /libfranka/include/franka/control_tools.h || true \
    && mkdir build \
    && cd build \
    && cmake  -DBUILD_TESTS=0 -DBUILD_EXAMPLES=0 -DCMAKE_BUILD_TYPE=Release .. \
    && make -j $(nproc)  \
    && cpack -G DEB \
    && dpkg -i libfranka*deb
#    && rosdep update \
#    && mkdir -p franka_ws/src \
#    && git clone --recursive --branch ${FRANKAROS_VERSION} https://github.com/frankaemika/franka_ros /franka_ws/src/franka_ros \
#    && cd /franka_ws \
#    && rosdep install -r -q  --from-paths src --skip-keys libfranka --ignore-src  --rosdistro ${ROSDISTRO} -y \
#    && source /opt/ros/${ROSDISTRO}/setup.bash \
#    && catkin config -j $(nproc) \
#    && catkin build \
#    && rm -rf /var/lib/apt/lists/*
#
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
