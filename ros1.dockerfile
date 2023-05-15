ARG ROSDISTRO
FROM ros:${ROSDISTRO}-ros-base
ARG ROSDISTRO
ARG LIBFRANKA_VERSION
ARG FRANKAROS_VERSION

SHELL ["bash", "-c"]
WORKDIR /
RUN set -x && apt-get update \
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
    && dpkg -i libfranka*deb \
    && rosdep update \
    && mkdir -p franka_ws/src \
    && git clone --recursive --branch ${FRANKAROS_VERSION} https://github.com/frankaemika/franka_ros /franka_ws/src/franka_ros \
    && [ -d /franka_ws/src/franka_ros/franka_moveit_config ] && rm -rf /franka_ws/src/franka_ros/franka_moveit_config || true \
    && [ -d /franka_ws/src/franka_ros/franka_gazebo ] && rm -rf /franka_ws/src/franka_ros/franka_gazebo || true \
    && [ -d /franka_ws/src/franka_ros/franka_visualization ] && rm -rf /franka_ws/src/franka_ros/franka_visualization || true \
    && [ -d /franka_ws/src/franka_ros/franka_example_controllers ] && rm -rf /franka_ws/src/franka_ros/franka_example_controllers || true \
    && [ -f /franka_ws/src/franka_ros/franka_ros/package.xml ] &&  sed -i '/moveit/d' /franka_ws/src/franka_ros/franka_ros/package.xml || true \
    && cd /franka_ws \
    && rosdep install -r -q  --from-paths src \
                        --skip-keys libfranka --skip-keys rviz \
                        --skip-keys joint_state_publisher_gui --skip-keys franka_visualization \
                        --skip-keys franka_example_controllers  --skip-keys panda_moveit_config  \
                        --ignore-src  --rosdistro ${ROSDISTRO} -y \
    && source /opt/ros/${ROSDISTRO}/setup.bash \
    && catkin config -j $(nproc) -DCMAKE_BUILD_TYPE=Release --install --install-space /opt/ros/${ROSDISTRO}/ --extend  /opt/ros/${ROSDISTRO}/ \
    && catkin build \
    && cd / && rm -rf /franka_ws \
    && rm -rf /var/lib/apt/lists/*

RUN  echo $'\
#!/bin/bash\n\
usage() {\n\
    echo -e "\n\nusage:\n    docker run \\\n\
--network=host --privileged IMAGE \\\n\
--ros_ip=ROS_IP \\\n\
--ros_master_uri=ROS_MASTER_URI \\\n\
--robot_ip=ROBOT_IP \\\n\
[--ros_ns=NAME_SPACE]\n\
\\n\\n note: there is no space around the \"=\" signs."\n\
}\n\
main() {\n\
\n\
    if [[ $# -lt 3 ]]; then\n\
        usage\n\
        exit 1\n\
    fi\n\
    ARGUMENT_LIST=(\n\
        "robot_ip"\n\
        "ros_master_uri"\n\
        "ros_ip"\n\
        "ros_ns"\n\
    )\n\
    opts=$(\n\
        getopt \\\n\
            --longoptions "$(printf "%s:," "${ARGUMENT_LIST[@]}")" \\\n\
            --name "$(basename "$0")" \\\n\
            --options "" \\\n\
            -- "$@" || usage && exit 1\n\
    )\n\
    eval set --$opts\n\
    local ros_ns=""\n\
    while [[ $# -gt 1 ]]; do\n\
        case "$1" in\n\
        --robot_ip)\n\
            robot_ip="$2"\n\
            shift 2\n\
            ;;\n\
        --ros_master_uri)\n\
            ros_master_uri="$2"\n\
            shift 2\n\
            ;;\n\
        --ros_ip)\n\
            ros_ip="$2"\n\
            shift 2\n\
            ;;\n\
        --ros_ns)\n\
            ros_ns="$2"\n\
            shift 2\n\
            ;;\n\
        *)\n\
            usage\n\
            exit 1\n\
            ;;\n\
        esac\n\
    done\n\
    export ROS_MASTER_URI=$ros_master_uri\n\
    export ROS_IP=$ros_ip\n\
    if [[ ! -z $ros_ns ]]; then\n\
        export ROS_NAMESPACE=$ros_ns\n\
    fi\n\
    roslaunch franka_control franka_control.launch robot_ip:=$robot_ip\n\
}\n\
main $@' > /entrypoint.bash
ENTRYPOINT ["bash", "/entrypoint.bash"]
