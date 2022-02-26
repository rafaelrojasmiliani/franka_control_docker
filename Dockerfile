FROM moveit/moveit:melodic-release

SHELL ["bash", "-c"]
WORKDIR /
RUN  apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get -y install --no-install-recommends \
      build-essential cmake git libpoco-dev libeigen3-dev \
  && rm -rf /var/lib/apt/lists/*

RUN git clone --recursive --branch 0.7.1 https://github.com/frankaemika/libfranka /libfranka && cd /libfranka \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release .. \
    && cmake --build . \
    && cpack -G DEB \
    && dpkg -i libfranka*deb

RUN rosdep init \
    && rosdep update \
    && mkdir -p franka_ws/src \
    && git clone --recursive --branch 0.7.0 https://github.com/frankaemika/franka_ros /franka_ws/src/franka_ros \
    && cd /franka_ws && rosdep install -r -q  --from-paths src --ignore-src --rosdistro melodic -y \
    && catkin build
