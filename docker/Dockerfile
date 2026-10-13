ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
# base: ROS 2 + middleware. Every node container is built on top of this.
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-demo-nodes-cpp \
      ros-${ROS_DISTRO}-xacro \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# ---------------------------------------------------------------------------
# workspace: our packages are built once here and copied to the other targets
# ---------------------------------------------------------------------------
FROM base AS workspace

COPY src /ws/src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------------
# sim: Gazebo Harmonic server and the ROS bridge
# ---------------------------------------------------------------------------
FROM base AS sim

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-ros-gz-bridge \
    && rm -rf /var/lib/apt/lists/*

COPY --from=workspace /ws/install /ws/install
