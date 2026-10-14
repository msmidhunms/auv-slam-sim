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
# gz: Gazebo Harmonic and the ROS bridge, used by the sim and gui targets
# ---------------------------------------------------------------------------
FROM base AS gz

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz-sim \
      ros-${ROS_DISTRO}-ros-gz-bridge \
      libgl1-mesa-dri \
      libegl1 \
    && rm -rf /var/lib/apt/lists/*

# no GPU inside docker on macOS, sonar is rendered on CPU with Mesa llvmpipe
ENV LIBGL_ALWAYS_SOFTWARE=1

# models are needed by the gz server and also by the gz GUI
ENV GZ_SIM_RESOURCE_PATH=/ws/install/auv_sim/share/auv_sim/models

# ---------------------------------------------------------------------------
# assets: vehicle mesh and textures, "MBARI Tethys LRAUV" from Gazebo Fuel
# (author Mabel Zhang, CC-BY 4.0). Version is fixed so the build is repeatable.
# ---------------------------------------------------------------------------
FROM gz AS assets

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && gz fuel download -u "https://fuel.gazebosim.org/1.0/accurrent/models/MBARI Tethys LRAUV/3" \
    && M="/root/.gz/fuel/fuel.gazebosim.org/accurrent/models/mbari tethys lrauv/3" \
    && mkdir -p /assets/auv/meshes /assets/auv/materials/textures \
    && cp "$M/meshes/tethys.dae" /assets/auv/meshes/ \
    && cp "$M"/materials/textures/*.png /assets/auv/materials/textures/

# ---------------------------------------------------------------------------
# workspace: our packages are built once here and copied to the other targets
# ---------------------------------------------------------------------------
FROM base AS workspace

COPY src /ws/src
COPY --from=assets /assets/auv /ws/src/auv_sim/models/auv
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# ---------------------------------------------------------------------------
# sim: Gazebo server and the ROS bridge
# ---------------------------------------------------------------------------
FROM gz AS sim

COPY --from=workspace /ws/install /ws/install

# ---------------------------------------------------------------------------
# nodes: our own nodes and the light weight ROS nodes (state publisher,
# robot_localization, foxglove bridge)
# ---------------------------------------------------------------------------
FROM base AS nodes

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-robot-localization \
      ros-${ROS_DISTRO}-tf2-ros \
      ros-${ROS_DISTRO}-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

COPY --from=workspace /ws/install /ws/install

# ---------------------------------------------------------------------------
# slam: RTAB-Map
# ---------------------------------------------------------------------------
FROM base AS slam

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rtabmap-slam \
    && rm -rf /var/lib/apt/lists/*

COPY --from=workspace /ws/install /ws/install

# ---------------------------------------------------------------------------
# gui: desktop in the browser (noVNC) with RViz and Gazebo GUI.
# No GPU inside docker on macOS, so rendering is on CPU with Mesa llvmpipe.
# ---------------------------------------------------------------------------
FROM sim AS gui

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-rviz2 \
      tigervnc-standalone-server \
      novnc \
      websockify \
      openbox \
      supervisor \
      libgl1-mesa-dri \
      mesa-utils \
      fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*

# RViz on the left half and Gazebo on the right half of the desktop (for 1600x900).
# Gazebo GUI is taking its window size from its own config, so default config is
# copied and the window part alone is changed.
RUN sed -i 's|</applications>|\
  <application title="*RViz*"><decor>no</decor><position force="yes"><x>0</x><y>0</y></position><size><width>50%</width><height>100%</height></size></application>\
  <application title="Gazebo*"><decor>no</decor><position force="yes"><x>50%</x><y>0</y></position></application>\
</applications>|' /etc/xdg/openbox/rc.xml \
    && mkdir -p /root/.gz/sim/8 \
    && sed -e 's|<width>1000</width>|<width>800</width>|' \
           -e 's|<height>845</height>|<height>900</height>|' \
           /opt/ros/${ROS_DISTRO}/opt/gz_sim_vendor/share/gz/gz-sim8/gui/gui.config \
           > /root/.gz/sim/8/gui.config

COPY docker/gui/supervisord.conf /etc/supervisor/conf.d/gui.conf
COPY docker/gui/index.html /usr/share/novnc/index.html

ENV DISPLAY=:1 \
    VNC_RESOLUTION=1600x900 \
    START_RVIZ=true \
    START_GZ_GUI=true \
    RVIZ_CONFIG=/ws/install/auv_description/share/auv_description/rviz/auv.rviz \
    QT_X11_NO_MITSHM=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root
RUN mkdir -m 700 -p /tmp/runtime-root

EXPOSE 6080
CMD ["supervisord", "-c", "/etc/supervisor/supervisord.conf"]
