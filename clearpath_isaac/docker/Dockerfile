# Stripped-down Isaac Sim + ROS 2 image for the Clearpath URDF -> USD workflow.
#
# All clearpath_* dependencies are installed from apt during the image build
# (via rosdep). The workspace itself (clearpath_isaac + urdf2usd) is staged in
# the image but NOT built; build + source it once after exec:
#
#   cd /isaac-sim/workspace && colcon build --symlink-install && source install/setup.bash
#
#   ros2 run urdf2usd urdf2usd_export --xacro $(ros2 pkg prefix clearpath_isaac)/share/clearpath_isaac/urdf/dd100.urdf.xacro --output /isaac-sim/workspace/src/generated_assets/dd100/dd100.usd
#   ros2 run urdf2usd urdf2usd_export --xacro $(ros2 pkg prefix clearpath_isaac)/share/clearpath_isaac/urdf/do100.urdf.xacro --output /isaac-sim/workspace/src/generated_assets/do100/do100.usd
#   ros2 run urdf2usd urdf2usd_export --xacro $(ros2 pkg prefix clearpath_isaac)/share/clearpath_isaac/urdf/j100.urdf.xacro  --output /isaac-sim/workspace/src/generated_assets/j100/j100.usd
#   ros2 run urdf2usd urdf2usd_export --xacro $(ros2 pkg prefix clearpath_isaac)/share/clearpath_isaac/urdf/a300.urdf.xacro  --output /isaac-sim/workspace/src/generated_assets/a300/a300.usd
#   . ~/runapp.sh


ARG ISAAC_SIM_VERSION=5.1.0
FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_VERSION}

ARG ROS_VERSION=jazzy
ENV ROS_DISTRO=${ROS_VERSION}

# Match the host user so files written to bind mounts (the generated USDs)
# come back with the right ownership.
ARG HOST_UID=1000
ARG HOST_GID=1000

USER root
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# --- Minimal system tooling + the ROS 2 and Clearpath apt repositories ---
RUN apt-get update && apt-get install -y \
    sudo curl gnupg2 lsb-release git wget ca-certificates xterm && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/ros2-latest.list && \
    curl -fsSL https://packages.clearpathrobotics.com/public.key \
        | gpg --dearmor -o /usr/share/keyrings/clearpath-archive-keyring.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/clearpath-archive-keyring.gpg] https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/clearpath-latest.list && \
    apt-get update && apt-get install -y \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-xacro \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-clearpath-common \
        ros-${ROS_DISTRO}-teleop-twist-keyboard \
        ros-dev-tools && \
    rosdep init || true && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# xacro is the only extra the Isaac-side Python launcher might import; the
# ROS-side entry point uses the system `xacro` CLI installed above.
RUN /isaac-sim/python.sh -m pip install --no-cache-dir xacro

# --- Stage only the packages that live in this repo into the workspace.
#     All clearpath_* dependencies are pulled from apt by rosdep below.
#     The colcon build + source are deliberately left to the user (run once
#     after `docker compose exec`); see clearpath_isaac_env.sh for the hint. ---
ENV CLEARPATH_WS=/isaac-sim/workspace
RUN mkdir -p ${CLEARPATH_WS}/src
COPY src/clearpath_simulator/clearpath_isaac ${CLEARPATH_WS}/src/clearpath_isaac
COPY src/urdf2usd                            ${CLEARPATH_WS}/src/urdf2usd

# Overlay the in-repo clearpath_platform_description so the do100 omni mecanum
# wheel (individual passive rollers + Roller.STL) is used instead of the apt
# package's simplified cylinder wheel. Built by the user's colcon invocation.
COPY src/clearpath_common/clearpath_platform_description ${CLEARPATH_WS}/src/clearpath_platform_description

# Resolve dependencies from apt (clearpath_* descriptions, xacro, rclpy, …).
RUN apt-get update && \
    rosdep update --rosdistro=${ROS_DISTRO} && \
    rosdep install --from-paths ${CLEARPATH_WS}/src --ignore-src -r -y \
        --rosdistro=${ROS_DISTRO} \
        --skip-keys "isaacsim" && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# --- Container runtime configuration (DDS + the auto-sourced env) ---
COPY src/clearpath_simulator/clearpath_isaac/docker/cyclonedds.xml /isaac-sim/cyclonedds.xml
COPY src/clearpath_simulator/clearpath_isaac/docker/clearpath_isaac_env.sh /isaac-sim/clearpath_isaac_env.sh
RUN echo "source /isaac-sim/clearpath_isaac_env.sh" >> /isaac-sim/.bashrc

# --- Re-map the built-in isaac-sim user to the host uid/gid ---
RUN if id ubuntu >/dev/null 2>&1; then deluser ubuntu && rm -rf /home/ubuntu; fi && \
    usermod -u ${HOST_UID} -a -G sudo isaac-sim && \
    groupmod -g ${HOST_GID} isaac-sim && \
    echo "isaac-sim ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/isaac-sim && \
    chmod 0440 /etc/sudoers.d/isaac-sim && \
    chown -R ${HOST_UID}:${HOST_GID} /isaac-sim

USER isaac-sim
WORKDIR /isaac-sim
ENTRYPOINT ["/bin/bash"]
