# clearpath_isaac container environment.
#
# Sourced from /isaac-sim/.bashrc so that an interactive shell opened with
# `docker exec ... bash` is ready to run the urdf2usd export commands and
# `. ~/runapp.sh` without any further setup.

# --- ROS 2 + the workspace (build it once, see the hint below) ---
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /isaac-sim/workspace/install/setup.bash ]; then
  source /isaac-sim/workspace/install/setup.bash
else
  echo "[clearpath_isaac] workspace not built yet. Build it once with:"
  echo "    cd /isaac-sim/workspace && colcon build --symlink-install && source install/setup.bash"
fi

# --- Isaac Sim location (used by `ros2 run urdf2usd urdf2usd_export`) ---
export ISAAC_SIM_PATH=/isaac-sim

# --- DDS so the running sim and host/other containers can see each other ---
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///isaac-sim/cyclonedds.xml

# --- Persist bash history across container restarts ---
export HISTFILE=/isaac-sim/workspace/src/clearpath_simulator/.histfile
touch "${HISTFILE}"

export PROMPT_COMMAND='history -a'
export HISTSIZE=100000
export HISTFILESIZE=200000
export HISTTIMEFORMAT="%d/%m/%y %T "
# Base image sets HISTCONTROL=ignoreboth in .bashrc; disable filtering so
# repeated commands are still recorded in history.
unset HISTCONTROL
