# clearpath_isaac

**Experimental.** NVIDIA Isaac Sim 5.1 + ROS 2 Jazzy integration for Clearpath
robots. Converts a Clearpath xacro into a physics-ready Isaac Sim USD (via
`urdf2usd`), spawns it, and drives it from `/cmd_vel` while publishing `/odom`,
`/tf`, and `/joint_states`.

Everything runs inside one Docker container built from the official
`isaac-sim:5.1.0` image. Supported robots: `a300`, `do100`, `j100`.

## Prerequisites

- Host with an NVIDIA GPU + recent driver
- Docker + the NVIDIA Container Toolkit (`runtime: nvidia` must work)
- A running X server (for the Isaac Sim GUI)

## 1. One-time host setup

```bash
# Allow the container to reach your X server
xhost +local:root

# Pre-create the Isaac Sim cache dirs so the bind mounts inherit your uid
mkdir -p /var/tmp/isaac-sim/{cache/{main,computecache},logs,config,data,pkg}
```

## 2. Build and start the container

`HOST_UID` / `HOST_GID` are **required** — they map the in-container user to
you so generated USDs come back with your ownership.

```bash
cd docker
export HOST_UID=$(id -u) HOST_GID=$(id -g)
docker compose build
docker compose up -d
docker compose exec clearpath-isaac bash
```

## 3. Create workspace, clone repos, and build (once, inside the container)

```bash
mkdir -p ~/workspace/src
cd ~/workspace/src

# Clone this repo and clearpath_common on the branch you want to use
git clone --branch feature/isaac-sim-support https://github.com/clearpathrobotics/clearpath_simulator.git
git clone --branch feature/isaac-sim-support https://github.com/clearpathrobotics/clearpath_common.git

cd ~/workspace
colcon build --symlink-install --packages-up-to urdf2usd clearpath_isaac
source install/setup.bash
```

## 4. Export a robot USD (inside the container)

```bash
URDF_DIR=$(ros2 pkg prefix clearpath_isaac)/share/clearpath_isaac/urdf
OUT=/isaac-sim/workspace/src/clearpath_simulator/clearpath_isaac/generated_assets

ros2 run urdf2usd urdf2usd_export \
  --xacro  $URDF_DIR/a300.urdf.xacro \
  --output $OUT/a300/a300.usd
```

Swap `a300` for `do100` or `j100` as needed.

## 5. Run the simulation + teleop (inside the container)

```bash
ros2 launch clearpath_isaac robot_teleop.launch.xml \
  robot_path:=/isaac-sim/workspace/src/clearpath_simulator/clearpath_isaac/generated_assets/a300/a300.usd
```

Launch args:

| arg          | default        | description                          |
| ------------ | -------------- | ------------------------------------ |
| `robot_path` | *(required)*   | Path to the exported robot USD       |
| `robot_prim` | `/World/robot` | Prim path to reference the USD under |
| `namespace`  | `a300`         | ROS 2 namespace for teleop and `cmd_vel` |
| `spawn_z`    | `0.15`         | Initial spawn height (m)             |
| `headless`   | `false`        | Run Isaac Sim without a GUI          |
| `teleop`     | `true`         | Launch `teleop_twist_keyboard`       |

The teleop window (an `xterm`) captures the keyboard and publishes
`<namespace>/cmd_vel`.

## Customizing a robot

Each xacro carries an `<isaac_inputs>` block that tells `urdf2usd` how to build
the articulation, drives, physics materials, sensors, and ROS 2 graphs. See
`urdf/example.urdf.xacro` for the full option set.

## Tear-down

```bash
cd docker
docker compose down
```
