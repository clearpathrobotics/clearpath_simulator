# clearpath_isaac

**Experimental.** NVIDIA Isaac Sim 5.1 integration for the Clearpath A300.
Drives a 4WD skid-steer A300 in Isaac Sim from a ROS 2 `cmd_vel` topic and
publishes ground-truth `odom` + `/tf`.

> Status: forward motion works end-to-end. In-place rotation currently slips
> (PhysX cylinder-on-plane contact issue — see *Known issues* below).

## What's here

```
clearpath_isaac/
├── docker/
│   ├── compose.yaml        # isaac-sim 5.1.0 + ros2:jazzy services
│   └── fastdds-udp.xml     # UDP-only DDS profile (see notes)
├── scripts/
│   ├── generate_urdf.sh    # host-side: xacro + stage meshes
│   ├── import_a300.py      # in-container: URDF -> USD
│   └── run_a300.py         # in-container: scene + cmd_vel bridge
├── urdf/
│   └── a300_standalone.urdf.xacro
└── CMakeLists.txt / package.xml
```

`urdf/a300.urdf`, `meshes/`, and `usd/` are git-ignored — regenerate them
with the steps below.

## Prerequisites

- Host with an NVIDIA GPU and recent driver
- Docker + the NVIDIA Container Toolkit (`runtime: nvidia` must work)
- ROS 2 Jazzy on the host (only used to generate the URDF and to send
  `cmd_vel` / read `odom`)
- `clearpath_common` built and sourced (for `clearpath_platform_description`)
- An X server on `DISPLAY=:1` (adjust in `compose.yaml` if different)

## One-time setup

```bash
# 1. Allow the container's X access
xhost +local:root

# 2. Set up the .docker.xauth file
touch /tmp/.docker.xauth
xauth nlist "$DISPLAY" | sed -e 's/^..../ffff/' | xauth -f /tmp/.docker.xauth nmerge -

# 3. Pre-create the cache directories so the bind mounts inherit your uid
mkdir -p ~/docker/isaac-sim/{cache/{kit,ov,pip,glcache,computecache},logs,data,documents}
```

## Generate the URDF and meshes (host)

```bash
source /opt/ros/jazzy/setup.bash
source ~/your_ws/install/setup.bash    # provides clearpath_platform_description
./scripts/generate_urdf.sh
```

This writes `urdf/a300.urdf` (with mesh paths rewritten to
`file:///workspace/meshes/...`) and copies the A300 meshes into
`meshes/clearpath_platform_description/meshes/a300/`.

## Start the containers

The compose file runs Isaac Sim as your host UID (not the built-in `isaac-sim`
user) so files written under `/workspace` come back with your ownership. It
adds `group_add: 1234` because parts of `/isaac-sim/` are mode `0750` owned by
that group.

```bash
cd docker
GID=$(id -g) docker compose up -d
```

`$UID` is exported by bash automatically; `$GID` is not, hence the prefix.

## Convert the URDF to USD (one-time)

```bash
docker exec isaac-sim /isaac-sim/python.sh /workspace/scripts/import_a300.py
```

Produces `usd/a300.usd` (~19 MB).

## Run the simulation

```bash
docker exec -d isaac-sim bash -c \
  'export DISPLAY=:1; /isaac-sim/python.sh /workspace/scripts/run_a300.py --gui > /workspace/sim.log 2>&1'
```

`--headless` is also supported. The script:

- adds a ground plane and distant light
- references in `a300.usd` and spawns it at `z=0.3`
- configures velocity drives on the four wheel joints
- binds a wheel-friction physics material
- disables the spurious sub-colliders that the URDF importer merges into
  `base_link`
- subscribes to `/cmd_vel`, publishes `/odom` and `/tf`

## Drive it

From the host (or the `ros2-jazzy` container — `docker exec -it ros2-jazzy bash`):

```bash
source /opt/ros/jazzy/setup.bash
# Important: must match the DDS profile the sim is using
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/../docker/fastdds-udp.xml"

ros2 topic pub --rate 30 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'
ros2 topic echo --once /odom
ros2 run tf2_ros tf2_echo odom base_link
```

## Why the FastDDS UDP-only profile?

`network_mode: host` shares the network namespace but **not** `/dev/shm`. The
default FastDDS shared-memory transport will silently fail to deliver messages
between host and container. The profile in `docker/fastdds-udp.xml` forces
UDPv4 only and fixes it for both sides — set
`FASTRTPS_DEFAULT_PROFILES_FILE` to it everywhere.

## Why is `rclpy` / `tf2_msgs` coming from a weird path?

Isaac Sim 5.1 ships its own ROS 2 Jazzy install bundled inside
`/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/`. The compose file points
`LD_LIBRARY_PATH` and `PYTHONPATH` there so we get a self-contained ROS 2
runtime in the container without installing anything. `tf2_ros` is not in
the bundle, so `run_a300.py` publishes `/tf` directly via `tf2_msgs/TFMessage`.

## Known issues

- **In-place rotation slips.** Forward/backward driving works (TF-validated:
  `cmd_vel.linear.x = 0.5` produces ~0.15 m/s ground speed at the current
  ~0.33 real-time factor). But `cmd_vel.angular.z` spins the wheels correctly
  while the chassis barely yaws. Root cause is PhysX cylinder-on-plane line
  contact behaving as near-frictionless under lateral scrubbing. NVIDIA's own
  wheeled robots (Carter, Jetbot) use sphere/capsule wheel colliders to avoid
  this — proper fix is to swap the wheel collision geometry in the USD or
  re-import with convex decomposition.
- **`isaacsim.ros2.bridge` extension startup logs an error.** It's benign;
  we don't use the OmniGraph-based bridge, only direct `rclpy`.
- **Real-time factor ~0.33** on a laptop RTX PRO 500 (6 GiB). Account for
  this when interpreting motion test results.
- **No suspension dynamics.** The URDF importer merges all suspension/motor
  links into `base_link` (no `<inertial>` on those links in the source xacro).
  Suspension behaviour from the real robot is not simulated.

## Tear-down

```bash
cd docker
docker compose down
```
