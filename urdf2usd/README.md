# urdf2usd

Convert a URDF/xacro robot description into an Isaac Sim USD, driven by a single
`<isaac_inputs>` block embedded in the URDF (or supplied as a standalone XML
file). Everything **outside** `<isaac_inputs>` stays standards-compliant, so the
same description still works with `robot_state_publisher`, `ros2_control`, and
Gazebo unchanged.

The `<isaac_inputs>` block is only emitted when xacro is invoked with
`use_isaac:=true` (the default in the Clearpath `*_isaac.urdf.xacro` files).
Setting `use_isaac:=false` produces a plain URDF with no Isaac-specific tags.

## How it works

The exporter runs in two interpreters because `xacro` / `rclpy` and `isaacsim`
are C-extensions built for different Python versions:

1. **ROS side** (`scripts/urdf2usd_export`, system Python): expands the xacro,
   rewrites `package://` URIs to absolute paths, parses `<isaac_inputs>` into a
   flat JSON config, then hands off to the Isaac worker.
2. **Isaac side** (`urdf2usd/_isaac_worker.py`, Isaac Sim's bundled Python):
   imports the URDF, then applies the config via `post_process.apply_isaac_inputs`
   (physics scene → link materials → joint properties → sensors → articulation
   graphs) and saves the USD.

## Isaac Sim version compatibility

The package supports **both Isaac Sim 5.x and 6.x** from one codebase. The
running version is never parsed from a string; instead each API that diverged
between releases is chosen by *capability detection* (`urdf2usd/_compat.py`), so
any 5.x or 6.x point release works. The backend actually selected is printed at
import time (`[urdf2usd] URDF import backend: ...`).

Where the two releases differ:

| Concern | Isaac Sim 5.x | Isaac Sim 6.x |
| --- | --- | --- |
| URDF importer | command API (`URDFCreateImportConfig` / `URDFParseFile` / `URDFImportRobot`) | class API (`URDFImporter` / `URDFImporterConfig`) |
| GLB/glTF visual meshes | loaded natively (assimp) | **converted to OBJ first** (the 6.x `urdf-usd-converter` loads only OBJ/DAE/STL) |
| IMU authoring | `isaacsim.sensors.physics.IMUSensor` | `isaacsim.sensors.experimental.physics.IMU` |
| Materials / articulation / camera | `isaacsim.core.experimental.*` (Isaac Sim ≥ 5.0) | same |

**GLB conversion (6.x only).** Because the 6.x importer cannot read GLB/glTF,
those visual meshes are transparently converted to OBJ before import (5.x keeps
native GLB for best fidelity). This needs [`trimesh`](https://trimesh.org) in
Isaac Sim's Python:

```bash
${ISAAC_SIM_PATH}/python.sh -m pip install 'trimesh[easy]'
```

If a robot has no GLB/glTF visuals, `trimesh` is never imported and the
dependency is unnecessary.


## Running the exporter

Inside the project's Isaac Sim container:

```bash
ros2 run urdf2usd urdf2usd_export \
    --xacro $(ros2 pkg prefix clearpath_isaac)/share/clearpath_isaac/urdf/example.urdf.xacro \
    --output /tmp/example.usd
```

Useful flags (see `--help` for the full list):

| Flag | Purpose |
| --- | --- |
| `--urdf PATH` | Use a pre-expanded URDF instead of a xacro. |
| `--xacro PATH` | Expand a xacro via the `xacro` CLI (mutually exclusive with `--urdf`). |
| `--from-rsp` | Fetch `/robot_description` from a running `robot_state_publisher`. |
| `--external-inputs PATH` | Standalone `<isaac_inputs>` XML that **overrides** any embedded block. |
| `--output, -o PATH` | Destination USD path (required). |
| `--no-headless` | Open the Isaac Sim window to watch the conversion. |
| `--keep-open` | Keep simulating after saving (until Ctrl-C). |
| `--keep-intermediates` | Keep `<stem>.expanded.urdf` and `<stem>.isaac_inputs.json`. |
| `--isaac-sim-path PATH` | Isaac Sim install dir (default `$ISAAC_SIM_PATH` or `/isaac-sim`). |

## Testing

The parser and URDF helpers have **no** Isaac Sim / ROS dependency, so their
unit tests run in a plain Python environment:

```bash
# from src/urdf2usd
pytest test/test_parser.py
# or, once built with colcon:
colcon test --packages-select urdf2usd
```

To exercise the **full** pipeline (import + all config sections + every sensor
type), export the reference robot inside the Isaac container:

```bash
ros2 run urdf2usd urdf2usd_export \
    --xacro $(ros2 pkg prefix clearpath_isaac)/share/clearpath_isaac/urdf/example.urdf.xacro \
    --output /tmp/example.usd --keep-intermediates
```

[`example.urdf.xacro`](../clearpath_simulator/clearpath_isaac/urdf/example.urdf.xacro)
is a deliberately maximal testbed: it activates every `<isaac_inputs>` section
and mounts one of every supported sensor type. The lidar and depth camera use
off-the-shelf vendor USD assets, so that export needs a reachable Nucleus server
(the `{nucleus}` placeholder). The per-platform files (`a300.urdf.xacro`,
`j100.urdf.xacro`, `do100.urdf.xacro`, `do100_sensors.urdf.xacro`) show the
minimal real-world subset.

---

# `<isaac_inputs>` schema reference

Every attribute below is **optional** unless marked *(required)*. Anything
omitted keeps the documented default (or, for `import_config`, the Isaac Sim
`URDFImporterConfig` default — only attributes that are present are
forwarded). All values are strings in XML; the consumers coerce them.

Multiple embedded `<isaac_inputs>` blocks are merged in document order. A
standalone file passed via `--external-inputs` fully **replaces** any embedded
block.

## `<import_config>` — URDF import settings (singleton)

Import settings are backend-agnostic: on Isaac Sim 6.x they populate
`URDFImporterConfig`; on 5.x they are routed onto the equivalent
`URDFCreateImportConfig` attributes. Only present attributes are applied. Both
the new (6.x) and legacy (5.x) attribute spellings are accepted regardless of
the running version: `self_collision`, `density`, `default_drive_strength`,
`default_position_drive_damping`, `default_drive_type`, and `convex_decomp` map
to/from `allow_self_collision`, `link_density`, `override_joint_stiffness`,
`override_joint_damping`, `joint_target_type`, and `collision_type`
respectively. On 6.x, `import_inertia_tensor`, `override_joint_dynamics`,
`parse_mimic`, `replace_cylinders_with_capsules`, `distance_scale`, and
`make_default_prim` are ignored with a warning (they still apply on 5.x);
conversely `run_asset_transformer` / `run_multi_physics_conversion` are 6.x-only
and ignored on 5.x.

| Attribute | Meaning |
| --- | --- |
| `merge_fixed_joints` | Consolidate links connected by fixed joints. |
| `fix_base` | `true` fixes the base to the world; `false` makes it floating. |
| `merge_mesh` | Merge meshes where possible to optimize the model. |
| `collision_from_visuals` | Generate collision from visual geometry. |
| `collision_type` | `Convex Hull` \| `Convex Decomposition` \| `Bounding Sphere` \| `Bounding Cube`. |
| `allow_self_collision` | Enable self-collision between links. |
| `link_density` | Default density (kg/m³) for links with no explicit mass. |
| `joint_drive_type` | `force` \| `acceleration`. |
| `joint_target_type` | `none` \| `position` \| `velocity`. |
| `override_joint_stiffness` | Joint drive stiffness (Nm/rad or N/m). |
| `override_joint_damping` | Joint drive damping (Nm·s/rad or N·s/m). |
| `robot_type` | Isaac robot schema category, e.g. `Default`, `Wheeled`. |
| `run_asset_transformer` | Restructure the USD and **collect mesh dependencies** via the asset-transformer profile (pinned `true` — required, otherwise visual meshes are lost). |
| `run_multi_physics_conversion` | Run URDF→PhysX joint conversion (pinned `false`; `post_process` authors joint attributes itself). |

## `<articulation>` — articulation root + articulation-level ROS 2 bridges (singleton)

Wires the graphs that live at the articulation level (clock, joint_states,
joint_commands, odom, tf) and, optionally, an off-the-shelf `cmd_vel` drive
controller.

| Attribute | Default | Meaning |
| --- | --- | --- |
| `chassis_link` | *(required for odom/tf/joint graphs)* | Articulation root link. |
| `namespace` | `""` | ROS 2 node namespace. |
| `clock` | `false` | Publish sim time (topic fixed at `/clock`). |
| `joint_states` | `false` | Publish `sensor_msgs/JointState`. |
| `joint_states_topic` | `/joint_states` | |
| `joint_commands` | `false` | Subscribe to joint commands. |
| `joint_commands_topic` | `/joint_command` | |
| `odom` | `false` | Publish `nav_msgs/Odometry`. |
| `odom_topic` | `/odom` | |
| `tf` | `false` | Publish the TF tree. |
| `tf_topic` | `/tf` | |

### `cmd_vel` drive controller (optional, off-the-shelf)

When `cmd_vel="true"`, an Isaac `wheeled_robots` controller is baked into the
graph and subscribed to a `geometry_msgs/Twist`.

| Attribute | Default | Meaning |
| --- | --- | --- |
| `cmd_vel` | `false` | Enable the drive controller. |
| `drive_controller` | `diff` | `diff` \| `holonomic` \| `ackermann`. |
| `cmd_vel_topic` | `cmd_vel` | Twist topic to subscribe to. |
| `cmd_vel_use_namespace` | `true` | Prefix `cmd_vel_topic` with `namespace`. |
| `wheel_radius` | `0.1` | Wheel radius (m). |
| `wheel_distance` | `0.4` | Track width (m). |
| `wheel_base` | `0.4` | Front-to-rear distance (ackermann). |
| `front_wheel_joints` | | Space-separated joint names. |
| `rear_wheel_joints` | | Space-separated joint names. |
| `wheel_joints` | | All wheel joints (holonomic). |
| `steer_wheel_joints` | | Steering joints (ackermann). |
| `front_wheel_radius` / `rear_wheel_radius` | | Per-axle radius override. |
| `mecanum_angles` | | One roller angle per `wheel_joints` entry (holonomic). |
| `com_link` | `chassis_link` | Center-of-mass link. |

## `<physics_scene>` — PhysX scene settings (singleton)

Authors a self-contained `PhysicsScene` so the robot runs at the right rate
straight from the USD.

| Attribute | Default | Meaning |
| --- | --- | --- |
| `time_steps_per_second` | Isaac default | Physics sub-stepping rate (Hz). |
| `gravity` | `9.81` | Gravity magnitude (m/s², pulls −Z). |
| `friction_type` | Isaac default | `patch` \| `oneDirectional` \| `twoDirectional`. |
| `friction_offset_threshold` | Isaac default | Patch-merging offset threshold. |
| `friction_correlation_distance` | Isaac default | Patch-merging correlation distance. |

## `<physics_material name="...">` — reusable rigid-body material preset

| Attribute | Default | Meaning |
| --- | --- | --- |
| `name` | *(required)* | Preset id, referenced by `<link physics_material=...>`. |
| `static_friction` | `0.5` | |
| `dynamic_friction` | `0.5` | |
| `restitution` | `0.0` | |
| `friction_combine_mode` | PhysX default | `average` \| `min` \| `multiply` \| `max`. |
| `restitution_combine_mode` | PhysX default | `average` \| `min` \| `multiply` \| `max`. |

PhysX uses the higher-priority combine mode of the two contacting materials
(`max` > `multiply` > `min` > `average`), so `max` makes this material's
friction dominate the contact.

## `<visual_material name="...">` — reusable visual material preset

Creates a `UsdPreviewSurface`.

| Attribute | Default | Meaning |
| --- | --- | --- |
| `name` | *(required)* | Preset id, referenced by `<link visual_material=...>`. |
| `diffuse` | `0.5 0.5 0.5` | RGB diffuse color. |
| `roughness` | `0.5` | |
| `metallic` | `0.0` | |

## `<link name="...">` — per-link material binding

Binds a physics and/or visual preset onto a link. At least one of the two
attributes is required. Works even for links collapsed by `merge_fixed_joints`
(the geometry is relocated and re-bound).

| Attribute | Meaning |
| --- | --- |
| `name` | *(required)* URDF link name. |
| `physics_material` | Name of a `<physics_material>` preset. |
| `visual_material` | Name of a `<visual_material>` preset. |

## `<joint_properties name="...">` — reusable per-joint preset

Applies to **any** joint (not just driven ones). Forwarded to Isaac's
`Articulation` wrapper. Omitting an attribute leaves whatever the URDF / Isaac
Sim already set.

| Attribute | Default | Meaning |
| --- | --- | --- |
| `name` | *(required)* | Preset id, referenced by `<joint properties=...>`. |
| `armature` | `0.0` | `set_dof_armatures`. |
| `static_friction` | `0.0` | `set_dof_friction_properties`. |
| `dynamic_friction` | `0.0` | `set_dof_friction_properties`. |
| `viscous_friction` | `0.0` | `set_dof_friction_properties`. |
| `lower_limit` / `upper_limit` | from URDF | `set_dof_limits`. |
| `max_velocity` | from URDF | `set_dof_max_velocities`. |
| `max_effort` | from URDF | `set_dof_max_efforts`. |
| `drive_type` | `force` | `force` \| `acceleration`. |
| `control_mode` | *(unset)* | `position` \| `velocity` \| `effort`. |
| `stiffness` | `0.0` | Drive kP (`set_dof_gains`). |
| `damping` | `0.0` | Drive kD (`set_dof_gains`). |

## `<joint name="..." properties="...">` — per-joint preset binding

| Attribute | Meaning |
| --- | --- |
| `name` | *(required)* URDF joint name (must match a DOF). |
| `properties` | *(required)* Name of a `<joint_properties>` preset. |

## `<sensor name="..." type="...">` — sensors

Each sensor's `type` picks the sensor kind; `ros2` toggles ROS 2 publishing per
sensor. Pick a sensor with a **config name**:

- **Basic sensor:** omit `config` for the type's default, or name a built-in
  config. No USD asset needed.
- **Real device:** name its config (`OS1`, `HESAI_XT32_SD10`, `SICK_microScan3`,
  ...) to get the vendor's scan pattern.
- **Tweak it:** set numeric parameters (e.g. `max_range`) alongside the config
  to override just those values.

Common attributes:

| Attribute | Default | Meaning |
| --- | --- | --- |
| `name` | *(required)* | Unique sensor name. |
| `type` | *(required)* | `imu` \| `rtx_lidar` \| `rgb_camera` \| `depth_camera`. |
| `link` | *(required)* | URDF link the sensor mounts to. |
| `ros2` | `true` | Create a ROS 2 OmniGraph for this sensor. |
| `frame_id` | `<link>` | ROS frame_id in published messages. |
| `frequency` | `60.0` | Update rate (Hz), where applicable. |
| `translation` | `0 0 0` | XYZ offset relative to `link`. |
| `orientation` | `1 0 0 0` | WXYZ quaternion relative to `link`. |
| `config` | *(type default)* | Built-in or vendor config name (see each type below). |
| `variant` | *(optional)* | Config variant, e.g. Ouster `OS1_REV7_128ch10hz1024res`. |
| `asset` | *(optional)* | Vendor USD path instead of a config; supports `{nucleus}`, `package://`, or absolute. |

### `type="imu"`

Authors an Isaac `IsaacImuSensor` prim (via `IMU.create`) and, when
`ros2="true"`, a graph publishing `sensor_msgs/Imu`. Note: `frequency` is
ignored for IMUs in Isaac Sim 6.x.

| Attribute | Default |
| --- | --- |
| `linear_acceleration_filter_size` | `10` |
| `angular_velocity_filter_size` | `10` |
| `orientation_filter_size` | `10` |
| `topic` | `imu` |
| `publish_orientation` | `true` |
| `publish_linear_acceleration` | `true` |
| `publish_angular_velocity` | `true` |

### `type="rtx_lidar"`

With no `config`, defaults to a basic **`Example_Rotary`** lidar. Set `config`
to a vendor model — `Example_Rotary_2D`, `Example_Solid_State`,
`OS0`/`OS1`/`OS2` (with a `variant`), `HESAI_XT32_SD10`, `SICK_microScan3`, ...
— to use its scan pattern instead.

Optional numeric overrides:

| Attribute | Meaning |
| --- | --- |
| `min_range` | Minimum range (m). |
| `max_range` | Maximum range (m). |
| `scan_rate` | Scan rate (Hz). |
| `channels` | Number of channels. |
| `max_returns` | Max returns per beam. |

Graph attributes:

| Attribute | Default |
| --- | --- |
| `laser_scan` | `true` |
| `laser_scan_topic` | `scan` |
| `point_cloud` | `false` |
| `point_cloud_topic` | `point_cloud` |

### `type="rgb_camera"` (requires `asset`)

| Attribute | Default |
| --- | --- |
| `rgb` | `true` |
| `rgb_topic` | `color/image_raw` |
| `instance` | `false` |
| `instance_topic` | `instance_segmentation` |
| `semantic` | `false` |
| `semantic_topic` | `semantic_segmentation` |
| `bbox_2d_tight` | `false` |
| `bbox_2d_tight_topic` | `bbox_2d_tight` |
| `bbox_2d_loose` | `false` |
| `bbox_2d_loose_topic` | `bbox_2d_loose` |
| `bbox_3d` | `false` |
| `bbox_3d_topic` | `bbox_3d` |
| `rgb_camera_info_topic` | derived from `rgb_topic` |
| `depth_camera_info_topic` | derived from `depth_topic` |
| `color_camera_subpath` | `RSD455/Camera_OmniVision_OV9782_Color` |

### `type="depth_camera"` (requires `asset`)

All `rgb_camera` attributes, plus:

| Attribute | Default |
| --- | --- |
| `depth` | `true` |
| `depth_topic` | `depth/image_rect_raw` |
| `depth_pcl` | `false` |
| `depth_pcl_topic` | `depth/points` |
| `initialize` | `false` (eagerly attach depth annotators) |
| `depth_camera_subpath` | `RSD455/Camera_Pseudo_Depth` |

**Camera notes:**

- `frame_id` is shared across every annotator publisher (rgb, depth,
  segmentation, bboxes). Per-channel frame_ids would need a custom graph.
- `CameraInfo` is always published; use `rgb_camera_info_topic` and
  `depth_camera_info_topic` to split color and depth info topics.
- The `*_camera_subpath` defaults target the RealSense D455 USD; override them
  for other vendor camera assets.
