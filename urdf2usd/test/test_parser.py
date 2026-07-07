# Software License Agreement (proprietary)
#
# \authors   Ahmed Hamouda <ahmed.hamouda@rockwellautomation.com>
#
# Copyright (c) 2026 Rockwell Automation Technologies, Inc. All Rights Reserved.
"""Unit tests for the ``urdf2usd`` parser and URDF I/O helpers.

These tests deliberately have no Isaac Sim / ROS dependency so they can be
executed via ``pytest`` from a plain Python 3 environment.
"""
from __future__ import annotations

import sys
import textwrap
from pathlib import Path

import pytest

# Make the package importable when tests are run from the source tree (i.e.
# before colcon has installed it).
_PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PKG_ROOT))

from clearpath_simulator.urdf2usd.urdf2usd.parser import parse_isaac_inputs  # noqa: E402
from clearpath_simulator.urdf2usd.urdf2usd.urdf_io import strip_isaac_inputs  # noqa: E402


_FULL_URDF = textwrap.dedent(
    """\
    <?xml version="1.0"?>
    <robot name="test_robot">
      <link name="base_link"/>
      <link name="left_wheel"/>
      <link name="right_wheel"/>
      <joint name="left_wheel_jnt" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel"/>
      </joint>
      <joint name="right_wheel_jnt" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
      </joint>

      <isaac_inputs>
        <import_config merge_fixed_joints="true" fix_base="false"/>
        <articulation chassis_link="base_link" namespace="test"
                      clock="true"
                      joint_states="true" joint_states_topic="/joint_states"
                      joint_commands="true" joint_commands_topic="/joint_command"
                      odom="true" odom_topic="/odom"
                      tf="true" tf_topic="/tf"/>

        <physics_material name="rubber" static_friction="0.9" dynamic_friction="0.8"/>
        <visual_material name="red" diffuse="0.8 0.1 0.1" roughness="0.6"/>

        <link name="left_wheel" physics_material="rubber" visual_material="red"/>
        <link name="right_wheel" physics_material="rubber"/>

        <joint_properties name="diff_drive_wheel" control_mode="velocity"
                          stiffness="0.0" damping="1000.0"/>

        <joint name="left_wheel_jnt" properties="diff_drive_wheel"/>
        <joint name="right_wheel_jnt" properties="diff_drive_wheel"/>

        <sensor name="imu" type="imu" link="base_link"
                ros2="true" topic="/imu/data" frame_id="imu_link"/>
        <sensor name="front_lidar" type="rtx_lidar"
                link="base_link"
                asset="{nucleus}/Isaac/Sensors/SICK/microScan3/SICK_microScan3.usd"
                ros2="true"
                laser_scan="true" laser_scan_topic="scan"/>
      </isaac_inputs>
    </robot>
    """
)


def test_parse_full_embedded_block():
    cfg = parse_isaac_inputs(urdf_xml=_FULL_URDF)

    assert cfg["import_config"] == {"merge_fixed_joints": "true", "fix_base": "false"}
    assert cfg["articulation"]["chassis_link"] == "base_link"
    assert cfg["articulation"]["namespace"] == "test"
    # Articulation-level ROS 2 toggles + topics
    assert cfg["articulation"]["joint_states"] == "true"
    assert cfg["articulation"]["joint_states_topic"] == "/joint_states"
    assert cfg["articulation"]["odom_topic"] == "/odom"
    assert cfg["articulation"]["clock"] == "true"

    assert set(cfg["physics_materials"]) == {"rubber"}
    assert cfg["physics_materials"]["rubber"]["static_friction"] == "0.9"
    assert set(cfg["visual_materials"]) == {"red"}

    assert cfg["link_materials"] == {
        "left_wheel": {"physics": "rubber", "visual": "red"},
        "right_wheel": {"physics": "rubber"},
    }

    assert set(cfg["joint_property_presets"]) == {"diff_drive_wheel"}
    assert cfg["joint_property_presets"]["diff_drive_wheel"]["control_mode"] == "velocity"

    assert cfg["joint_properties"] == {
        "left_wheel_jnt": "diff_drive_wheel",
        "right_wheel_jnt": "diff_drive_wheel",
    }

    # Sensors preserve declaration order and per-sensor ros2 / topic attrs
    assert [s["name"] for s in cfg["sensors"]] == ["imu", "front_lidar"]
    assert cfg["sensors"][0]["type"] == "imu"
    assert cfg["sensors"][0]["ros2"] == "true"
    assert cfg["sensors"][0]["topic"] == "/imu/data"
    assert cfg["sensors"][1]["laser_scan_topic"] == "scan"


def test_parse_returns_empty_when_no_block():
    urdf = "<robot name='x'><link name='a'/></robot>"
    cfg = parse_isaac_inputs(urdf_xml=urdf)
    assert cfg["import_config"] == {}
    assert cfg["physics_materials"] == {}
    assert cfg["sensors"] == []


def test_parse_standalone_external_file(tmp_path: Path):
    inputs_xml = textwrap.dedent(
        """\
        <?xml version="1.0"?>
        <isaac_inputs>
          <joint_properties name="p" stiffness="1.0" damping="2.0"/>
          <joint name="j" properties="p"/>
        </isaac_inputs>
        """
    )
    file_path = tmp_path / "inputs.xml"
    file_path.write_text(inputs_xml)

    cfg = parse_isaac_inputs(external_xml=str(file_path))
    assert cfg["joint_property_presets"] == {"p": {"stiffness": "1.0", "damping": "2.0"}}
    assert cfg["joint_properties"] == {"j": "p"}


def test_external_overrides_embedded(tmp_path: Path):
    """When both sources supply a block, the external file wins."""
    external_xml = textwrap.dedent(
        """\
        <?xml version="1.0"?>
        <isaac_inputs>
          <articulation namespace="from_external"/>
        </isaac_inputs>
        """
    )
    file_path = tmp_path / "inputs.xml"
    file_path.write_text(external_xml)

    cfg = parse_isaac_inputs(urdf_xml=_FULL_URDF, external_xml=str(file_path))
    assert cfg["articulation"] == {"namespace": "from_external"}
    # External file overrides embedded: joint property presets from embedded must NOT leak through.
    assert cfg["joint_property_presets"] == {}


def test_joint_without_properties_attr_raises():
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <joint name='j'/>
          </isaac_inputs>
        </robot>
        """
    )
    with pytest.raises(ValueError, match="must have a 'properties' attribute"):
        parse_isaac_inputs(urdf_xml=urdf)


def test_link_without_material_attr_raises():
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <link name='left'/>
          </isaac_inputs>
        </robot>
        """
    )
    with pytest.raises(ValueError, match="no physics_material or visual_material"):
        parse_isaac_inputs(urdf_xml=urdf)


def test_undefined_joint_properties_reference_raises():
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <joint name='j' properties='missing'/>
          </isaac_inputs>
        </robot>
        """
    )
    with pytest.raises(ValueError, match="undefined <joint_properties> preset"):
        parse_isaac_inputs(urdf_xml=urdf)


def test_undefined_material_reference_raises():
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <link name='l' physics_material='missing'/>
          </isaac_inputs>
        </robot>
        """
    )
    with pytest.raises(ValueError, match="undefined <physics_material>"):
        parse_isaac_inputs(urdf_xml=urdf)


def test_duplicate_joint_properties_name_raises():
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <joint_properties name='p'/>
            <joint_properties name='p'/>
          </isaac_inputs>
        </robot>
        """
    )
    with pytest.raises(ValueError, match="Duplicate <joint_properties name='p'>"):
        parse_isaac_inputs(urdf_xml=urdf)


def test_multiple_blocks_are_merged():
    """Several <isaac_inputs> blocks aggregate like scattered <gazebo> tags."""
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <import_config merge_fixed_joints='true'/>
            <joint_properties name='wheel' damping='1.0'/>
            <joint name='left_wheel_jnt' properties='wheel'/>
            <sensor name='imu' type='imu'/>
          </isaac_inputs>
          <link name='somewhere'/>
          <isaac_inputs>
            <articulation namespace='robot'/>
            <joint name='right_wheel_jnt' properties='wheel'/>
            <sensor name='lidar' type='rtx_lidar'/>
          </isaac_inputs>
        </robot>
        """
    )
    cfg = parse_isaac_inputs(urdf_xml=urdf)
    # Singletons come from whichever block defined them.
    assert cfg["import_config"] == {"merge_fixed_joints": "true"}
    assert cfg["articulation"] == {"namespace": "robot"}
    # Bindings from both blocks are present and cross-reference the shared preset.
    assert cfg["joint_properties"] == {
        "left_wheel_jnt": "wheel",
        "right_wheel_jnt": "wheel",
    }
    # Sensors keep document order across blocks.
    assert [s["name"] for s in cfg["sensors"]] == ["imu", "lidar"]


def test_singleton_split_across_blocks_raises():
    """import_config may appear at most once, even across separate blocks."""
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <import_config merge_fixed_joints='true'/>
          </isaac_inputs>
          <isaac_inputs>
            <import_config fix_base='true'/>
          </isaac_inputs>
        </robot>
        """
    )
    with pytest.raises(ValueError, match="Multiple <import_config> elements"):
        parse_isaac_inputs(urdf_xml=urdf)


def test_legacy_drive_tag_raises_with_migration_hint():
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <drive name='d'/>
          </isaac_inputs>
        </robot>
        """
    )
    with pytest.raises(ValueError, match="renamed to <joint_properties>"):
        parse_isaac_inputs(urdf_xml=urdf)


def test_unknown_tag_raises():
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <foobar/>
          </isaac_inputs>
        </robot>
        """
    )
    with pytest.raises(ValueError, match="Unknown element <foobar>"):
        parse_isaac_inputs(urdf_xml=urdf)


def test_strip_isaac_inputs_removes_block_and_leaves_valid_urdf():
    stripped = strip_isaac_inputs(_FULL_URDF)
    assert "isaac_inputs" not in stripped
    # Standard URDF content must still be there
    assert "left_wheel" in stripped
    assert "right_wheel_jnt" in stripped

    # Should re-parse without error
    from xml.dom import minidom
    dom = minidom.parseString(stripped)
    assert dom.documentElement.tagName == "robot"


def test_strip_isaac_inputs_is_noop_without_block():
    urdf = "<robot name='x'><link name='a'/></robot>"
    assert strip_isaac_inputs(urdf) == urdf


# --- Coverage for the full <isaac_inputs> menu (mirrors example.urdf.xacro) ---


_TESTBED_URDF = textwrap.dedent(
    """\
    <robot name="testbed">
      <link name="base_link"/>
      <isaac_inputs>
        <physics_scene time_steps_per_second="240" gravity="9.81"
                       friction_type="patch"/>
        <import_config merge_fixed_joints="true" fix_base="false"/>
        <articulation chassis_link="base_link" namespace="a300"
                      clock="true"
                      joint_states="true" joint_commands="true"
                      odom="true" tf="true"
                      cmd_vel="true" drive_controller="holonomic"
                      wheel_radius="0.05"
                      wheel_joints="fl fr rl rr"
                      mecanum_angles="-135 -45 -45 -135"/>
        <physics_material name="rubber" static_friction="1.0"
                          dynamic_friction="0.9" restitution="0.0"
                          friction_combine_mode="max"
                          restitution_combine_mode="min"/>
        <visual_material name="black" diffuse="0.05 0.05 0.05"/>
        <link name="fl_wheel" physics_material="rubber" visual_material="black"/>
        <joint_properties name="wheel" control_mode="velocity" damping="1000.0"/>
        <joint name="fl" properties="wheel"/>
        <sensor name="imu" type="imu" link="imu_link" ros2="true"/>
        <sensor name="front_lidar" type="rtx_lidar" link="front_laser_link"
                asset="{nucleus}/Isaac/Sensors/SICK/microScan3/SICK_microScan3.usd"
                laser_scan="true"/>
        <sensor name="front_rgb" type="rgb_camera" link="cam_link"
                asset="{nucleus}/Isaac/Sensors/RealSense/D455/rsd455.usd"
                semantic="true"/>
        <sensor name="front_depth" type="depth_camera" link="cam_link"
                asset="{nucleus}/Isaac/Sensors/RealSense/D455/rsd455.usd"
                depth_pcl="true"/>
      </isaac_inputs>
    </robot>
    """
)


def test_physics_scene_parsed():
    cfg = parse_isaac_inputs(urdf_xml=_TESTBED_URDF)
    assert cfg["physics_scene"] == {
        "time_steps_per_second": "240",
        "gravity": "9.81",
        "friction_type": "patch",
    }


def test_physics_material_combine_modes_parsed():
    cfg = parse_isaac_inputs(urdf_xml=_TESTBED_URDF)
    rubber = cfg["physics_materials"]["rubber"]
    assert rubber["friction_combine_mode"] == "max"
    assert rubber["restitution_combine_mode"] == "min"


def test_articulation_cmd_vel_drive_attrs_pass_through():
    cfg = parse_isaac_inputs(urdf_xml=_TESTBED_URDF)
    art = cfg["articulation"]
    assert art["cmd_vel"] == "true"
    assert art["drive_controller"] == "holonomic"
    assert art["wheel_joints"] == "fl fr rl rr"
    assert art["mecanum_angles"] == "-135 -45 -45 -135"


def test_all_sensor_types_parsed_in_order():
    cfg = parse_isaac_inputs(urdf_xml=_TESTBED_URDF)
    assert [(s["name"], s["type"]) for s in cfg["sensors"]] == [
        ("imu", "imu"),
        ("front_lidar", "rtx_lidar"),
        ("front_rgb", "rgb_camera"),
        ("front_depth", "depth_camera"),
    ]
    # Off-the-shelf sensors keep their vendor asset path (placeholder intact).
    lidar = next(s for s in cfg["sensors"] if s["name"] == "front_lidar")
    assert lidar["asset"].startswith("{nucleus}/Isaac/Sensors/SICK")


def test_lidar_config_and_numeric_overrides_pass_through():
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <sensor name='basic_lidar' type='rtx_lidar' link='laser_link'/>
            <sensor name='device_lidar' type='rtx_lidar' link='laser_link'
                    config='OS1' variant='OS1_REV7_128ch10hz1024res'
                    max_range='40.0' channels='128' scan_rate='10'/>
          </isaac_inputs>
        </robot>
        """
    )
    cfg = parse_isaac_inputs(urdf_xml=urdf)
    basic = next(s for s in cfg["sensors"] if s["name"] == "basic_lidar")
    # A bare lidar carries no config; post_process defaults it to Example_Rotary.
    assert "config" not in basic
    assert "asset" not in basic
    device = next(s for s in cfg["sensors"] if s["name"] == "device_lidar")
    assert device["config"] == "OS1"
    assert device["variant"] == "OS1_REV7_128ch10hz1024res"
    assert device["max_range"] == "40.0"
    assert device["channels"] == "128"
    assert device["scan_rate"] == "10"


def test_sensor_without_type_raises():
    urdf = textwrap.dedent(
        """\
        <robot name='x'>
          <isaac_inputs>
            <sensor name='s' link='l'/>
          </isaac_inputs>
        </robot>
        """
    )
    with pytest.raises(ValueError, match="must specify a 'type' attribute"):
        parse_isaac_inputs(urdf_xml=urdf)
