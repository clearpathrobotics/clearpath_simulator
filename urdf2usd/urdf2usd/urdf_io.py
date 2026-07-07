# Software License Agreement (proprietary)
#
# \authors   Ahmed Hamouda <ahmed.hamouda@rockwellautomation.com>
#
# Copyright (c) 2026 Rockwell Automation Technologies, Inc. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission of
# Rockwell Automation Technologies, Inc.
"""URDF I/O helpers.

Kept Isaac Sim agnostic so they can be unit-tested without ``SimulationApp``.
Functional copies of the helpers in ``mujoco_ros2_control.urdf_to_mujoco_utils``.
"""
from __future__ import annotations

import re
from xml.dom import minidom


def get_xml_from_file(urdf_file: str) -> str:
    """Read a URDF/xacro file from disk and return its text content."""
    with open(urdf_file, encoding="utf-8") as file:
        return file.read()


def get_urdf_from_rsp(args=None) -> str:
    """Pull ``robot_description`` from the ``/robot_state_publisher`` node.

    ``rclpy`` is imported lazily so the module loads without a ROS 2 install.
    """
    import rclpy
    from rcl_interfaces.srv import GetParameters
    from rclpy.node import Node

    class ParameterClient(Node):
        def __init__(self, node_name: str = "/robot_state_publisher/get_parameters"):
            super().__init__("urdf2usd_parameter_client")
            self.node_name = node_name
            self.client = self.create_client(GetParameters, node_name)

        def get_params(self, params):
            while not self.client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting again...")
            req = GetParameters.Request()
            req.names = params
            future = self.client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            return future.result()

    rclpy.init(args=args)
    urdf = ""
    try:
        param_client = ParameterClient()
        response = param_client.get_params(["robot_description"])
        if response is not None:
            urdf = response.values[0].string_value
        else:
            param_client.get_logger().error("Failed to fetch robot_description parameter")
        param_client.destroy_node()
    finally:
        rclpy.try_shutdown()
    return urdf


def replace_package_names(xml_data: str) -> str:
    """Replace ``package://<pkg>/`` URIs with absolute filesystem paths.

    Packages are resolved via ``ament_index_python``; ``file://`` is stripped.
    """
    from ament_index_python.packages import get_package_share_directory

    pattern = r"package://([A-Za-z][A-Za-z0-9_]*)/"
    package_names = set(re.findall(pattern, xml_data))
    for package_name in package_names:
        old_string = f"package://{package_name}/"
        replace_string = f"{get_package_share_directory(package_name)}/"
        print(f"replacing {old_string} with {replace_string}")
        xml_data = xml_data.replace(old_string, replace_string)
    xml_data = xml_data.replace("file://", "")
    return xml_data


def strip_isaac_inputs(xml_data: str) -> str:
    """Remove every ``<isaac_inputs>`` element from a URDF text.

    The Isaac URDF importer rejects unknown root-level elements, so the block
    has to be removed before handing the URDF off. Returns the input unchanged
    if it contains no ``<isaac_inputs>``.
    """
    if "<isaac_inputs" not in xml_data:
        return xml_data

    dom = minidom.parseString(xml_data)
    # Collect first to avoid mutating the live NodeList while iterating.
    nodes = list(dom.getElementsByTagName("isaac_inputs"))
    for node in nodes:
        parent = node.parentNode
        if parent is not None:
            parent.removeChild(node)
            node.unlink()

    return dom.toxml()


def robot_name_from_urdf(xml_data: str) -> str | None:
    """Return the ``name`` attribute of the URDF ``<robot>`` element.

    Isaac Sim 6.x's URDF importer derives the robot name (and hence the USD
    default-prim path) from the source file's basename, so callers that write a
    temporary ``.urdf`` file use this to name it after the real robot. Returns
    ``None`` when the document can't be parsed or has no named ``<robot>``.
    """
    try:
        dom = minidom.parseString(xml_data)
    except Exception:
        return None
    robots = dom.getElementsByTagName("robot")
    if not robots:
        return None
    name = robots[0].getAttribute("name").strip()
    return name or None
