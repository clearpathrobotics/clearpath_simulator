# Software License Agreement (proprietary)
#
# \authors   Ahmed Hamouda <ahmed.hamouda@rockwellautomation.com>
#
# Copyright (c) 2026 Rockwell Automation Technologies, Inc. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission of
# Rockwell Automation Technologies, Inc.
"""Public API of the urdf2usd package.

Converts a URDF/xacro to an Isaac Sim USD using a single ``<isaac_inputs>``
block embedded in the URDF (or supplied as a separate XML file). Outside that
block the URDF stays standards-compliant for non-Isaac consumers.
"""

from .parser import parse_isaac_inputs
from .urdf_io import (
    get_urdf_from_rsp,
    get_xml_from_file,
    replace_package_names,
    strip_isaac_inputs,
)
from .importer import import_urdf
from .post_process import (
    apply_isaac_inputs,
    apply_physics_scene,
    apply_link_materials,
    apply_joint_properties,
    add_sensors,
    apply_articulation_graphs,
)

__all__ = [
    "parse_isaac_inputs",
    "get_urdf_from_rsp",
    "get_xml_from_file",
    "replace_package_names",
    "strip_isaac_inputs",
    "import_urdf",
    "apply_isaac_inputs",
    "apply_physics_scene",
    "apply_link_materials",
    "apply_joint_properties",
    "add_sensors",
    "apply_articulation_graphs",
]
