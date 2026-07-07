# Software License Agreement (proprietary)
#
# \authors   Ahmed Hamouda <ahmed.hamouda@rockwellautomation.com>
#
# Copyright (c) 2026 Rockwell Automation Technologies, Inc. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission of
# Rockwell Automation Technologies, Inc.
"""Parse the ``<isaac_inputs>`` configuration block into a flat dict.

The block may be embedded in the URDF (inside ``<xacro:if value="false">``)
or supplied as a standalone XML file, which overrides the embedded one.
Multiple embedded blocks are merged in document order. All leaf values stay
as strings; the consumers (importer.py / post_process.py) coerce them.
"""
from __future__ import annotations

from typing import Any
from xml.dom import minidom
from xml.dom.minidom import Element


def parse_isaac_inputs(urdf_xml: str | None = None,
                       external_xml: str | None = None) -> dict[str, Any]:
    """Return the parsed ``<isaac_inputs>`` config.

    ``urdf_xml`` is searched for an embedded block; ``external_xml`` is an
    optional path to a standalone file that overrides it. Every key in the
    returned dict is always present (empty container if nothing matched).
    """
    inputs_nodes = _locate_inputs_nodes(urdf_xml=urdf_xml, external_xml=external_xml)
    if not inputs_nodes:
        return _empty_config()
    return _build_config(inputs_nodes)


def _empty_config() -> dict[str, Any]:
    return {
        "import_config": {},
        "articulation": {},
        "physics_scene": {},
        "physics_materials": {},
        "visual_materials": {},
        "link_materials": {},
        "joint_property_presets": {},
        "joint_properties": {},
        "sensors": [],
    }


def _locate_inputs_nodes(urdf_xml: str | None,
                         external_xml: str | None) -> list[Element]:
    """Locate every ``<isaac_inputs>`` element to parse.

    An external file fully replaces any embedded block(s).
    """
    if external_xml:
        print(f"Parsing Isaac inputs from: {external_xml}")
        dom = minidom.parse(external_xml)
        root = dom.documentElement
        if root.tagName == "isaac_inputs":
            return [root]
        if root.tagName == "robot":
            nodes = dom.getElementsByTagName("isaac_inputs")
            if nodes:
                return list(nodes)
        raise ValueError(
            "External XML root must be 'isaac_inputs' or 'robot' containing "
            f"<isaac_inputs>, got '{root.tagName}'"
        )

    if not urdf_xml:
        return []

    if "<isaac_inputs" not in urdf_xml:
        return []

    dom = minidom.parseString(urdf_xml)
    return list(dom.getElementsByTagName("isaac_inputs"))


def _iter_element_children(parent: Element):
    for child in parent.childNodes:
        if child.nodeType == child.ELEMENT_NODE:
            yield child


def _iter_all_children(parents: list[Element]):
    """Yield element children across every block in document order."""
    for parent in parents:
        yield from _iter_element_children(parent)


def _attrs(element: Element) -> dict[str, str]:
    """Return ``element``'s attributes as a plain ``{name: value}`` dict."""
    return {
        element.attributes.item(i).name: element.attributes.item(i).value
        for i in range(element.attributes.length)
    }


def _require_name(element: Element) -> str:
    name = element.getAttribute("name")
    if not name:
        raise ValueError(
            f"<{element.tagName}> inside <isaac_inputs> must have a 'name' attribute"
        )
    return name


def _build_config(inputs_nodes: list[Element]) -> dict[str, Any]:
    cfg = _empty_config()

    for child in _iter_all_children(inputs_nodes):
        tag = child.tagName
        attrs = _attrs(child)

        if tag == "import_config":
            _ensure_singleton(cfg["import_config"], tag)
            cfg["import_config"] = attrs

        elif tag == "articulation":
            _ensure_singleton(cfg["articulation"], tag)
            cfg["articulation"] = attrs

        elif tag == "physics_scene":
            _ensure_singleton(cfg["physics_scene"], tag)
            cfg["physics_scene"] = attrs

        elif tag == "physics_material":
            name = _require_name(child)
            _ensure_unique(cfg["physics_materials"], name, tag)
            cfg["physics_materials"][name] = _without(attrs, "name")

        elif tag == "visual_material":
            name = _require_name(child)
            _ensure_unique(cfg["visual_materials"], name, tag)
            cfg["visual_materials"][name] = _without(attrs, "name")

        elif tag == "drive":
            raise ValueError(
                "<drive> has been renamed to <joint_properties> (a generic per-joint "
                "preset). Update your URDF to use <joint_properties name=...> and "
                "<joint name=... properties=...>."
            )

        elif tag == "joint_properties":
            name = _require_name(child)
            _ensure_unique(cfg["joint_property_presets"], name, tag)
            cfg["joint_property_presets"][name] = _without(attrs, "name")

        elif tag == "link":
            link_name = _require_name(child)
            entry: dict[str, str] = {}
            if "physics_material" in attrs:
                entry["physics"] = attrs["physics_material"]
            if "visual_material" in attrs:
                entry["visual"] = attrs["visual_material"]
            if not entry:
                raise ValueError(
                    f"<link name='{link_name}'> inside <isaac_inputs> has no "
                    "physics_material or visual_material attribute (nothing to apply)"
                )
            _ensure_unique(cfg["link_materials"], link_name, tag)
            cfg["link_materials"][link_name] = entry

        elif tag == "joint":
            joint_name = _require_name(child)
            preset_name = child.getAttribute("properties")
            if not preset_name:
                raise ValueError(
                    f"<joint name='{joint_name}'> inside <isaac_inputs> must "
                    "have a 'properties' attribute referencing a <joint_properties> preset"
                )
            _ensure_unique(cfg["joint_properties"], joint_name, tag)
            cfg["joint_properties"][joint_name] = preset_name

        elif tag == "sensor":
            if "name" not in attrs:
                raise ValueError("<sensor> inside <isaac_inputs> must have a 'name' attribute")
            if "type" not in attrs:
                raise ValueError(
                    f"<sensor name='{attrs['name']}'> must specify a 'type' attribute"
                )
            cfg["sensors"].append(attrs)

        else:
            raise ValueError(f"Unknown element <{tag}> inside <isaac_inputs>")

    _validate_cross_references(cfg)
    return cfg


def _ensure_singleton(existing: dict, tag: str) -> None:
    if existing:
        raise ValueError(f"Multiple <{tag}> elements inside <isaac_inputs>; only one is allowed")


def _ensure_unique(existing: dict, name: str, tag: str) -> None:
    if name in existing:
        raise ValueError(f"Duplicate <{tag} name='{name}'> inside <isaac_inputs>")


def _without(attrs: dict[str, str], key: str) -> dict[str, str]:
    return {k: v for k, v in attrs.items() if k != key}


def _validate_cross_references(cfg: dict[str, Any]) -> None:
    """Verify that every name referenced by a binding tag was defined."""
    for joint_name, preset_name in cfg["joint_properties"].items():
        if preset_name not in cfg["joint_property_presets"]:
            raise ValueError(
                f"<joint name='{joint_name}' properties='{preset_name}'> references "
                "an undefined <joint_properties> preset"
            )

    for link_name, entry in cfg["link_materials"].items():
        if "physics" in entry and entry["physics"] not in cfg["physics_materials"]:
            raise ValueError(
                f"<link name='{link_name}' physics_material='{entry['physics']}'> "
                "references an undefined <physics_material>"
            )
        if "visual" in entry and entry["visual"] not in cfg["visual_materials"]:
            raise ValueError(
                f"<link name='{link_name}' visual_material='{entry['visual']}'> "
                "references an undefined <visual_material>"
            )
