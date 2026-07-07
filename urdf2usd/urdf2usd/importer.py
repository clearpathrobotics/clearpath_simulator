# Software License Agreement (proprietary)
#
# \authors   Ahmed Hamouda <ahmed.hamouda@rockwellautomation.com>
#
# Copyright (c) 2026 Rockwell Automation Technologies, Inc. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission of
# Rockwell Automation Technologies, Inc.
"""Import a URDF string into the current USD stage via Isaac Sim's importer.

The single entry point is :func:`import_urdf`. It strips any ``<isaac_inputs>``
block, writes the URDF to a temporary file, and imports it with the Isaac Sim
6.x class API :class:`URDFImporter` / :class:`URDFImporterConfig`.

The imported USD is written to disk and opened as the current stage, then the
stage is normalized so :mod:`post_process` can author materials and sensors on
a uniform layout.

Isaac Sim modules are imported lazily so this module stays importable from a
plain Python interpreter (the parser tests rely on that).
"""

from __future__ import annotations

import os
import tempfile
from typing import Any

from . import _compat
from .urdf_io import robot_name_from_urdf, strip_isaac_inputs

__all__ = ["import_urdf"]


# Isaac Sim 6.x renamed several URDFCreateImportConfig attributes when it
# replaced the command-based importer with URDFImporterConfig. Map the old
# <import_config> attribute names onto their new field so existing xacro files
# keep working.
_RENAMED_ATTRS = {
    "self_collision": "allow_self_collision",
    "density": "link_density",
    "default_drive_strength": "override_joint_stiffness",
    "default_position_drive_damping": "override_joint_damping",
    "default_drive_type": "joint_target_type",
}

# Old attributes with no URDFImporterConfig equivalent. Dropped with a warning
# so a stale <import_config> block still imports instead of crashing.
_REMOVED_ATTRS = {
    "import_inertia_tensor": "inertia tensors are always imported",
    "override_joint_dynamics": "no longer configurable",
    "parse_mimic": "<mimic> joints are parsed automatically",
    "replace_cylinders_with_capsules": "no longer supported",
    "distance_scale": "unit scaling is automatic",
    "make_default_prim": "the robot is always the stage default prim",
}

# --- Isaac Sim 5.x command-API config ---------------------------------------
# The 5.x URDFCreateImportConfig object carries the *original* attribute names.
# Map every accepted <import_config> attribute (old OR new spelling) onto the
# 5.x ImportConfig attribute so a single xacro vocabulary drives both backends.
_V5_ATTR = {
    # Attributes shared by both spellings / both versions.
    "merge_fixed_joints": "merge_fixed_joints",
    "fix_base": "fix_base",
    "collision_from_visuals": "collision_from_visuals",
    "convex_decomp": "convex_decomp",
    "distance_scale": "distance_scale",
    "make_default_prim": "make_default_prim",
    "import_inertia_tensor": "import_inertia_tensor",
    "parse_mimic": "parse_mimic",
    "replace_cylinders_with_capsules": "replace_cylinders_with_capsules",
    "self_collision": "self_collision",
    "density": "density",
    "default_drive_strength": "default_drive_strength",
    "default_position_drive_damping": "default_position_drive_damping",
    "default_drive_type": "default_drive_type",
    # 6.x spellings routed back onto their 5.x equivalents.
    "allow_self_collision": "self_collision",
    "link_density": "density",
    "override_joint_stiffness": "default_drive_strength",
    "override_joint_damping": "default_position_drive_damping",
    "joint_target_type": "default_drive_type",
}

# 6.x-only attributes with no 5.x ImportConfig equivalent: dropped on the 5.x
# path (they only matter to the 6.x asset-transformer pipeline).
_V5_DROP = {
    "run_asset_transformer",
    "run_multi_physics_conversion",
    "collision_type",
}

# String tokens accepted for the drive type, mapped to the 5.x enum member name
# on ``_urdf.UrdfJointTargetType`` (resolved lazily so import stays safe).
_V5_DRIVE_TYPE_TOKENS = {
    "none": "JOINT_DRIVE_NONE",
    "position": "JOINT_DRIVE_POSITION",
    "velocity": "JOINT_DRIVE_VELOCITY",
}


# NOTE: this package intentionally pins run_asset_transformer to False. The
# converter still writes and opens the imported stage, then post_process authors
# materials/sensors on that stage before export. Keep this comment in sync with
# _PINNED_DEFAULTS to avoid backend-debug churn from stale guidance.
# run_multi_physics_conversion is pinned False so the multi-physics pass does not
# rewrite the joint attributes that post_process authors itself. Both may still
# be overridden per robot from <import_config>. post_process is robust to the
# transformer's restructured layout (links nested under a Geometry scope) via
# recursive link resolution.
_PINNED_DEFAULTS = {
    "run_asset_transformer": False,
    "run_multi_physics_conversion": False,
}


def _coerce_value(raw: str) -> Any:
    """Convert a string attribute into bool / int / float / str."""
    low = raw.strip().lower()
    if low in ("true", "false"):
        return low == "true"

    # int before float so "5" stays int
    try:
        return int(raw)
    except ValueError:
        pass
    try:
        return float(raw)
    except ValueError:
        pass

    return raw


def _build_import_kwargs(cfg: dict[str, str] | None) -> dict[str, Any]:
    """Translate a parsed ``<import_config>`` block into URDFImporterConfig kwargs.

    Applies the Isaac Sim 6.x renames, drops removed attributes with a warning,
    and validates the remainder against the live dataclass so typos surface.
    """
    import dataclasses

    from isaacsim.asset.importer.urdf import URDFImporterConfig  # lazy

    valid = {f.name for f in dataclasses.fields(URDFImporterConfig)}
    kwargs: dict[str, Any] = dict(_PINNED_DEFAULTS)

    for attr, raw in (cfg or {}).items():
        if attr in _REMOVED_ATTRS:
            print(
                f"[urdf2usd] WARNING: <import_config {attr}='{raw}'> is ignored "
                f"in Isaac Sim 6.x ({_REMOVED_ATTRS[attr]})."
            )
            continue

        # convex_decomp was a bool; the new API selects it via collision_type.
        if attr == "convex_decomp":
            if _coerce_value(raw):
                kwargs["collision_type"] = "Convex Decomposition"
            continue

        field = _RENAMED_ATTRS.get(attr, attr)
        if field not in valid:
            raise AttributeError(
                f"<import_config {attr}='{raw}'> is not a known "
                "URDFImporterConfig attribute"
            )

        value = _coerce_value(raw)
        # Old density='0' meant "auto"; the new link_density expects None.
        if field == "link_density" and not value:
            continue
        kwargs[field] = value

    return kwargs


def import_urdf(urdf_xml: str, import_cfg: dict[str, str] | None = None) -> str:
    """Import a URDF string into the current USD stage.

    ``urdf_xml`` is stripped of any embedded ``<isaac_inputs>`` block, written
    to a temporary ``.urdf`` file, and converted with whichever importer the
    running Isaac Sim exposes (6.x class API or 5.x command API, selected by
    capability detection). The robot's default-prim path is returned, e.g.
    ``/a300_isaac``. ``import_cfg`` holds ``<import_config>`` attribute overrides
    (``None`` / ``{}`` uses Isaac Sim defaults).
    """
    # Lazy imports so the module is safe to load outside Isaac Sim.
    import omni.kit.app

    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)

    clean_xml = strip_isaac_inputs(urdf_xml)

    # The importer derives the robot name (and thus the default-prim path) from
    # the URDF file's basename, so name the temp file after <robot name=...>.
    robot_name = robot_name_from_urdf(clean_xml) or "robot"
    work_dir = tempfile.mkdtemp(prefix="urdf2usd_import_")

    backend = _compat.urdf_importer_backend()
    print(f"[urdf2usd] URDF import backend: {backend} ({_compat.describe_backend()})")
    if backend == "v6":
        # The 6.x mesh loader has no GLB/glTF support; convert those visuals to
        # OBJ first. 5.x uses assimp and reads GLB natively, so it is skipped.
        from .mesh_convert import convert_unsupported_meshes

        clean_xml = convert_unsupported_meshes(
            clean_xml, os.path.join(work_dir, "meshes")
        )

    urdf_file = os.path.join(work_dir, f"{robot_name}.urdf")
    with open(urdf_file, "w") as handle:
        handle.write(clean_xml)

    if backend == "v6":
        prim_path = _import_urdf_v6(urdf_file, work_dir, import_cfg)
    else:
        prim_path = _import_urdf_v5(urdf_file, import_cfg)

    _clear_instanceable_flags(prim_path)
    _repair_joint_axes(prim_path)

    return prim_path


def _import_urdf_v6(
    urdf_file: str, work_dir: str, import_cfg: dict[str, str] | None
) -> str:
    """Import via the Isaac Sim 6.x :class:`URDFImporter` class API.

    Writes the USD next to the temp URDF and opens it as the current stage,
    returning the stage's default-prim path.
    """
    import omni.usd
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig

    config = URDFImporterConfig(
        urdf_path=urdf_file,
        usd_path=os.path.join(work_dir, "usd"),
        **_build_import_kwargs(import_cfg),
    )

    final_path = URDFImporter(config).import_urdf()

    if not omni.usd.get_context().open_stage(final_path):
        raise RuntimeError(f"Failed to open imported USD stage: {final_path}")

    stage = omni.usd.get_context().get_stage()
    default_prim = stage.GetDefaultPrim()
    if not default_prim or not default_prim.IsValid():
        raise RuntimeError(f"Imported stage has no default prim: {final_path}")
    return default_prim.GetPath().pathString


def _import_urdf_v5(urdf_file: str, import_cfg: dict[str, str] | None) -> str:
    """Import via the Isaac Sim 5.x command API.

    Opens a fresh stage, builds an ``URDFCreateImportConfig`` object, parses the
    URDF file, and imports the robot into the current stage. Returns the robot's
    prim path (also set as the stage default prim).
    """
    import omni.kit.commands
    import omni.usd

    # Import into a clean stage so the robot is the only content (mirrors the
    # 6.x path, which opens a freshly-written USD).
    omni.usd.get_context().new_stage()

    _, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
    _apply_v5_config(import_config, import_cfg)

    _, robot_model = omni.kit.commands.execute(
        "URDFParseFile", urdf_path=urdf_file, import_config=import_config
    )
    _, prim_path = omni.kit.commands.execute(
        "URDFImportRobot",
        urdf_path=urdf_file,
        urdf_robot=robot_model,
        import_config=import_config,
    )

    if not prim_path:
        raise RuntimeError(f"URDFImportRobot returned no prim path for {urdf_file}")

    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(prim_path)
    if not robot_prim.IsValid():
        raise RuntimeError(f"Imported robot prim path is not valid: {prim_path}")
    # make_default_prim should have set this, but enforce it so post_process can
    # rely on GetDefaultPrim() the same way it does on the 6.x path.
    stage.SetDefaultPrim(robot_prim)
    return prim_path


def _apply_v5_config(import_config: Any, cfg: dict[str, str] | None) -> None:
    """Populate a 5.x ``URDFCreateImportConfig`` object from ``<import_config>``.

    Accepts both the old (5.x) and new (6.x) attribute spellings and routes them
    onto the corresponding 5.x ImportConfig attribute; 6.x-only attributes are
    dropped with a note. ``make_default_prim`` defaults to ``True`` so the robot
    becomes the stage default prim (the 6.x importer does this unconditionally).
    """
    # Default the robot to the stage's default prim unless overridden below.
    if hasattr(import_config, "make_default_prim"):
        import_config.make_default_prim = True

    for attr, raw in (cfg or {}).items():
        if attr in _V5_DROP:
            print(
                f"[urdf2usd] NOTE: <import_config {attr}='{raw}'> is a 6.x-only "
                "setting and is ignored on Isaac Sim 5.x."
            )
            continue

        target = _V5_ATTR.get(attr)
        if target is None:
            print(
                f"[urdf2usd] WARNING: <import_config {attr}='{raw}'> is not a "
                "known Isaac Sim 5.x import setting; ignoring it."
            )
            continue

        if not hasattr(import_config, target):
            print(
                f"[urdf2usd] WARNING: this Isaac Sim 5.x build has no "
                f"ImportConfig.{target}; ignoring <import_config {attr}='{raw}'>."
            )
            continue

        if target == "default_drive_type":
            value = _v5_drive_type(raw)
            if value is None:
                continue
        else:
            value = _coerce_value(raw)
            # density='0' historically meant "auto"; leave the 5.x default.
            if target == "density" and not value:
                continue

        setattr(import_config, target, value)


def _v5_drive_type(raw: str) -> Any:
    """Resolve a drive-type token / int to the 5.x ``UrdfJointTargetType`` enum."""
    token = str(raw).strip().lower()
    member = _V5_DRIVE_TYPE_TOKENS.get(token)
    if member is None:
        # Allow raw ints (the enum's underlying value) to pass through.
        coerced = _coerce_value(raw)
        if isinstance(coerced, int):
            return coerced
        print(
            f"[urdf2usd] WARNING: unknown default_drive_type='{raw}'; "
            f"expected one of {sorted(_V5_DRIVE_TYPE_TOKENS)}. Ignoring it."
        )
        return None
    try:
        from isaacsim.asset.importer.urdf import _urdf  # lazy

        return getattr(_urdf.UrdfJointTargetType, member)
    except Exception:
        print(
            f"[urdf2usd] WARNING: could not resolve UrdfJointTargetType.{member}; "
            "ignoring default_drive_type."
        )
        return None


def _repair_joint_axes(robot_prim_path: str) -> None:
    """Restore the ``physics:axis`` token on joints with an arbitrary URDF axis.

    When a joint's URDF ``<axis>`` is not a principal axis (e.g. the DO100
    mecanum rollers, tilted 45 deg), Isaac's importer bakes the orientation
    into ``localRot0``/``localRot1`` but leaves ``physics:axis`` empty, so PhysX
    treats the joint as locked. Set the missing token to ``"X"`` (the axis the
    importer baked around) to restore the free DOF. Valid tokens are left as-is.
    """
    import omni.usd
    from pxr import Usd, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        return

    repaired = []
    for prim in Usd.PrimRange(robot_prim):
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue
        axis_attr = prim.GetAttribute("physics:axis")
        if axis_attr and axis_attr.Get():  # already a valid X / Y / Z token
            continue
        if prim.IsA(UsdPhysics.RevoluteJoint):
            UsdPhysics.RevoluteJoint(prim).CreateAxisAttr().Set(UsdPhysics.Tokens.x)
        else:
            UsdPhysics.PrismaticJoint(prim).CreateAxisAttr().Set(UsdPhysics.Tokens.x)
        repaired.append(prim.GetName())

    if repaired:
        print(
            f"[urdf2usd] repaired {len(repaired)} joint(s) with a missing "
            f"physics:axis token (set to X): {', '.join(repaired)}"
        )


def _clear_instanceable_flags(robot_prim_path: str) -> None:
    """Clear the ``instanceable`` flag on every prim under the robot root.

    Material and sensor authoring in :mod:`post_process` needs direct access to
    the prims, which is not possible inside an instanced subtree.
    """
    import omni.usd
    from pxr import Usd

    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if not robot_prim.IsValid():
        raise ValueError(f"Imported robot prim path is not valid: {robot_prim_path}")

    for prim in Usd.PrimRange(robot_prim):
        if prim.IsInstanceable():
            prim.SetInstanceable(False)
