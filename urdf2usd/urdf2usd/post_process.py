# Software License Agreement (proprietary)
#
# \authors   Ahmed Hamouda <ahmed.hamouda@rockwellautomation.com>
#
# Copyright (c) 2026 Rockwell Automation Technologies, Inc. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission of
# Rockwell Automation Technologies, Inc.
"""Apply a parsed ``<isaac_inputs>`` config to an imported robot.

The entry point :func:`apply_isaac_inputs` runs, in order: physics scene,
link materials, joint properties, sensors, and the articulation-level ROS 2
graphs (clock / joint_states / joint_commands / odom / tf). Isaac Sim imports
are deferred to the call sites so the module loads in a plain interpreter.
"""

from __future__ import annotations

from typing import Any

from . import _compat

__all__ = [
    "apply_isaac_inputs",
    "apply_physics_scene",
    "apply_link_materials",
    "apply_joint_properties",
    "add_sensors",
    "apply_articulation_graphs",
]


# Value coercion helpers (strings out of XML -> Python objects)

def _as_bool(raw: str, default: bool = False) -> bool:
    if raw is None:
        return default
    return str(raw).strip().lower() in ("1", "true", "yes")


def _as_float(raw: str | None, default: float | None = None) -> float | None:
    if raw is None or str(raw).strip() == "":
        return default
    return float(raw)


def _as_int(raw: str | None, default: int | None = None) -> int | None:
    if raw is None or str(raw).strip() == "":
        return default
    return int(raw)


def _as_vec(raw: str | None, default):
    """Parse a whitespace-separated numeric vector like ``"1 0 0 0"``."""
    if raw is None or str(raw).strip() == "":
        return default
    return [float(x) for x in str(raw).split()]


def _as_str_list(raw: str | None, default=None):
    """Parse a comma/whitespace separated token list like ``"a, b c"``."""
    if raw is None or str(raw).strip() == "":
        return list(default) if default else []
    return [tok for tok in str(raw).replace(",", " ").split() if tok]


def _as_float_list(raw: str | None, default=None):
    """Parse a comma/whitespace separated float list like ``"-45, 45"``."""
    if raw is None or str(raw).strip() == "":
        return list(default) if default else []
    return [float(tok) for tok in str(raw).replace(",", " ").split() if tok]


def _resolve_asset(raw: str | None) -> str | None:
    """Expand ``{nucleus}`` placeholders in an asset path."""
    if raw is None:
        return None
    if "{nucleus}" in raw:
        root = _compat.get_assets_root_path()  # tolerant of module moves
        if root is None:
            raise FileNotFoundError(
                "Nucleus server not reachable; cannot expand {nucleus} "
                f"placeholder in asset path '{raw}'"
            )
        return raw.replace("{nucleus}", root.rstrip("/"))
    return raw


# Top-level orchestration

def apply_isaac_inputs(
    robot_prim_path: str, cfg: dict[str, Any], urdf_xml: str | None = None
) -> None:
    """Apply every section of a parsed ``<isaac_inputs>`` config to a robot.

    ``robot_prim_path`` is the path returned by :func:`importer.import_urdf` and
    ``cfg`` is the output of :func:`parser.parse_isaac_inputs`. ``urdf_xml`` (the
    expanded URDF) is optional and lets material binding relocate geometry that
    ``merge_fixed_joints`` consolidated. Empty sections are skipped.
    """
    if cfg.get("physics_scene"):
        apply_physics_scene(cfg["physics_scene"])

    if cfg.get("link_materials"):
        apply_link_materials(
            robot_prim_path,
            cfg["link_materials"],
            cfg.get("physics_materials", {}),
            cfg.get("visual_materials", {}),
            urdf_xml=urdf_xml,
        )

    if cfg.get("joint_properties"):
        apply_joint_properties(
            robot_prim_path,
            cfg.get("joint_property_presets", {}),
            cfg["joint_properties"],
        )

    if cfg.get("sensors"):
        add_sensors(
            robot_prim_path, cfg["sensors"], cfg.get("articulation", {}), urdf_xml
        )

    if cfg.get("articulation"):
        apply_articulation_graphs(robot_prim_path, cfg["articulation"])


# Physics scene

def apply_physics_scene(scene_cfg: dict[str, Any]) -> None:
    """Bake PhysX scene settings from ``<physics_scene>`` onto the USD stage.

    Authors a self-contained PhysicsScene so the robot runs at the right rate
    straight from the USD. Supported attributes: ``time_steps_per_second``
    (physics sub-stepping rate), ``gravity`` (m/s^2, default 9.81),
    ``friction_type`` (``patch`` / ``oneDirectional`` / ``twoDirectional``) and
    the patch-merging tolerances ``friction_offset_threshold`` /
    ``friction_correlation_distance``.
    """
    import omni.usd
    from pxr import Gf, Sdf, UsdPhysics

    rate = _as_int(scene_cfg.get("time_steps_per_second"))
    gravity = _as_float(scene_cfg.get("gravity"), 9.81)
    friction_type = (scene_cfg.get("friction_type") or "").strip()
    friction_offset = _as_float(scene_cfg.get("friction_offset_threshold"), None)
    friction_corr = _as_float(scene_cfg.get("friction_correlation_distance"), None)
    has_friction = bool(friction_type) or friction_offset is not None or friction_corr is not None
    if rate is None and "gravity" not in scene_cfg and not has_friction:
        return

    stage = omni.usd.get_context().get_stage()
    prim = next(
        (p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)),
        None,
    )
    if prim is None:
        prim = UsdPhysics.Scene.Define(stage, Sdf.Path("/PhysicsScene")).GetPrim()

    scene = UsdPhysics.Scene(prim)
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(gravity)

    physx_scene = None
    if rate is not None or has_friction:
        try:
            from pxr import PhysxSchema
            physx_scene = PhysxSchema.PhysxSceneAPI.Apply(prim)
        except ImportError:
            physx_scene = None

    if rate is not None:
        if physx_scene is not None:
            physx_scene.CreateTimeStepsPerSecondAttr().Set(rate)
        else:
            prim.CreateAttribute(
                "physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt
            ).Set(rate)

    if physx_scene is not None:
        if friction_type:
            physx_scene.CreateFrictionTypeAttr().Set(friction_type)
        if friction_offset is not None:
            physx_scene.CreateFrictionOffsetThresholdAttr().Set(friction_offset)
        if friction_corr is not None:
            physx_scene.CreateFrictionCorrelationDistanceAttr().Set(friction_corr)
    elif has_friction:
        if friction_type:
            prim.CreateAttribute(
                "physxScene:frictionType", Sdf.ValueTypeNames.Token
            ).Set(friction_type)
        if friction_offset is not None:
            prim.CreateAttribute(
                "physxScene:frictionOffsetThreshold", Sdf.ValueTypeNames.Float
            ).Set(friction_offset)
        if friction_corr is not None:
            prim.CreateAttribute(
                "physxScene:frictionCorrelationDistance", Sdf.ValueTypeNames.Float
            ).Set(friction_corr)

    print(
        f"[urdf2usd] physics scene: time_steps_per_second="
        f"{rate if rate is not None else 'default'}, gravity={gravity}, "
        f"friction_type={friction_type or 'default'}, "
        f"friction_offset_threshold={friction_offset if friction_offset is not None else 'default'}, "
        f"friction_correlation_distance={friction_corr if friction_corr is not None else 'default'}"
    )


# Materials

def _resolve_link_prim(stage, Usd, robot_prim_path: str, link_name: str):
    """Return the prim for *link_name* under *robot_prim_path*, or ``None``.

    Tries the direct child path first, then a recursive name search so links
    reparented by ``merge_fixed_joints`` are still located.
    """
    direct = stage.GetPrimAtPath(f"{robot_prim_path}/{link_name}")
    if direct.IsValid():
        return direct
    root = stage.GetPrimAtPath(robot_prim_path)
    if not root.IsValid():
        return None
    for prim in Usd.PrimRange(root):
        if prim.GetName() == link_name:
            return prim
    return None


def _resolve_link_path(robot_prim_path: str, link_name: str) -> str:
    """Return the absolute prim path of *link_name*, resolved against the stage.

    Isaac Sim 6.0's URDF importer nests the articulation links under a
    ``Geometry`` scope (e.g. ``<robot>/Geometry/base_link``) instead of directly
    under the robot root as in 5.1. ``merge_fixed_joints`` can also reparent
    links. Resolve the real path via a recursive name search so the articulation
    graphs point at the surviving prim; fall back to the flat 5.1-style path if
    the link cannot be found (which keeps the previous behaviour/error message).
    """
    import omni.usd
    from pxr import Usd

    flat = f"{robot_prim_path}/{link_name}"
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        return flat
    prim = _resolve_link_prim(stage, Usd, robot_prim_path, link_name)
    if prim is not None:
        resolved = prim.GetPath().pathString
        if resolved != flat:
            print(
                f"[urdf2usd] resolved link '{link_name}' to '{resolved}' "
                f"(6.0 importer nests links under a scope / merge_fixed_joints)"
            )
        return resolved
    print(
        f"[urdf2usd] WARNING: could not resolve link '{link_name}' under "
        f"'{robot_prim_path}'; using flat path '{flat}'"
    )
    return flat


def _sanitize_usd_name(name: str) -> str:
    """Mirror the importer's prim-name sanitisation (non ``[A-Za-z0-9_]`` -> ``_``)."""
    return "".join(c if (c.isalnum() or c == "_") else "_" for c in name)


def _link_visual_mesh_stems(urdf_xml: str) -> dict[str, list[str]]:
    """Map each URDF link to the sanitised stems of its ``<visual>`` mesh files.

    ``merge_fixed_joints`` moves a merged link's visuals under the surviving
    body as ``<body>/visuals/<stem>`` prims named after the mesh file, so the
    stems are how that geometry is relocated.
    """
    import os
    import xml.dom.minidom as minidom

    stems: dict[str, list[str]] = {}
    try:
        dom = minidom.parseString(urdf_xml)
    except Exception:
        return stems
    for link in dom.getElementsByTagName("link"):
        name = link.getAttribute("name")
        out: list[str] = []
        for vis in link.getElementsByTagName("visual"):
            for mesh in vis.getElementsByTagName("mesh"):
                fn = mesh.getAttribute("filename")
                if fn:
                    stem = os.path.splitext(os.path.basename(fn))[0]
                    out.append(_sanitize_usd_name(stem))
        if out:
            stems[name] = out
    return stems


def _bind_merged_link_visual(stage, UsdShade, link_prim, stems, material) -> list[str]:
    """Bind *material* onto the consolidated visual prims of a merged link.

    Walks up from *link_prim* to the nearest surviving body (first ancestor
    carrying a ``visuals`` child scope) and binds every visual prim whose name
    matches one of *stems* (``stem`` or ``stem_<N>``). Returns the prim names
    that were bound.
    """
    import re

    if not stems:
        return []
    vis_scope = None
    p = link_prim
    while p and p.IsValid() and p.GetPath().pathString != "/":
        child = p.GetChild("visuals")
        if child and child.IsValid():
            vis_scope = child
            break
        p = p.GetParent()
    if vis_scope is None:
        return []
    patterns = [re.compile(r"^" + re.escape(s) + r"(_\d+)?$") for s in stems]
    bound: list[str] = []
    for child in vis_scope.GetChildren():
        cname = child.GetName()
        if any(pat.match(cname) for pat in patterns):
            UsdShade.MaterialBindingAPI.Apply(child).Bind(
                material, bindingStrength=UsdShade.Tokens.strongerThanDescendants
            )
            bound.append(cname)
    return bound


_COMBINE_MODES = ("average", "min", "multiply", "max")


def _apply_combine_modes(stage, mat_path: str, name: str, attrs: dict[str, str]) -> None:
    """Author PhysX friction/restitution combine modes on a physics material.

    PhysX uses the higher-priority mode of the two contacting materials
    (max > multiply > min > average), so ``max`` makes this material's friction
    dominate the contact. Accepted modes: average, min, multiply, max.
    """
    fmode = (attrs.get("friction_combine_mode") or "").strip().lower()
    rmode = (attrs.get("restitution_combine_mode") or "").strip().lower()
    if not fmode and not rmode:
        return
    from pxr import PhysxSchema

    prim = stage.GetPrimAtPath(mat_path)
    if not prim or not prim.IsValid():
        print(f"[urdf2usd] WARNING: physics_material '{name}': prim {mat_path} missing; cannot set combine mode")
        return
    api = PhysxSchema.PhysxMaterialAPI.Apply(prim)
    for label, mode, create in (
        ("friction", fmode, api.CreateFrictionCombineModeAttr),
        ("restitution", rmode, api.CreateRestitutionCombineModeAttr),
    ):
        if not mode:
            continue
        if mode not in _COMBINE_MODES:
            print(
                f"[urdf2usd] WARNING: physics_material '{name}': invalid {label}_combine_mode "
                f"'{mode}' (use one of {', '.join(_COMBINE_MODES)}); skipping"
            )
            continue
        create().Set(mode)
        print(f"[urdf2usd] physics_material '{name}': {label}CombineMode={mode}")


def apply_link_materials(
    robot_prim_path: str,
    link_materials: dict[str, dict[str, str]],
    physics_materials: dict[str, dict[str, str]],
    visual_materials: dict[str, dict[str, str]],
    urdf_xml: str | None = None,
) -> None:
    """Bind physics and visual materials onto per-link prims.

    Materials are created once under ``{robot_prim_path}/Looks/`` then
    bound to the matching link's collision (physics) or visual (visual)
    subtree.
    """
    import omni.usd

    _compat.require_experimental_materials("<link_material>/<physics_material> binding")
    _compat.require_experimental_prims("<link_material>/<physics_material> binding")
    from isaacsim.core.experimental.materials import RigidBodyMaterial
    from isaacsim.core.experimental.prims import GeomPrim
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    stage = omni.usd.get_context().get_stage()
    stem_map = _link_visual_mesh_stems(urdf_xml) if urdf_xml else {}

    # Create the physics material presets once.
    phys_mat_prims: dict[str, Any] = {}
    for name, attrs in physics_materials.items():
        mat_path = f"{robot_prim_path}/Looks/physics_{name}"
        phys_mat_prims[name] = RigidBodyMaterial(
            mat_path,
            static_frictions=[_as_float(attrs.get("static_friction"), 0.5)],
            dynamic_frictions=[_as_float(attrs.get("dynamic_friction"), 0.5)],
            restitutions=[_as_float(attrs.get("restitution"), 0.0)],
        )
        _apply_combine_modes(stage, mat_path, name, attrs)

    # Materialise the visual material presets once (PreviewSurface).
    visual_mat_prims: dict[str, Any] = {}
    for name, attrs in visual_materials.items():
        mat_path = f"{robot_prim_path}/Looks/visual_{name}"
        material = UsdShade.Material.Define(stage, Sdf.Path(mat_path))
        shader = UsdShade.Shader.Define(stage, Sdf.Path(mat_path + "/Shader"))
        shader.CreateIdAttr("UsdPreviewSurface")
        diffuse = _as_vec(attrs.get("diffuse"), [0.5, 0.5, 0.5])
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            tuple(diffuse[:3])
        )
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(
            _as_float(attrs.get("roughness"), 0.5)
        )
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(
            _as_float(attrs.get("metallic"), 0.0)
        )
        material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
        visual_mat_prims[name] = material

    # Bind to each link. merge_fixed_joints can collapse a link into a parent
    # body, reparenting (and renaming) its geometry, so resolve each link by
    # path then by name and skip anything that no longer exists.
    for link_name, bindings in link_materials.items():
        link_prim = _resolve_link_prim(stage, Usd, robot_prim_path, link_name)
        if link_prim is None:
            print(
                f"[urdf2usd] WARNING: <link name='{link_name}'> in <isaac_inputs> "
                f"matches no prim under {robot_prim_path} (collapsed by "
                "merge_fixed_joints?); skipping its material binding."
            )
            continue

        # When merge_fixed_joints collapses a link, its geometry is moved to
        # ``<body>/visuals/<mesh-stem>`` and the link prim is left as an empty
        # frame with no Gprim descendants.
        own_gprims = any(
            p.IsA(UsdGeom.Gprim) for p in Usd.PrimRange(link_prim)
        )

        phys_name = bindings.get("physics")
        if phys_name:
            if phys_name not in phys_mat_prims:
                raise ValueError(
                    f"<link name='{link_name}' physics_material='{phys_name}'> "
                    "references an undefined <physics_material>"
                )
            if own_gprims:
                # Bind only to colliders the importer authored. Targeting the
                # whole link would add a CollisionAPI to the visual meshes too,
                # turning the GLB into a triangle-mesh collider PhysX rejects
                # for dynamic bodies (the robot then jumps).
                collider_paths = [
                    p.GetPath().pathString
                    for p in Usd.PrimRange(link_prim)
                    if p.HasAPI(UsdPhysics.CollisionAPI)
                ]
                if collider_paths:
                    GeomPrim(
                        collider_paths, apply_collision_apis=False
                    ).apply_physics_materials(
                        phys_mat_prims[phys_name], weaker_than_descendants=False
                    )
                else:
                    # Link has visuals but no collider of its own; don't build
                    # one from the visual mesh (triangle mesh PhysX can't use on
                    # a dynamic body). Skip it.
                    print(
                        f"[urdf2usd] WARNING: <link name='{link_name}' "
                        f"physics_material='{phys_name}'> has no collider of its "
                        "own; skipping the binding. Add a <collision> to this "
                        "link in the URDF, or set collision_from_visuals=\"true\" "
                        "in <import_config>, if it is meant to collide."
                    )
            else:
                print(
                    f"[urdf2usd] WARNING: <link name='{link_name}'> has no collision "
                    "geometry of its own (merged by merge_fixed_joints); skipping "
                    "its physics_material binding."
                )

        visual_name = bindings.get("visual")
        if visual_name:
            if visual_name not in visual_mat_prims:
                raise ValueError(
                    f"<link name='{link_name}' visual_material='{visual_name}'> "
                    "references an undefined <visual_material>"
                )
            material = visual_mat_prims[visual_name]
            if own_gprims:
                UsdShade.MaterialBindingAPI.Apply(link_prim).Bind(
                    material,
                    bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                )
            else:
                bound = _bind_merged_link_visual(
                    stage, UsdShade, link_prim, stem_map.get(link_name, []), material
                )
                if not bound:
                    print(
                        f"[urdf2usd] WARNING: <link name='{link_name}'> was merged by "
                        "merge_fixed_joints and its consolidated visual geometry could "
                        "not be located; skipping its visual_material binding."
                    )


# Joint properties

# Maps schema attr -> (Articulation setter, kwarg name on the setter).
_PER_DOF_SETTERS: dict[str, tuple[str, str]] = {
    "stiffness": ("set_dof_gains", "stiffnesses"),
    "damping": ("set_dof_gains", "dampings"),
    "armature": ("set_dof_armatures", "armatures"),
    "static_friction": ("set_dof_friction_properties", "static_frictions"),
    "dynamic_friction": ("set_dof_friction_properties",  "dynamic_frictions"),
    "viscous_friction": ("set_dof_friction_properties", "viscous_frictions"),
    "max_effort": ("set_dof_max_efforts", "max_efforts"),
    "max_velocity": ("set_dof_max_velocities", "max_velocities"),
    "lower_limit": ("set_dof_limits", "lower"),
    "upper_limit": ("set_dof_limits", "upper"),
}


def apply_joint_properties(
    robot_prim_path: str,
    presets: dict[str, dict[str, str]],
    bindings: dict[str, str],
) -> None:
    """Apply per-DOF properties to selected joints.

    For each ``<joint name=... properties=...>`` binding, look up the DOF index
    by name and apply the preset via the ``Articulation`` API.
    """
    if not bindings:
        return

    _compat.require_experimental_prims("<joint properties> authoring")
    from isaacsim.core.experimental.prims import Articulation

    art = Articulation([robot_prim_path])
    dof_names = list(art.dof_names)

    # Group joints by preset so each setter is called once per preset.
    by_preset: dict[str, list[int]] = {}
    for joint_name, preset_name in bindings.items():
        if joint_name not in dof_names:
            raise ValueError(
                f"<joint name='{joint_name}'> in <isaac_inputs> does not match any "
                f"DOF of {robot_prim_path}. Known DOFs: {dof_names}"
            )
        by_preset.setdefault(preset_name, []).append(dof_names.index(joint_name))

    for preset_name, dof_indices in by_preset.items():
        attrs = presets[preset_name]
        _apply_preset_to_dofs(art, attrs, dof_indices)


def _apply_preset_to_dofs(art: Any, attrs: dict[str, str], dof_indices: list[int]) -> None:
    """Apply one preset's attrs to a list of DOF indices on the articulation."""
    grouped: dict[str, dict[str, list[float]]] = {}
    for attr, raw in attrs.items():
        if attr not in _PER_DOF_SETTERS:
            continue
        setter, kwarg = _PER_DOF_SETTERS[attr]
        value = _as_float(raw)
        if value is None:
            continue
        grouped.setdefault(setter, {})[kwarg] = [value]

    for setter_name, kwargs in grouped.items():
        getattr(art, setter_name)(dof_indices=dof_indices, **kwargs)

    control_mode = attrs.get("control_mode")
    if control_mode:
        art.switch_dof_control_mode(control_mode, dof_indices=dof_indices)

    drive_type = attrs.get("drive_type")
    if drive_type:
        art.set_dof_drive_types(drive_type, dof_indices=dof_indices)


# Articulation-level ROS 2 graphs

def apply_articulation_graphs(robot_prim_path: str, art_cfg: dict[str, str]) -> None:
    """Wire clock / joint_states / joint_commands / odom / tf graphs."""
    from .omnigraphs import (
        Ros2ClockGraph,
        Ros2JointStatesGraph,
        Ros2OdometryGraph,
        Ros2TfPubGraph,
    )

    namespace = art_cfg.get("namespace", "")
    chassis_link = art_cfg.get("chassis_link")
    # Isaac 6.0 nests links under a Geometry scope; resolve the real path once.
    chassis_path = _resolve_link_path(robot_prim_path, chassis_link) if chassis_link else None

    if _as_bool(art_cfg.get("clock")):
        Ros2ClockGraph(og_path=f"{robot_prim_path}/ros2_graphs/clock").create_graph()
        # Ros2ClockGraph has no clock_topic override; PublishClock defaults to "/clock".

    pub = _as_bool(art_cfg.get("joint_states"))
    sub = _as_bool(art_cfg.get("joint_commands"))
    if pub or sub:
        if not chassis_link:
            raise ValueError(
                "<articulation joint_states='true'> (or joint_commands) requires "
                "the chassis_link attribute"
            )
        Ros2JointStatesGraph(
            og_path=f"{robot_prim_path}/ros2_graphs/joint_interface",
            node_namespace=namespace,
            art_root_path=chassis_path,
            pub_topic=art_cfg.get("joint_states_topic", "/joint_states"),
            sub_topic=art_cfg.get("joint_commands_topic", "/joint_command"),
            publisher=pub,
            subscriber=sub,
            sub_move_robot=True,
        ).create_graph()

    if _as_bool(art_cfg.get("odom")):
        if not chassis_link:
            raise ValueError("<articulation odom='true'> requires the chassis_link attribute")
        Ros2OdometryGraph(
            og_path=f"{robot_prim_path}/ros2_graphs/odometry",
            node_namespace=namespace,
            art_root_prim=chassis_path,
            odom_pub_topic=art_cfg.get("odom_topic", "/odom"),
            tf_pub_topic=art_cfg.get("tf_topic", "/tf"),
            tf_robot_pub=_as_bool(art_cfg.get("tf")),
            chassis_prim=robot_prim_path,
            chassis_link_name=chassis_link,
        ).create_graph()
    elif _as_bool(art_cfg.get("tf")):
        # TF only (no odom) - use the standalone TF graph.
        if not chassis_link:
            raise ValueError("<articulation tf='true'> requires the chassis_link attribute")
        Ros2TfPubGraph(
            og_path=f"{robot_prim_path}/ros2_graphs/tf",
            node_namespace=namespace,
            target_prim=chassis_path,
            parent_prim=robot_prim_path,
            pub_topic=art_cfg.get("tf_topic", "/tf"),
        ).create_graph()

    if _as_bool(art_cfg.get("cmd_vel")):
        if not chassis_link:
            raise ValueError("<articulation cmd_vel='true'> requires the chassis_link attribute")
        _apply_cmd_vel_drive(robot_prim_path, chassis_link, namespace, art_cfg)


def _apply_cmd_vel_drive(
    robot_prim_path: str,
    chassis_link: str,
    namespace: str,
    art_cfg: dict[str, str],
) -> None:
    """Bake an off-the-shelf cmd_vel drive controller into the robot graph."""
    from .omnigraphs import Ros2DriveGraph

    drive_type = art_cfg.get("drive_controller", "diff").strip().lower()
    art_root_path = _resolve_link_path(robot_prim_path, chassis_link)
    # Namespaced cmd_vel is the default; disable it only when explicitly requested.
    cmd_vel_namespace = namespace if _as_bool(art_cfg.get("cmd_vel_use_namespace"), True) else ""
    wheel_radius = _as_float(art_cfg.get("wheel_radius"), 0.1)

    front_joints = _as_str_list(art_cfg.get("front_wheel_joints"))
    rear_joints = _as_str_list(art_cfg.get("rear_wheel_joints"))
    wheel_joints = _as_str_list(art_cfg.get("wheel_joints"))

    if drive_type == "holonomic":
        # HolonomicRobotUsdSetup reads these per-joint attributes off the USD.
        mecanum_angles = _as_float_list(art_cfg.get("mecanum_angles"))
        _author_mecanum_wheel_attrs(robot_prim_path, wheel_joints, wheel_radius, mecanum_angles)

    Ros2DriveGraph(
        og_path=f"{robot_prim_path}/ros2_graphs/cmd_vel_drive",
        node_namespace=cmd_vel_namespace,
        art_root_path=art_root_path,
        # HolonomicRobotUsdSetup walks descendants of robot_root to find the
        # mecanum-tagged joints (which sit at <root>/joints/..., not under
        # the articulation-root link).
        robot_root_path=robot_prim_path,
        drive_type=drive_type,
        cmd_vel_topic=art_cfg.get("cmd_vel_topic", "cmd_vel"),
        wheel_radius=wheel_radius,
        wheel_distance=_as_float(art_cfg.get("wheel_distance"), 0.4),
        front_wheel_joints=front_joints,
        rear_wheel_joints=rear_joints,
        wheel_base=_as_float(art_cfg.get("wheel_base"), 0.4),
        steer_wheel_joints=_as_str_list(art_cfg.get("steer_wheel_joints")),
        front_wheel_radius=_as_float(art_cfg.get("front_wheel_radius")),
        rear_wheel_radius=_as_float(art_cfg.get("rear_wheel_radius")),
        wheel_joints=wheel_joints,
        com_path=_resolve_link_path(
            robot_prim_path, art_cfg.get("com_link", chassis_link)
        ),
    ).create_graph()


def _author_mecanum_wheel_attrs(
    robot_prim_path: str,
    wheel_joints: list[str],
    wheel_radius: float,
    mecanum_angles: list[float],
) -> None:
    """Tag the mecanum wheel joints with the attributes Isaac's holonomic setup
    node reads (``isaacmecanumwheel:radius`` / ``:angle``)."""
    import omni.usd
    from pxr import Sdf

    if not wheel_joints:
        raise ValueError(
            "<articulation drive_controller='holonomic'> requires 'wheel_joints'"
        )
    if len(mecanum_angles) != len(wheel_joints):
        raise ValueError(
            "holonomic drive needs one 'mecanum_angles' entry per 'wheel_joints' "
            f"(got {len(mecanum_angles)} angles for {len(wheel_joints)} joints)"
        )

    stage = omni.usd.get_context().get_stage()
    for joint_name, angle in zip(wheel_joints, mecanum_angles):
        prim = _find_joint_prim(stage, robot_prim_path, joint_name)
        if prim is None:
            raise ValueError(
                f"holonomic drive: wheel joint '{joint_name}' not found under "
                f"'{robot_prim_path}'"
            )
        prim.CreateAttribute("isaacmecanumwheel:radius", Sdf.ValueTypeNames.Float).Set(
            float(wheel_radius)
        )
        prim.CreateAttribute("isaacmecanumwheel:angle", Sdf.ValueTypeNames.Float).Set(
            float(angle)
        )


def _find_joint_prim(stage, robot_prim_path: str, joint_name: str):
    """Locate a joint prim by name anywhere under the robot prim."""
    from pxr import Usd

    # Common layout first: all joints grouped under ``<robot>/joints``.
    direct = stage.GetPrimAtPath(f"{robot_prim_path}/joints/{joint_name}")
    if direct and direct.IsValid():
        return direct
    root = stage.GetPrimAtPath(robot_prim_path)
    if root and root.IsValid():
        for prim in Usd.PrimRange(root):
            if prim.GetName() == joint_name:
                return prim
    return None


# Sensors

def add_sensors(
    robot_prim_path: str,
    sensors: list[dict[str, str]],
    art_cfg: dict[str, str],
    urdf_xml: str | None = None,
) -> None:
    """Create each sensor + its optional ROS 2 graph."""
    namespace = art_cfg.get("namespace", "")
    for sensor in sensors:
        stype = sensor["type"]
        dispatch = _SENSOR_DISPATCH.get(stype)
        if dispatch is None:
            raise ValueError(
                f"Unknown sensor type '{stype}' (sensor name='{sensor.get('name')}'). "
                f"Supported types: {sorted(_SENSOR_DISPATCH)}"
            )
        dispatch(robot_prim_path, sensor, namespace, urdf_xml)


def _nearest_rigid_ancestor(stage, path: str):
    """Return the path of the first prim at/above *path* with RigidBodyAPI.

    Used to decide whether a resolved sensor mount actually rides a rigid body
    (survived ``merge_fixed_joints`` intact) or is a detached frame that needs
    rebuilding. Keyed on the RigidBodyAPI, not on the prim's name or type.
    """
    from pxr import UsdPhysics

    prim = stage.GetPrimAtPath(path)
    while prim and prim.IsValid():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim.GetPath().pathString
        if prim.IsPseudoRoot():
            break
        prim = prim.GetParent()
    return None


def _urdf_link_transform(urdf_xml: str, link_name: str, base_link_name: str):
    """Return the ``base_link`` -> ``link_name`` transform as a ``Gf.Matrix4d``.

    Walks the URDF joint chain child -> parent, accumulating each ``<origin>``
    (xyz + rpy), so the offset a massless mount link lost when
    ``merge_fixed_joints`` collapsed it can be restored. Returns ``None`` if the
    chain cannot be traced back to ``base_link_name``.
    """
    import math
    import xml.dom.minidom as minidom
    from pxr import Gf

    try:
        dom = minidom.parseString(urdf_xml)
    except Exception:
        return None

    joints: dict[str, tuple[str, list[float], list[float]]] = {}
    for j in dom.getElementsByTagName("joint"):
        child_el = j.getElementsByTagName("child")
        parent_el = j.getElementsByTagName("parent")
        if not child_el or not parent_el:
            continue
        child = child_el[0].getAttribute("link")
        parent = parent_el[0].getAttribute("link")
        origins = j.getElementsByTagName("origin")
        xyz = _as_vec(origins[0].getAttribute("xyz"), [0.0, 0.0, 0.0]) if origins else [0.0, 0.0, 0.0]
        rpy = _as_vec(origins[0].getAttribute("rpy"), [0.0, 0.0, 0.0]) if origins else [0.0, 0.0, 0.0]
        joints[child] = (parent, xyz, rpy)

    accum = Gf.Matrix4d(1.0)
    cur = link_name
    seen: set[str] = set()
    while cur != base_link_name:
        entry = joints.get(cur)
        if entry is None or cur in seen:
            print(
                f"[urdf2usd] WARNING: could not trace mount link '{link_name}' back "
                f"to base '{base_link_name}' in the URDF; sensor mount pose may be off"
            )
            return None
        seen.add(cur)
        parent, xyz, rpy = entry
        roll, pitch, yaw = (list(rpy) + [0.0, 0.0, 0.0])[:3]
        local = Gf.Matrix4d(1.0)
        local.SetRotateOnly(
            Gf.Rotation(Gf.Vec3d(1, 0, 0), math.degrees(roll))
            * Gf.Rotation(Gf.Vec3d(0, 1, 0), math.degrees(pitch))
            * Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw))
        )
        local.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in (list(xyz) + [0.0, 0.0, 0.0])[:3]]))
        accum = accum * local
        cur = parent
    return accum


def _author_mount_frame(stage, mount_path: str, matrix, source_prim=None) -> str:
    """(Re)author an Xform at *mount_path* with the exact local pose *matrix*.

    A stale detached frame at the mount link's old location (``source_prim``) is
    removed so we do not leave an empty duplicate at the robot root. When
    *matrix* is ``None`` the frame is authored at identity.
    """
    from pxr import Gf, Sdf, UsdGeom

    if source_prim is not None and source_prim.IsValid():
        src = source_prim.GetPath().pathString
        if src != mount_path:
            try:
                stage.RemovePrim(Sdf.Path(src))
            except Exception as exc:
                print(
                    f"[urdf2usd] WARNING: failed to remove stale mount frame '{src}': {exc}"
                )

    xform = UsdGeom.Xform.Define(stage, Sdf.Path(mount_path))
    xform.ClearXformOpOrder()
    xform.AddTransformOp().Set(matrix if matrix is not None else Gf.Matrix4d(1.0))
    return mount_path


def _resolve_sensor_mount(
    robot_prim_path: str, sensor: dict[str, str], urdf_xml: str | None
) -> str:
    """Resolve a sensor's mount link to a prim path that rides the chassis.

    Generic fix for ``merge_fixed_joints`` collapsing the massless URDF
    sensor-mount links: the importer leaves them as detached Xform frames at the
    robot root (no rigid-body ancestor, joint offset lost), so a sensor authored
    there stays pinned in world at the wrong pose.

    The decision is keyed on the RigidBodyAPI, not on the sensor type:

    * If the resolved mount already has a rigid-body ancestor, it survived the
      merge with its correct pose -> use it as-is.
    * Otherwise rebuild it as a child of the articulation base body, placed at
      the transform recovered from the URDF fixed-joint chain
      (``base_link`` -> ... -> mount link). Anything authored under it then
      follows the chassis at the correct pose.

    Sensors that carry their own RigidBodyAPI (e.g. a RealSense USD) are moved
    back out to the robot root and fixed-jointed by their own authoring path,
    because a rigid body may not be nested under another rigid body.
    """
    import omni.usd
    from pxr import Usd

    link = sensor.get("link")
    if not link:
        raise ValueError(
            f"<sensor name='{sensor.get('name')}'> is missing the required "
            "'link' attribute"
        )
    stage = omni.usd.get_context().get_stage()

    prim = _resolve_link_prim(stage, Usd, robot_prim_path, link)
    if prim is not None and _nearest_rigid_ancestor(stage, prim.GetPath().pathString):
        # Mount survived the merge attached to a rigid body -> pose is correct.
        return prim.GetPath().pathString

    base_body = _resolve_articulation_base_body(stage, robot_prim_path)
    if not base_body:
        # No rigid body to anchor to; keep the previous (flat) behaviour.
        return prim.GetPath().pathString if prim is not None else f"{robot_prim_path}/{link}"

    base_link_name = base_body.rsplit("/", 1)[-1]
    matrix = _urdf_link_transform(urdf_xml, link, base_link_name) if urdf_xml else None
    mount_path = f"{base_body}/{link}"
    _author_mount_frame(stage, mount_path, matrix, source_prim=prim)
    print(
        f"[urdf2usd] Sensor mount '{link}' was collapsed by merge_fixed_joints; "
        f"rebuilt it under base body '{base_body}' at path '{mount_path}'"
    )
    return mount_path


def _resolve_sensor_target_path(
    stage,
    robot_prim_path: str,
    mount_path: str,
    sensor_name: str,
    frame_id: str,
) -> str:
    """Return the prim path where a camera asset should be authored.

    Prefer reusing an existing prim matching ``frame_id`` (absolute path,
    robot-relative path, or unique prim name under the robot subtree). This
    keeps TF frame names aligned with message headers even when the frame is not
    directly under ``robot_prim_path``.
    """
    from pxr import Usd

    if not frame_id:
        return f"{robot_prim_path}/{sensor_name}"

    token = str(frame_id).strip()
    if not token:
        return f"{robot_prim_path}/{sensor_name}"

    if token.startswith("/"):
        return token if stage.GetPrimAtPath(token).IsValid() else token

    if "/" in token:
        rel_path = f"{robot_prim_path}/{token}"
        if stage.GetPrimAtPath(rel_path).IsValid():
            return rel_path
        abs_path = f"/{token}"
        if stage.GetPrimAtPath(abs_path).IsValid():
            return abs_path
        return rel_path

    mount_prim = stage.GetPrimAtPath(mount_path)
    if mount_prim.IsValid() and mount_prim.GetName() == token:
        return mount_path

    robot_prim = stage.GetPrimAtPath(robot_prim_path)
    if robot_prim.IsValid():
        matches: list[str] = []
        for prim in Usd.PrimRange(robot_prim):
            if prim.GetName() == token:
                matches.append(prim.GetPath().pathString)
        if len(matches) == 1:
            return matches[0]
        if len(matches) > 1:
            matches.sort(key=len)
            print(
                f"[urdf2usd] WARNING: frame_id '{token}' matched multiple prims "
                f"under '{robot_prim_path}'. Using '{matches[0]}'."
            )
            return matches[0]

    return f"{robot_prim_path}/{token}"


def _add_imu(
    robot_prim_path: str,
    sensor: dict[str, str],
    namespace: str,
    urdf_xml: str | None = None,
) -> None:
    """Author an Isaac Sim IMU sensor prim under the sensor's mount link.

    Isaac Sim 6.x moved the IMU into ``isaacsim.sensors.experimental.physics``
    and split it into an authoring class (``IMU``) and a runtime reader; asset
    generation only needs to author the ``IsaacImuSensor`` prim. Isaac Sim 5.x
    ships the single ``isaacsim.sensors.physics.IMUSensor`` class, whose
    constructor authors the prim. The backend is chosen by capability detection.
    """
    sensor_name = sensor["name"]
    mount_path = _resolve_sensor_mount(robot_prim_path, sensor, urdf_xml)
    sensor_path = f"{mount_path}/{sensor_name}"

    if _compat.imu_backend() == "experimental":
        _author_imu_experimental(sensor_path, sensor)
    else:
        _author_imu_legacy(sensor_path, sensor_name, sensor)

    if _as_bool(sensor.get("ros2"), True):
        from .omnigraphs import Ros2ImuGraph

        Ros2ImuGraph(
            og_path=f"{robot_prim_path}/ros2_graphs/{sensor_name}",
            imu_prim=sensor_path,
            frame_id=sensor.get("frame_id", sensor["link"]),
            topic=sensor.get("topic", "imu"),
            node_namespace=namespace,
            publish_orientation=_as_bool(sensor.get("publish_orientation"), True),
            publish_linear_acceleration=_as_bool(
                sensor.get("publish_linear_acceleration"), True
            ),
            publish_angular_velocity=_as_bool(
                sensor.get("publish_angular_velocity"), True
            ),
        ).create_graph()


def _author_imu_experimental(sensor_path: str, sensor: dict[str, str]) -> None:
    """Author the IMU prim via the Isaac Sim 6.x experimental ``IMU`` class.

    Transforms are batched (one row per sensor) and the old ``frequency``
    construction argument was dropped.
    """
    from isaacsim.sensors.experimental.physics import IMU

    IMU.create(
        sensor_path,
        translations=[_as_vec(sensor.get("translation"), [0.0, 0.0, 0.0])],
        orientations=[_as_vec(sensor.get("orientation"), [1.0, 0.0, 0.0, 0.0])],
        linear_acceleration_filter_size=_as_int(
            sensor.get("linear_acceleration_filter_size"), 10
        ),
        angular_velocity_filter_size=_as_int(
            sensor.get("angular_velocity_filter_size"), 10
        ),
        orientation_filter_size=_as_int(sensor.get("orientation_filter_size"), 10),
    )

def _author_imu_legacy(
    sensor_path: str, sensor_name: str, sensor: dict[str, str]
) -> None:
    """Author the IMU prim via the Isaac Sim 5.x ``IMUSensor`` class.

    Constructing ``IMUSensor`` with a not-yet-existing ``prim_path`` authors the
    ``IsaacImuSensor`` prim. Pose is passed as numpy arrays; the ``frequency``
    argument is left at its default (runtime concern, not needed for authoring).
    """
    import numpy as np
    from isaacsim.sensors.physics import IMUSensor

    IMUSensor(
        prim_path=sensor_path,
        name=sensor_name,
        translation=np.array(
            _as_vec(sensor.get("translation"), [0.0, 0.0, 0.0]), dtype=float
        ),
        orientation=np.array(
            _as_vec(sensor.get("orientation"), [1.0, 0.0, 0.0, 0.0]), dtype=float
        ),
        linear_acceleration_filter_size=_as_int(
            sensor.get("linear_acceleration_filter_size"), 10
        ),
        angular_velocity_filter_size=_as_int(
            sensor.get("angular_velocity_filter_size"), 10
        ),
        orientation_filter_size=_as_int(sensor.get("orientation_filter_size"), 10),
    )


# Friendly ``<sensor>`` attribute names mapped to the ``omni:sensor:Core:*``
# USD attributes the RTX lidar prim actually carries. The create command only
# assigns an override when the chosen config's prim already has the attribute,
# so unsupported names are silently ignored rather than corrupting the prim.
_RTX_LIDAR_ATTR_ALIASES: dict[str, tuple[str, str]] = {
    "min_range": ("omni:sensor:Core:nearRangeM", "float"),
    "near_range": ("omni:sensor:Core:nearRangeM", "float"),
    "max_range": ("omni:sensor:Core:farRangeM", "float"),
    "far_range": ("omni:sensor:Core:farRangeM", "float"),
    "scan_rate": ("omni:sensor:Core:scanRateBaseHz", "float"),
    "number_of_channels": ("omni:sensor:Core:numberOfChannels", "int"),
    "channels": ("omni:sensor:Core:numberOfChannels", "int"),
    "max_returns": ("omni:sensor:Core:maxReturns", "int"),
}


def _rtx_lidar_attribute_overrides(sensor: dict[str, str]) -> dict[str, Any]:
    """Collect numeric ``omni:sensor:Core:*`` overrides from a ``<sensor>`` block.

    Both friendly aliases (``min_range``, ``scan_rate``, ...) and raw
    ``omni:sensor:Core:*`` attribute names are accepted; the latter are coerced
    with the same rule as their alias when one exists, else passed as a float.
    """
    overrides: dict[str, Any] = {}
    for key, (attr, kind) in _RTX_LIDAR_ATTR_ALIASES.items():
        raw = sensor.get(key)
        if raw is None or str(raw).strip() == "":
            continue
        overrides[attr] = _as_int(raw) if kind == "int" else _as_float(raw)
    return overrides


def _add_rtx_lidar(
    robot_prim_path: str,
    sensor: dict[str, str],
    namespace: str,
    urdf_xml: str | None = None,
) -> None:
    """Create an Isaac Sim RTX lidar and wire it to a ROS 2 graph.

    Creation is unified around a config name. An omitted ``config`` (and no
    ``asset``) falls back to the built-in ``Example_Rotary`` rotary lidar, so a
    bare ``<sensor type="rtx_lidar">`` still yields a working sensor. Numeric
    parameters (``min_range``/``max_range``/``scan_rate``/``channels``/...) are
    applied on top of the chosen config, so a vendor config can be tweaked
    without abandoning it. An explicit ``asset`` USD path is honored as an
    escape hatch and takes the place of a config.
    """
    import omni.kit.commands
    from pxr import Gf
    from .omnigraphs import Ros2RtxLidarGraph

    sensor_name = sensor["name"]
    mount_path = _resolve_sensor_mount(robot_prim_path, sensor, urdf_xml)

    asset_path = _resolve_asset(sensor.get("asset"))
    config = sensor.get("config")
    if not config and not asset_path:
        config = "Example_Rotary"

    translation = _as_vec(sensor.get("translation"), [0.0, 0.0, 0.0])
    orientation = _as_vec(sensor.get("orientation"), [1.0, 0.0, 0.0, 0.0])

    create_kwargs: dict[str, Any] = {
        "path": f"/{sensor_name}",
        "parent": mount_path,
        "translation": Gf.Vec3d(*(float(v) for v in translation[:3])),
        "orientation": Gf.Quatd(*(float(v) for v in orientation[:4])),
    }
    if config:
        create_kwargs["config"] = config
    if asset_path:
        create_kwargs["usd_path"] = asset_path
    variant = sensor.get("variant")
    if variant:
        create_kwargs["variant"] = variant
    create_kwargs.update(_rtx_lidar_attribute_overrides(sensor))

    _, lidar_prim = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar", **create_kwargs
    )
    sensor_prim_path = lidar_prim.GetPath().pathString

    if not _as_bool(sensor.get("ros2"), True):
        return

    Ros2RtxLidarGraph(
        og_path=f"{robot_prim_path}/ros2_graphs/{sensor_name}",
        frame_id=sensor.get("frame_id", sensor["link"]),
        node_namespace=namespace,
        sensor_prim=sensor_prim_path,
        laser_scan_pub=_as_bool(sensor.get("laser_scan"), True),
        laser_scan_topic=sensor.get("laser_scan_topic", "scan"),
        point_cloud_pub=_as_bool(sensor.get("point_cloud"), False),
        point_cloud_topic=sensor.get("point_cloud_topic", "point_cloud"),
    ).create_graph()


def _add_camera(
    robot_prim_path: str,
    sensor: dict[str, str],
    namespace: str,
    is_depth: bool,
    urdf_xml: str | None = None,
) -> None:
    """Create an RGB or depth camera and (optionally) its ROS 2 graph.

    With an ``asset`` set, reference the vendor USD and wire its render product
    into a Ros2CameraGraph. Vanilla cameras (no ``asset``) are not supported
    yet, so raise pointing the user at the off-the-shelf path.
    """
    import omni.usd

    from .omnigraphs import Ros2CameraGraph

    sensor_name = sensor["name"]
    frame_id = sensor.get("frame_id", sensor["link"])
    asset_path = _resolve_asset(sensor.get("asset"))
    rgb_topic = sensor.get("rgb_topic", "color/image_raw")
    depth_topic = sensor.get("depth_topic", "depth/image_rect_raw")
    rgb_camera_info_topic = sensor.get(
        "rgb_camera_info_topic",
        _camera_info_topic_for_stream(rgb_topic),
    )
    depth_camera_info_topic = sensor.get(
        "depth_camera_info_topic",
        _camera_info_topic_for_stream(depth_topic),
    )

    if not asset_path:
        raise NotImplementedError(
            f"<sensor name='{sensor_name}' type='{sensor['type']}'>: vanilla "
            "camera creation (no 'asset' attribute) is not implemented yet. "
            "Provide an Isaac Sim camera USD asset via 'asset' (e.g. the "
            "Intel RealSense D455 at "
            "{nucleus}/Isaac/Sensors/RealSense/D455/rsd455.usd)."
        )

    mount_path = _resolve_sensor_mount(robot_prim_path, sensor, urdf_xml)
    stage = omni.usd.get_context().get_stage()
    target_path = _resolve_sensor_target_path(
        stage=stage,
        robot_prim_path=robot_prim_path,
        mount_path=mount_path,
        sensor_name=sensor_name,
        frame_id=frame_id,
    )
    color_prim_path, depth_prim_path = _attach_sensor_asset(
        mount_path=mount_path,
        target_path=target_path,
        asset_path=asset_path,
        mount_orientation=_as_vec(sensor.get("orientation"), [1.0, 0.0, 0.0, 0.0]),
        initialize=_as_bool(sensor.get("initialize"), False) and is_depth,
        depth_camera_subpath=sensor.get(
            "depth_camera_subpath", "RSD455/Camera_Pseudo_Depth"
        ),
        color_camera_subpath=sensor.get(
            "color_camera_subpath", "RSD455/Camera_OmniVision_OV9782_Color"
        ),
    )

    if not _as_bool(sensor.get("ros2"), True):
        return

    if _as_bool(sensor.get("rgb"), True):
        Ros2CameraGraph(
            og_path=f"{robot_prim_path}/ros2_graphs/{sensor_name}_rgb",
            frame_id=frame_id,
            node_namespace=namespace,
            sensor_prim=color_prim_path,
            camera_info_topic=rgb_camera_info_topic,
            rgb_pub=True,
            rgb_topic=rgb_topic,
            instance_pub=_as_bool(sensor.get("instance")),
            instance_topic=sensor.get("instance_topic", "instance_segmentation"),
            semantic_pub=_as_bool(sensor.get("semantic")),
            semantic_topic=sensor.get("semantic_topic", "semantic_segmentation"),
            bbox2d_tight_pub=_as_bool(sensor.get("bbox_2d_tight")),
            bbox2d_tight_topic=sensor.get("bbox_2d_tight_topic", "bbox_2d_tight"),
            bbox2d_loose_pub=_as_bool(sensor.get("bbox_2d_loose")),
            bbox2d_loose_topic=sensor.get("bbox_2d_loose_topic", "bbox_2d_loose"),
            bbox3d_pub=_as_bool(sensor.get("bbox_3d")),
            bbox3d_topic=sensor.get("bbox_3d_topic", "bbox_3d"),
        ).create_graph()

    if is_depth and _as_bool(sensor.get("depth"), True):
        Ros2CameraGraph(
            og_path=f"{robot_prim_path}/ros2_graphs/{sensor_name}_depth",
            frame_id=frame_id,
            node_namespace=namespace,
            sensor_prim=depth_prim_path,
            camera_info_topic=depth_camera_info_topic,
            depth_pub=True,
            depth_topic=depth_topic,
            depth_pcl_pub=_as_bool(sensor.get("depth_pcl")),
            depth_pcl_topic=sensor.get("depth_pcl_topic", "depth/points"),
        ).create_graph()


def _resolve_articulation_base_body(stage, robot_root_path: str):
    """Return the path of the articulation's base rigid body, or ``None``.

    Prefers the prim carrying ``PhysicsArticulationRootAPI`` (when it is also a
    rigid body); otherwise falls back to the first prim with
    ``PhysicsRigidBodyAPI``. Used to anchor collapsed sensor mounts (whose parent
    link was removed by ``merge_fixed_joints``) to a valid rigid body.
    """
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(robot_root_path)
    if not root.IsValid():
        return None
    art_root = None
    first_body = None
    for prim in Usd.PrimRange(root):
        is_body = prim.HasAPI(UsdPhysics.RigidBodyAPI)
        if art_root is None and is_body and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            art_root = prim.GetPath().pathString
        if first_body is None and is_body:
            first_body = prim.GetPath().pathString
    return art_root or first_body


def _first_rigid_body_path(stage, root_path: str):
    """Return the path of the first prim at/under ``root_path`` with RigidBodyAPI.

    Referenced camera assets (e.g. RealSense ``.../RSD455``) carry their own
    ``PhysicsRigidBodyAPI`` on a child prim, while the wrapper Xform we author at
    ``root_path`` is non-rigid. Anchoring a camera fixed joint to that rigid body
    (rather than the wrapper) keeps the camera a normal internal articulation
    link; anchoring to the non-rigid wrapper instead makes PhysX treat the joint
    as a world anchor and pins the whole robot as fixed-base.
    """
    from pxr import Usd, UsdPhysics

    root = stage.GetPrimAtPath(root_path)
    if not (root and root.IsValid()):
        return None
    for prim in Usd.PrimRange(root):
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            return prim.GetPath().pathString
    return None


def _camera_info_topic_for_stream(stream_topic: str) -> str:
    """Derive a camera-info topic from a stream topic path."""
    topic = (stream_topic or "").strip().rstrip("/")
    if not topic:
        return "camera_info"

    for suffix in ("/image_raw", "/image_rect_raw", "/image"):
        if topic.endswith(suffix):
            return topic[: -len(suffix)] + "/camera_info"

    if "/" in topic:
        return topic.rsplit("/", 1)[0] + "/camera_info"
    return "camera_info"


def _attach_sensor_asset(
    *,
    mount_path: str,
    target_path: str,
    asset_path: str,
    initialize: bool,
    depth_camera_subpath: str,
    color_camera_subpath: str,
    mount_orientation: list[float] | None = None,
) -> tuple[str, str]:
    import omni.kit.commands
    import omni.usd

    _compat.require_experimental_prims("camera sensor authoring")
    from isaacsim.sensors.camera import SingleViewDepthSensorAsset
    from isaacsim.core.experimental.prims import XformPrim
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    stage = omni.usd.get_context().get_stage()
    mount_prim = stage.GetPrimAtPath(mount_path)
    target_prim = stage.GetPrimAtPath(target_path)
    mount_exists = mount_prim.IsValid()
    target_exists = target_prim.IsValid()

    if mount_path != target_path and mount_exists and not target_exists:
        omni.kit.commands.execute("MovePrim", path_from=mount_path, path_to=target_path)
        target_exists = True
    elif mount_path != target_path and mount_exists and target_exists:
        print(
            f"[urdf2usd] mount '{mount_path}' and target '{target_path}' both exist; "
            "using existing target frame and leaving mount in place"
        )

    if not stage.GetPrimAtPath(target_path).IsValid():
        # No existing target frame to host the reference. Author a plain Xform
        # so the sensor USD can still be referenced.
        xform = UsdGeom.Xform.Define(stage, Sdf.Path(target_path))
        w, x, y, z = (mount_orientation or [1.0, 0.0, 0.0, 0.0])[:4]
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(
            Gf.Quatd(w, Gf.Vec3d(x, y, z))
        )
        print(
            f"[urdf2usd] WARNING: could not find mount/target frame for '{mount_path}'. "
            f"Authored fallback frame '{target_path}' with sensor orientation."
        )

    # Determine the joint's body0 (the robot link the camera fixes to). It must
    # be a valid rigid body; when merge_fixed_joints collapsed the mount (so its
    # parent path is the robot root / a scope, not a body), fall back to the
    # articulation base body.
    parent_link_path = mount_path.rsplit("/", 1)[0]
    parent_prim = stage.GetPrimAtPath(parent_link_path)
    if not (parent_prim.IsValid() and parent_prim.HasAPI(UsdPhysics.RigidBodyAPI)):
        robot_root_path = target_path.rsplit("/", 1)[0]
        base_body = _resolve_articulation_base_body(stage, robot_root_path)
        if base_body:
            print(
                f"[urdf2usd] sensor mount parent '{parent_link_path}' is not a "
                f"rigid body; anchoring sensor fixed joint to base body '{base_body}'"
            )
            parent_link_path = base_body
        else:
            parent_link_path = None

    # Reference the camera USD before deciding how to attach it so we can inspect
    # whether the referenced hierarchy carries a rigid body.
    target_xform = XformPrim(target_path)
    _, target_orient = target_xform.get_world_poses(indices=[0])

    depth_asset = SingleViewDepthSensorAsset(
        prim_path=target_path,
        asset_path=asset_path,
        orientation=target_orient[0],
    )

    camera_body_path = _first_rigid_body_path(stage, target_path)

    if parent_link_path and camera_body_path:
        joint_path = (
            f"{target_path.rsplit('/', 1)[0]}/joints/"
            f"{target_path.rsplit('/', 1)[-1]}_joint"
        )
        joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
        joint.CreateBody0Rel().SetTargets([Sdf.Path(parent_link_path)])
        joint.CreateBody1Rel().SetTargets([Sdf.Path(camera_body_path)])

    elif parent_link_path and not camera_body_path:
        # Non-rigid referenced assets should stay kinematic under the mount
        # hierarchy instead of creating an invalid fixed joint to a non-rigid
        # wrapper. If we moved the mount frame earlier, restore its original
        # place so transform inheritance remains intact.
        if target_path != mount_path and stage.GetPrimAtPath(target_path).IsValid():
            try:
                if not stage.GetPrimAtPath(mount_path).IsValid():
                    omni.kit.commands.execute(
                        "MovePrim", path_from=target_path, path_to=mount_path
                    )
                    target_path = mount_path
            except Exception as exc:
                print(
                    f"[urdf2usd] WARNING: failed to restore non-rigid sensor frame "
                    f"from '{target_path}' to '{mount_path}': {exc}"
                )

        print(
            f"[urdf2usd] INFO: sensor asset under '{target_path}' exposes no "
            "RigidBodyAPI; keeping it kinematic under the mount hierarchy."
        )

    else:
        print(
            f"[urdf2usd] WARNING: no rigid body found to anchor sensor "
            f"'{target_path}'; skipping fixed joint (camera will not follow the robot)"
        )

    color_prim_path = f"{target_path}/{color_camera_subpath}"
    depth_prim_path = f"{target_path}/{depth_camera_subpath}"

    if initialize:
        depth_asset.initialize()
        try:
            child = depth_asset.get_child_depth_sensor(depth_prim_path)
            child.attach_annotator("DepthSensorDistance")
        except Exception as exc:
            # Annotators are best-effort; never fail the import for them.
            print(
                f"[urdf2usd] WARNING: failed to attach DepthSensorDistance annotator at "
                f"'{depth_prim_path}': {exc}"
            )

    return color_prim_path, depth_prim_path


def _add_rgb_camera(
    robot_prim_path: str,
    sensor: dict[str, str],
    namespace: str,
    urdf_xml: str | None = None,
) -> None:
    _add_camera(robot_prim_path, sensor, namespace, is_depth=False, urdf_xml=urdf_xml)


def _add_depth_camera(
    robot_prim_path: str,
    sensor: dict[str, str],
    namespace: str,
    urdf_xml: str | None = None,
) -> None:
    _add_camera(robot_prim_path, sensor, namespace, is_depth=True, urdf_xml=urdf_xml)


_SENSOR_DISPATCH = {
    "imu": _add_imu,
    "rtx_lidar": _add_rtx_lidar,
    "rgb_camera": _add_rgb_camera,
    "depth_camera": _add_depth_camera,
}
