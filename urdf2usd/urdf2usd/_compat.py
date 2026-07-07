# Software License Agreement (proprietary)
#
# \authors   Ahmed Hamouda <ahmed.hamouda@rockwellautomation.com>
#
# Copyright (c) 2026 Rockwell Automation Technologies, Inc. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission of
# Rockwell Automation Technologies, Inc.
"""Isaac Sim version-compatibility helpers for :mod:`urdf2usd`.

The package targets both Isaac Sim 5.x and 6.x from a single codebase. Rather
than parse version strings (which would break for arbitrary 5.x / 6.x point
releases), every divergent API is selected by *capability detection*: we try to
import the class/module a given backend provides and fall back to the other.

All detection is lazy and wrapped in ``try/except`` so this module (and every
module that imports it) stays importable from a plain Python interpreter with no
Isaac Sim runtime present -- the parser unit tests depend on that.
"""

from __future__ import annotations

import functools
import importlib
from typing import Any

__all__ = [
    "module_available",
    "attr_available",
    "urdf_importer_backend",
    "imu_backend",
    "experimental_prims_available",
    "experimental_materials_available",
    "require_experimental_prims",
    "require_experimental_materials",
    "get_assets_root_path",
    "describe_backend",
]


@functools.lru_cache(maxsize=None)
def module_available(module: str) -> bool:
    """Return ``True`` if ``module`` can be imported in this interpreter."""
    try:
        importlib.import_module(module)
        return True
    except Exception:
        return False


@functools.lru_cache(maxsize=None)
def attr_available(module: str, attr: str) -> bool:
    """Return ``True`` if ``module`` imports and exposes ``attr``."""
    try:
        mod = importlib.import_module(module)
    except Exception:
        return False
    return hasattr(mod, attr)


@functools.lru_cache(maxsize=None)
def urdf_importer_backend() -> str:
    """Select the URDF-import backend.

    ``"v6"`` -> Isaac Sim 6.x class API
    (:class:`isaacsim.asset.importer.urdf.URDFImporter`).
    ``"v5"`` -> Isaac Sim 5.x command API
    (``URDFCreateImportConfig`` / ``URDFParse*`` / ``URDFImportRobot``).

    Detection is by presence of the ``URDFImporter`` class, which only exists in
    6.x. The extension name (``isaacsim.asset.importer.urdf``) is shared by both.
    """
    if attr_available("isaacsim.asset.importer.urdf", "URDFImporter"):
        return "v6"
    return "v5"


@functools.lru_cache(maxsize=None)
def imu_backend() -> str:
    """Select the IMU authoring backend.

    ``"experimental"`` -> 6.x ``isaacsim.sensors.experimental.physics.IMU``.
    ``"legacy"``       -> 5.x ``isaacsim.sensors.physics.IMUSensor``.
    """
    if attr_available("isaacsim.sensors.experimental.physics", "IMU"):
        return "experimental"
    return "legacy"


@functools.lru_cache(maxsize=None)
def experimental_prims_available() -> bool:
    """Return ``True`` when ``isaacsim.core.experimental.prims`` is importable.

    The batched experimental prim wrappers (``Articulation``, ``GeomPrim``,
    ``XformPrim``) exist in 6.x and recent 5.x; older 5.x only ships the stable
    ``isaacsim.core.prims`` API.
    """
    return module_available("isaacsim.core.experimental.prims")


@functools.lru_cache(maxsize=None)
def experimental_materials_available() -> bool:
    """Return ``True`` when ``isaacsim.core.experimental.materials`` imports."""
    return module_available("isaacsim.core.experimental.materials")


def require_experimental_prims(feature: str) -> None:
    """Raise an actionable error if ``core.experimental.prims`` is unavailable."""
    if not experimental_prims_available():
        raise RuntimeError(
            f"{feature} needs 'isaacsim.core.experimental.prims', which this "
            "Isaac Sim build does not provide (introduced in Isaac Sim 5.0). "
            "Upgrade to Isaac Sim >= 5.0, or omit the feature from <isaac_inputs>."
        )


def require_experimental_materials(feature: str) -> None:
    """Raise an actionable error if ``core.experimental.materials`` is missing."""
    if not experimental_materials_available():
        raise RuntimeError(
            f"{feature} needs 'isaacsim.core.experimental.materials', which this "
            "Isaac Sim build does not provide (introduced in Isaac Sim 5.0). "
            "Upgrade to Isaac Sim >= 5.0, or omit the feature from <isaac_inputs>."
        )


def get_assets_root_path() -> str | None:
    """Return the Isaac assets root, tolerating module moves across versions.

    5.1+ / 6.x expose it at ``isaacsim.storage.native``; very old 5.x builds
    kept it at ``omni.isaac.nucleus``. Returns ``None`` when neither resolves.
    """
    for module in ("isaacsim.storage.native", "omni.isaac.nucleus"):
        try:
            mod = importlib.import_module(module)
        except Exception:
            continue
        fn = getattr(mod, "get_assets_root_path", None)
        if fn is not None:
            return fn()
    return None


def describe_backend() -> dict[str, Any]:
    """Return a small dict summarising the detected backend, for logging."""
    return {
        "urdf_importer": urdf_importer_backend(),
        "imu": imu_backend(),
        "experimental_prims": experimental_prims_available(),
        "experimental_materials": experimental_materials_available(),
    }
