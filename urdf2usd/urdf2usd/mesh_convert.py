# Software License Agreement (proprietary)
#
# \authors   Ahmed Hamouda <ahmed.hamouda@rockwellautomation.com>
#
# Copyright (c) 2026 Rockwell Automation Technologies, Inc. All Rights Reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, is not permitted without the express permission of
# Rockwell Automation Technologies, Inc.
"""Convert glTF/GLB visual meshes to Wavefront OBJ for the Isaac Sim 6.x importer.

Isaac Sim 6.x delegates URDF mesh loading to the ``urdf-usd-converter`` package,
whose loaders cover only OBJ (tinyobjloader), Collada/DAE (pycollada) and STL
(numpy-stl) -- there is **no glTF/GLB loader**. A URDF whose ``<visual>`` meshes
are ``.glb`` therefore imports with collision geometry only (the visuals are
silently dropped). Isaac Sim 5.x used assimp, which reads GLB, so this module is
only ever invoked on the 6.x path (native GLB is kept on 5.x for best fidelity).

:func:`convert_unsupported_meshes` rewrites every ``.glb`` / ``.gltf`` mesh
reference in a URDF string to a freshly-exported ``.obj`` (with its ``.mtl`` and
textures) placed in a cache directory, and returns the rewritten URDF.

``trimesh`` is imported lazily so this module stays importable without it (the
parser unit tests never touch the conversion path).
"""

from __future__ import annotations

import os
import re
from typing import Iterable

__all__ = ["convert_unsupported_meshes", "UNSUPPORTED_MESH_EXTS"]

# Extensions the 6.x urdf-usd-converter cannot load and that this module maps to
# OBJ. Lower-case, leading dot.
UNSUPPORTED_MESH_EXTS = (".glb", ".gltf")

# Matches filename="..."/filename='...' inside a <mesh> reference. The URDF is
# machine-expanded (xacro) so a targeted regex avoids reserialising the whole
# document (which would drop comments / reorder attributes).
_FILENAME_RE = re.compile(
    r"""filename\s*=\s*(?P<q>["'])(?P<path>[^"']+)(?P=q)""",
    re.IGNORECASE,
)


def _needs_conversion(path: str) -> bool:
    return os.path.splitext(path)[1].lower() in UNSUPPORTED_MESH_EXTS


def _strip_scheme(path: str) -> str:
    """Drop a leading ``file://`` scheme, leaving an absolute filesystem path."""
    if path.startswith("file://"):
        return path[len("file://") :]
    return path


def convert_unsupported_meshes(urdf_xml: str, cache_dir: str) -> str:
    """Rewrite unsupported (GLB/glTF) mesh references to exported OBJ files.

    ``urdf_xml`` must already have its ``package://`` references resolved to
    absolute filesystem paths (``urdf_io.replace_package_names`` does this).
    Each unique GLB/glTF source is converted once into ``cache_dir`` and every
    matching ``filename="..."`` is rewritten to the absolute OBJ path. Meshes in
    formats the importer already understands (OBJ/DAE/STL) are left untouched.

    Returns the rewritten URDF. If no unsupported meshes are present the input is
    returned unchanged (and ``trimesh`` is never imported).
    """
    sources = _collect_unsupported_sources(urdf_xml)
    if not sources:
        return urdf_xml

    os.makedirs(cache_dir, exist_ok=True)
    mapping = _convert_sources(sources, cache_dir)

    def _replace(match: "re.Match[str]") -> str:
        raw = match.group("path")
        obj_path = mapping.get(_strip_scheme(raw))
        if obj_path is None:
            return match.group(0)
        quote = match.group("q")
        return f"filename={quote}{obj_path}{quote}"

    return _FILENAME_RE.sub(_replace, urdf_xml)


def _collect_unsupported_sources(urdf_xml: str) -> list[str]:
    """Return the unique, scheme-stripped absolute paths needing conversion."""
    seen: dict[str, None] = {}
    for match in _FILENAME_RE.finditer(urdf_xml):
        path = _strip_scheme(match.group("path"))
        if _needs_conversion(path):
            seen.setdefault(path, None)
    return list(seen)


def _convert_sources(sources: Iterable[str], cache_dir: str) -> dict[str, str]:
    """Convert each source mesh to OBJ, returning ``{source_path: obj_path}``."""
    try:
        import trimesh  # noqa: F401  (import surfaced for the clear error below)
    except Exception as exc:  # pragma: no cover - environment-specific
        raise ImportError(
            "urdf2usd needs the 'trimesh' package to convert GLB/glTF visual "
            "meshes for the Isaac Sim 6.x importer (its mesh loader supports "
            "OBJ/DAE/STL only). Install it into Isaac Sim's Python, e.g.:\n"
            "  ${ISAAC_SIM_PATH}/python.sh -m pip install 'trimesh[easy]'\n"
            f"(original import error: {exc})"
        ) from exc

    mapping: dict[str, str] = {}
    for src in sources:
        mapping[src] = _convert_one(src, cache_dir)
    return mapping


def _convert_one(src_path: str, cache_dir: str) -> str:
    """Convert a single GLB/glTF file to OBJ in ``cache_dir``; return OBJ path."""
    import trimesh

    if not os.path.isfile(src_path):
        raise FileNotFoundError(
            f"urdf2usd: visual mesh '{src_path}' referenced by the URDF does not "
            "exist (expected an absolute path after package:// resolution)."
        )

    stem = os.path.splitext(os.path.basename(src_path))[0]
    # Namespace the OBJ by a hash of the source path so two meshes with the same
    # basename from different directories don't collide in the cache.
    tag = f"{abs(hash(src_path)) & 0xFFFFFF:06x}"
    out_obj = os.path.join(cache_dir, f"{stem}_{tag}.obj")

    if os.path.isfile(out_obj):
        return out_obj

    # force='mesh' would merge a multi-part scene into one mesh but discard
    # per-part materials; load as a Scene so exported OBJ keeps its .mtl and
    # baked node transforms (matching how assimp presented the GLB on 5.x).
    loaded = trimesh.load(src_path, process=False)
    loaded.export(out_obj, file_type="obj", include_texture=True)
    return out_obj
