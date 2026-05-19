"""Headless A300 URDF -> USD conversion for Isaac Sim 5.x.

Run inside the isaac-sim container:
    /isaac-sim/python.sh /workspace/scripts/import_a300.py
"""
import os

from isaacsim import SimulationApp

kit = SimulationApp({"headless": True})

import omni.kit.commands
import omni.usd
from pxr import Usd

URDF_PATH = "/workspace/urdf/a300.urdf"
USD_PATH = "/workspace/usd/a300.usd"

os.makedirs(os.path.dirname(USD_PATH), exist_ok=True)
# The URDF importer writes intermediate *.tmp.usd files into the current
# working directory. Make sure that's somewhere we can write.
os.chdir(os.path.dirname(USD_PATH))

status, import_config = omni.kit.commands.execute("URDFCreateImportConfig")
import_config.merge_fixed_joints = True
import_config.convex_decomp = False
import_config.import_inertia_tensor = True
import_config.fix_base = False
import_config.self_collision = False
import_config.distance_scale = 1.0
import_config.density = 0.0
import_config.make_default_prim = True
import_config.create_physics_scene = True

status, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=URDF_PATH,
    import_config=import_config,
    get_articulation_root=True,
)
print(f"Imported A300 at prim: {prim_path}")

stage = omni.usd.get_context().get_stage()
stage.GetRootLayer().Export(USD_PATH)
print(f"Saved USD: {USD_PATH}")

kit.close()
