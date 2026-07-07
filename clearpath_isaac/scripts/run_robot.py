"""Spawn a robot USD in Isaac Sim and loop the simulation.

Run inside the clearpath-isaac container with Isaac's Python:

    /isaac-sim/python.sh .../scripts/run_robot.py \
      --robot-path /abs/path/to/robot.usd \
      [--robot-prim /World/robot] [--spawn-z 0.15] [--headless true]
"""

import argparse
import os


def _str_to_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _validate_robot_path(robot_path: str) -> None:
    if not os.path.exists(robot_path):
        raise SystemExit(f"[run_robot] USD not found: {robot_path}")
    if not os.path.isfile(robot_path):
        raise SystemExit(f"[run_robot] --robot-path is not a file: {robot_path}")
    if not robot_path.endswith((".usd", ".usda", ".usdc")):
        raise SystemExit(f"[run_robot] --robot-path must point to a USD file: {robot_path}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-path",
        required=True,
        help="Path to the robot USD file to spawn (absolute or relative).",
    )
    parser.add_argument(
        "--robot-prim",
        default="/World/robot",
        help="Prim path where the robot USD will be referenced.",
    )
    parser.add_argument(
        "--spawn-z",
        type=float,
        default=0.15,
        help="Initial robot height in meters.",
    )
    parser.add_argument(
        "--headless",
        default="false",
        help="Run without GUI (true/false).",
    )
    return parser.parse_args()


def baked_physics_hz(usd_path, default_hz=60.0):
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        return float(default_hz)

    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            attr = prim.GetAttribute("physxScene:timeStepsPerSecond")
            if attr and attr.HasAuthoredValue():
                return float(attr.Get())
    return float(default_hz)


def main():
    args = parse_args()
    headless = _str_to_bool(args.headless)
    robot_path = os.path.abspath(args.robot_path)
    _validate_robot_path(robot_path)

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": headless})

    import omni.timeline
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.utils.extensions import enable_extension
    from isaacsim.core.utils.stage import add_reference_to_stage
    from pxr import Gf, Sdf, UsdGeom, UsdLux

    enable_extension("isaacsim.ros2.bridge")
    enable_extension("omni.physx.bundle")

    physics_hz = baked_physics_hz(robot_path)
    world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / physics_hz,
        rendering_dt=1.0 / 60.0,
    )
    world.scene.add_default_ground_plane()

    stage = omni.usd.get_context().get_stage()
    UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight")).CreateIntensityAttr(1500)

    add_reference_to_stage(usd_path=robot_path, prim_path=args.robot_prim)
    robot = UsdGeom.Xformable(stage.GetPrimAtPath(args.robot_prim))
    robot.ClearXformOpOrder()
    robot.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, float(args.spawn_z)))

    world.reset()

    print(
        "\n[run_robot] ready"
        f"\n  robot_path: {robot_path}"
        f"\n  robot_prim: {args.robot_prim}"
        f"\n  physics_hz: {physics_hz:.2f}"
        "\n"
    )

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    try:
        while simulation_app.is_running():
            world.step(render=True)
    finally:
        timeline.stop()
        simulation_app.close()


if __name__ == "__main__":
    main()
