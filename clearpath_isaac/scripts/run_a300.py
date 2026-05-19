"""Run A300 in Isaac Sim with ROS 2 /cmd_vel control.

Usage (inside isaac-sim container):
    /isaac-sim/python.sh /workspace/scripts/run_a300.py [--headless]
"""
import sys

HEADLESS = "--headless" in sys.argv
GUI = "--gui" in sys.argv or not HEADLESS

from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": not GUI})

import numpy as np
import omni
from isaacsim.core.api import World
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.robot.wheeled_robots.controllers.differential_controller import (
    DifferentialController,
)
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdLux, UsdPhysics

enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

A300_USD = "/workspace/usd/a300.usd"
ROBOT_PRIM = "/World/a300"

# Wheel geometry (matches a300.urdf.xacro)
WHEEL_RADIUS = 0.165
WHEEL_TRACK = 0.562

JOINT_ORDER = [
    "front_left_wheel_joint",
    "rear_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_right_wheel_joint",
]


class A300Sim(Node):
    def __init__(self):
        super().__init__("a300_isaac_sim")
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Lighting
        stage = omni.usd.get_context().get_stage()
        dl = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/DistantLight"))
        dl.CreateIntensityAttr(1500)

        # Reference in the A300 USD
        add_reference_to_stage(usd_path=A300_USD, prim_path=ROBOT_PRIM)
        robot_prim = stage.GetPrimAtPath(ROBOT_PRIM)
        robot_xform = UsdGeom.Xformable(robot_prim)
        # Reset xform ops to a single translate so referenced ops don't clash
        robot_xform.ClearXformOpOrder()
        robot_xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.3))

        # Configure velocity drives on the four wheel joints
        for jn in JOINT_ORDER:
            jp = f"{ROBOT_PRIM}/joints/{jn}"
            prim = stage.GetPrimAtPath(jp)
            if not prim.IsValid():
                self.get_logger().warn(f"missing joint {jp}")
                continue
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateTypeAttr("force")
            drive.CreateMaxForceAttr(1e6)
            drive.CreateTargetVelocityAttr(0.0)
            drive.CreateDampingAttr(5e3)
            drive.CreateStiffnessAttr(0.0)

        # Wheel-ground material with PhysX patch friction. Cylinder colliders
        # on a plane produce line contact; without improvePatchFriction PhysX
        # only solves a single contact point per wheel and skid-steer rotation
        # silently slips in place. `frictionCombineMode=max` keeps the higher
        # of wheel/ground friction so the ground default (0.5) can't dilute us.
        from pxr import UsdShade
        mat_path = "/World/PhysicsMaterials/WheelMaterial"
        mat = UsdShade.Material.Define(stage, mat_path)
        pmat = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        pmat.CreateStaticFrictionAttr(1.2)
        pmat.CreateDynamicFrictionAttr(1.0)
        pmat.CreateRestitutionAttr(0.0)
        PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim())
        mp = mat.GetPrim()
        mp.CreateAttribute(
            "physxMaterial:improvePatchFriction", Sdf.ValueTypeNames.Bool
        ).Set(True)
        mp.CreateAttribute(
            "physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token
        ).Set("max")
        for side in ("front_left", "front_right", "rear_left", "rear_right"):
            col_path = f"{ROBOT_PRIM}/{side}_wheel_link/collisions"
            col_prim = stage.GetPrimAtPath(col_path)
            if not col_prim.IsValid():
                self.get_logger().warn(f"missing wheel collision: {col_path}")
                continue
            binding = UsdShade.MaterialBindingAPI.Apply(col_prim)
            binding.Bind(mat, materialPurpose="physics")
            # Tighten contact offset so the cylinder edge doesn't float on its
            # 2 cm default skin (which acts like a low-friction air cushion).
            for sub in col_prim.GetChildren():
                pxc = PhysxSchema.PhysxCollisionAPI.Apply(sub)
                pxc.CreateContactOffsetAttr(0.005)
                pxc.CreateRestOffsetAttr(0.0)

        # Disable base_link colliders (chassis + suspension + motor meshes from
        # merged links). Only the four wheel cylinders should contact ground;
        # leaving these enabled drags the body and prevents skid-steer rotation.
        base_coll_root = stage.GetPrimAtPath(f"{ROBOT_PRIM}/base_link/collisions")
        if base_coll_root.IsValid():
            for child in base_coll_root.GetChildren():
                col_api = UsdPhysics.CollisionAPI.Get(child)
                if col_api:
                    col_api.GetCollisionEnabledAttr().Set(False)
            self.get_logger().info("disabled base_link sub-colliders")

        self.cmd = np.zeros(2)  # linear x, angular z
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, 10)
        self._odom_pub = self.create_publisher(Odometry, "odom", 10)
        self._tf_pub = self.create_publisher(TFMessage, "/tf", 10)

        # Official Isaac differential-drive controller. For our 4WD skid-steer
        # we just duplicate left/right onto both wheels on each side.
        self._diff = DifferentialController(
            name="a300_diff",
            wheel_radius=WHEEL_RADIUS,
            wheel_base=WHEEL_TRACK,
        )

        self.world.reset()
        self._articulation = None

    def _on_cmd_vel(self, msg: Twist):
        self.cmd[0] = msg.linear.x
        self.cmd[1] = msg.angular.z

    def _wheel_targets(self):
        action = self._diff.forward(self.cmd)
        v_left, v_right = action.joint_velocities[0], action.joint_velocities[1]
        # Order matches JOINT_ORDER: FL, RL, FR, RR
        return np.array([v_left, v_left, v_right, v_right], dtype=np.float32)

    def _publish_odom_tf(self, art):
        try:
            pos, ori = art.get_world_pose()  # ori = (w, x, y, z)
            lin_v, ang_v = art.get_linear_velocity(), art.get_angular_velocity()
        except Exception:
            return
        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.w = float(ori[0])
        odom.pose.pose.orientation.x = float(ori[1])
        odom.pose.pose.orientation.y = float(ori[2])
        odom.pose.pose.orientation.z = float(ori[3])
        odom.twist.twist.linear.x = float(lin_v[0])
        odom.twist.twist.linear.y = float(lin_v[1])
        odom.twist.twist.linear.z = float(lin_v[2])
        odom.twist.twist.angular.x = float(ang_v[0])
        odom.twist.twist.angular.y = float(ang_v[1])
        odom.twist.twist.angular.z = float(ang_v[2])
        self._odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self._tf_pub.publish(TFMessage(transforms=[t]))

    def _ensure_articulation(self):
        if self._articulation is not None:
            return self._articulation
        from isaacsim.core.prims import SingleArticulation
        try:
            art = SingleArticulation(prim_path=ROBOT_PRIM, name="a300")
            art.initialize()
            self._articulation = art
            # Map our joint names to articulation indices
            names = list(art.dof_names)
            self._idx = [names.index(n) for n in JOINT_ORDER]
            self.get_logger().info(f"DOFs: {names}")
        except Exception as e:
            self.get_logger().warn(f"articulation not ready: {e}")
        return self._articulation

    def run(self):
        from isaacsim.core.utils.types import ArticulationAction
        self.timeline.play()
        reset_needed = False
        step = 0
        while simulation_app.is_running():
            self.world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.world.is_stopped() and not reset_needed:
                reset_needed = True
            if self.world.is_playing():
                if reset_needed:
                    self.world.reset()
                    reset_needed = False
                    self._articulation = None
                art = self._ensure_articulation()
                if art is not None:
                    tgt = self._wheel_targets()
                    full = np.zeros(len(art.dof_names), dtype=np.float32)
                    for i, v in zip(self._idx, tgt):
                        full[i] = v
                    art.apply_action(ArticulationAction(joint_velocities=full))
                    self._publish_odom_tf(art)
                    step += 1
                    if step % 60 == 0:
                        try:
                            pos, _ = art.get_world_pose()
                            jv = art.get_joint_velocities()
                            self.get_logger().info(
                                f"cmd={self.cmd.tolist()} tgt={tgt.tolist()} "
                                f"jv={[round(float(x),2) for x in jv]} pos={pos.tolist()}"
                            )
                        except Exception as e:
                            self.get_logger().warn(f"diag: {e}")
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    A300Sim().run()
