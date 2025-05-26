from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import omni
import omni.timeline
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.sensors.camera import Camera
from isaacsim.sensors.physics import IMUSensor
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.wheeled_robots.robots import WheeledRobot
from pxr import UsdShade, Sdf, Gf, UsdGeom
from omni.isaac.core.utils.stage import add_reference_to_stage
from scipy.spatial.transform import Rotation


from sim_utils import (
    publish_scan,
    publish_camera_info,
    publish_rgb,
    publish_depth,
    diff_drive_wheel_velocities,
)

enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

ROBOT_USD_PATH = "pipe_issac_sim/models/stormRobot.usd"
PIPE_USD_PATH = "pipe_issac_sim/models/pipe_flat.usd"
PIPE_TEXTURE_PATH = "pipe_issac_sim/models/testPipeTexture.png"


class StormRunnerSim(Node):
    def __init__(self):
        super().__init__("storm_runner_sim")

        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=1.0)
        self.ros_world.scene.add_default_ground_plane()

        stage = self.ros_world.stage

        # Add pipe
        pipe_path = "/World/pipe"
        add_reference_to_stage(usd_path=PIPE_USD_PATH, prim_path=pipe_path)

        shader_path = "/World/pipe/pipe/pipe/Default/Looks/PipeTexture/MDLShader"
        shader_prim = stage.GetPrimAtPath(shader_path)
        shader = UsdShade.Shader(shader_prim)

        diffuse_input = shader.CreateInput("diffuse_texture", Sdf.ValueTypeNames.Asset)
        diffuse_input.Set(Sdf.AssetPath(PIPE_TEXTURE_PATH))

        texture_scale = shader.CreateInput("texture_scale", Sdf.ValueTypeNames.Float2)
        texture_scale.Set((0.2, 0.2))

        pipe_prim = stage.GetPrimAtPath(pipe_path)
        pipe_xform = UsdGeom.Xformable(pipe_prim)
        pipe_xform.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, 0.5))

        simulation_app.update()

        rot = Rotation.from_euler("XYZ", [90, -90, 0], degrees=True)
        scipy_quat = rot.as_quat()
        isaac_quat = [
            scipy_quat[3],
            scipy_quat[0],
            scipy_quat[1],
            scipy_quat[2],
        ]  # [x, y, z, w] to [w, x, y, z]

        # Add Storm Runner
        self.storm_runner = self.ros_world.scene.add(
            WheeledRobot(
                prim_path="/World/Storm_Sight_Runner",
                name="Storm_Sight_Runner",
                wheel_dof_names=[
                    "front_left_joint",
                    "front_right_joint",
                    "back_left_joint",
                    "back_right_joint",
                ],
                create_robot=True,
                usd_path=ROBOT_USD_PATH,
                position=np.array([0.5, 0.0, 0.5]),
                orientation=isaac_quat,
            )
        )

        # Add sensors
        storm_base_path = "/World/Storm_Sight_Runner/storm_base"

        back_camera = Camera(
            prim_path=storm_base_path
            + "/storm_sensors/intel_realsense_camera/back_camera",
            resolution=(256, 256),
        )

        front_camera = Camera(
            prim_path=storm_base_path
            + "/storm_sensors/sensing_module_assembly/CameraandLightAssembly/front_camera",
            resolution=(256, 256),
        )

        self.imu = IMUSensor(
            prim_path=storm_base_path + "/chassis/Imu",
            name="imu",
            frequency=60,
            translation=np.array([0, 0, 0]),
            orientation=np.array([1, 0, 0, 0]),
            linear_acceleration_filter_size=10,
            angular_velocity_filter_size=10,
            orientation_filter_size=10,
        )

        _, lidar = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=storm_base_path
            + "/storm_sensors/sensing_module_assembly/LaserMountAssembly/RingLaser1/lidar",
            parent=None,
            config="Example_Rotary",
            translation=(0, 0, 1.0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        )

        # Initialize communication
        back_camera.initialize()
        front_camera.initialize()
        simulation_app.update()
        back_camera.initialize()
        front_camera.initialize()

        camera_freq = 30

        back_camera_namespace = "/back_camera"
        publish_camera_info(back_camera, back_camera_namespace, camera_freq)
        publish_rgb(back_camera, back_camera_namespace, camera_freq)
        publish_depth(back_camera, back_camera_namespace, camera_freq)

        front_camera_namespace = "/front_camera"
        publish_camera_info(front_camera, front_camera_namespace, camera_freq)
        publish_rgb(front_camera, front_camera_namespace, camera_freq)
        publish_depth(front_camera, front_camera_namespace, camera_freq)

        publish_scan(lidar)

        self.wheel_vel_sub = self.create_subscription(
            Float64MultiArray, "wheel_velocities", self.wheel_velocity_callback, 10
        )
        self.twist_sub = self.create_subscription(
            Twist, "cmd_vel", self.cmd_vel_callback, 10
        )

        simulation_app.update()
        self.ros_world.reset()

    def cmd_vel_callback(self, msg):
        if self.ros_world.is_playing():
            linear_x = msg.linear.x  # forward/backward velocity
            angular_z = msg.angular.z  # rotational velocity

            wheel_velocities = diff_drive_wheel_velocities(
                linear_x, angular_z, self.wheel_base_width, self.wheel_radius
            )

            action = ArticulationAction(joint_velocities=wheel_velocities)
            self.storm_runner.apply_wheel_actions(action)

    def wheel_velocity_callback(self, msg):
        if self.ros_world.is_playing():
            wheel_velocities = np.array(msg.data)
            action = ArticulationAction(joint_velocities=wheel_velocities)
            self.storm_runner.apply_wheel_actions(action)

    def run_simulation(self):
        self.timeline.play()
        reset_needed = False

        while simulation_app.is_running():
            self.ros_world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)

            if self.ros_world.is_stopped() and not reset_needed:
                reset_needed = True

            if self.ros_world.is_playing():
                if reset_needed:
                    self.ros_world.reset()
                    reset_needed = False

        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    storm_runner_sim_node = StormRunnerSim()
    storm_runner_sim_node.run_simulation()
