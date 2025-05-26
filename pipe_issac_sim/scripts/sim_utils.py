import omni
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata._syntheticdata as sd
from isaacsim.sensors.camera import Camera
import numpy as np


def publish_scan(lidar):
    hydra_texture = rep.create.render_product(lidar.GetPath(), [1, 1], name="Isaac")
    writer = rep.writers.get("RtxLidar" + "ROS2PublishLaserScan")
    writer.initialize(topicName="scan", frameId="base_scan")
    writer.attach([hydra_texture])


def publish_camera_info(camera: Camera, namespace, freq):
    from isaacsim.ros2.bridge import read_camera_info

    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_camera_info"
    queue_size = 1
    node_namespace = namespace
    frame_id = camera.prim_path.split("/")[-1]

    writer = rep.writers.get("ROS2PublishCameraInfo")
    camera_info = read_camera_info(render_product_path=render_product)
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name,
        width=camera_info["width"],
        height=camera_info["height"],
        projectionType=camera_info["projectionType"],
        k=camera_info["k"].reshape([1, 9]),
        r=camera_info["r"].reshape([1, 9]),
        p=camera_info["p"].reshape([1, 12]),
        physicalDistortionModel=camera_info["physicalDistortionModel"],
        physicalDistortionCoefficients=camera_info["physicalDistortionCoefficients"],
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        "PostProcessDispatch" + "IsaacSimulationGate", render_product
    )

    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_rgb(camera: Camera, namespace, freq):
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_rgb"
    queue_size = 1
    node_namespace = namespace
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.Rgb.name
    )
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name,
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def publish_depth(camera: Camera, namespace, freq):
    render_product = camera._render_product_path
    step_size = int(60 / freq)
    topic_name = camera.name + "_depth"
    queue_size = 1
    node_namespace = namespace
    frame_id = camera.prim_path.split("/")[-1]

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(
        sd.SensorType.DistanceToImagePlane.name
    )
    writer = rep.writers.get(rv + "ROS2PublishImage")
    writer.initialize(
        frameId=frame_id,
        nodeNamespace=node_namespace,
        queueSize=queue_size,
        topicName=topic_name,
    )
    writer.attach([render_product])

    gate_path = omni.syntheticdata.SyntheticData._get_node_path(
        rv + "IsaacSimulationGate", render_product
    )
    og.Controller.attribute(gate_path + ".inputs:step").set(step_size)


def diff_drive_wheel_velocities(linear_x, angular_z, wheel_base_width, wheel_radius):
    left_velocity = (linear_x - angular_z * wheel_base_width / 2) / wheel_radius
    right_velocity = -(linear_x + angular_z * wheel_base_width / 2) / wheel_radius
    return np.array([left_velocity, right_velocity, left_velocity, right_velocity])
