import sys
import os

sys.path.append(os.path.abspath('/home/ser_v/tfg/docker/workspace/src/optitrack_visca/optitrack_visca'))
from Viscaoverip import Camera

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose, PointStamped, TransformStamped

from mocap4r2_msgs.msg import RigidBodies, RigidBody

from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster, Buffer, TransformListener
import tf_transformations
from tf2_ros import TransformException

from scipy.spatial.transform import Rotation

pan_step1 = 179
tilt_step1 = 171
pan_step2 = 178
tilt_step2 = 171


class CombinedNode(Node):
    def __init__(self):
        super().__init__('combined_node')

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_broadcaster = TransformBroadcaster(self)

        # Listener de transformaciones
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

        # Parámetros de configuración
        self.declare_parameter('camera1_ip', '192.168.1.89999999999')
        self.camera1_ip = self.get_parameter('camera1_ip').get_parameter_value().string_value
        self.declare_parameter('camera1_port', 52381)
        self.camera1_port = self.get_parameter('camera1_port').get_parameter_value().integer_value
        self.declare_parameter('camera2_ip', '192.168.1.90')
        self.camera2_ip = self.get_parameter('camera2_ip').get_parameter_value().string_value
        self.declare_parameter('camera2_port', 52381)
        self.camera2_port = self.get_parameter('camera2_port').get_parameter_value().integer_value
        self.declare_parameter('optitrack_x', 0.0)
        self.declare_parameter('optitrack_y', 0.0)
        self.declare_parameter('optitrack_z', 0.0)

        # Publicar transformaciones estáticas
        self.publish_static_transforms()

        self.camera1 = Camera("cam1", self.camera1_ip, self.camera1_port)
        self.camera1.pantilt_home()
        self.camera1.zoom_to(0)

        self.camera2 = Camera("cam2", self.camera2_ip, self.camera2_port)
        self.camera2.pantilt_home()
        self.camera2.zoom_to(0)

        # Suscribirse a los datos de cuerpos rígidos
        self.timer_transform = self.create_timer(0.1, self.tf_from_to)

    def publish_static_transforms(self):
        static_transforms = []

        t0 = TransformStamped()
        t0.header.stamp = self.get_clock().now().to_msg()
        t0.header.frame_id = 'earth'
        t0.child_frame_id = 'grid_0'
        t0.transform.translation.x = -self.get_parameter('optitrack_x').get_parameter_value().double_value
        t0.transform.translation.y = -self.get_parameter('optitrack_y').get_parameter_value().double_value
        t0.transform.translation.z = self.get_parameter('optitrack_z').get_parameter_value().double_value
        t0.transform.rotation.x = 0.0
        t0.transform.rotation.y = 0.0
        t0.transform.rotation.z = 0.0
        t0.transform.rotation.w = 1.0
        static_transforms.append(t0)
        
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'grid_0'
        t1.child_frame_id = 'camera1/map'
        t1.transform.translation.x = -3.65 + self.get_parameter('optitrack_x').get_parameter_value().double_value
        t1.transform.translation.y = -1.55 + self.get_parameter('optitrack_y').get_parameter_value().double_value
        t1.transform.translation.z = 3.11 + self.get_parameter('optitrack_z').get_parameter_value().double_value
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        static_transforms.append(t1)

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'grid_0'
        t2.child_frame_id = 'camera2/map'
        t2.transform.translation.x = 4.65 + self.get_parameter('optitrack_x').get_parameter_value().double_value
        t2.transform.translation.y = 2.8 + self.get_parameter('optitrack_y').get_parameter_value().double_value
        t2.transform.translation.z = 3.11 + self.get_parameter('optitrack_z').get_parameter_value().double_value
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = -1.0
        t2.transform.rotation.w = 0.0
        static_transforms.append(t2)

        # Publicar transformaciones de base de cámara
        static_transforms.extend(self.publish_camera_base_transforms())

        self.static_broadcaster.sendTransform(static_transforms)

    def publish_camera_base_transforms(self):
        # Publicar transformaciones de base de cámara
        static_transforms = []

        # Transformación de la cámara 1
        t1_bl = TransformStamped()
        t1_bl.header.stamp = self.get_clock().now().to_msg()
        t1_bl.header.frame_id = 'camera1/map'
        t1_bl.child_frame_id = 'camera1/base_link'
        t1_bl.transform.translation.x = 0.0
        t1_bl.transform.translation.y = 0.0
        t1_bl.transform.translation.z = 0.0
        t1_bl.transform.rotation.x = 0.0
        t1_bl.transform.rotation.y = 0.0
        t1_bl.transform.rotation.z = 0.0
        t1_bl.transform.rotation.w = 1.0
        static_transforms.append(t1_bl)

        # Transformación de la cámara 2
        t2_bl = TransformStamped()
        t2_bl.header.stamp = self.get_clock().now().to_msg()
        t2_bl.header.frame_id = 'camera2/map'
        t2_bl.child_frame_id = 'camera2/base_link'
        t2_bl.transform.translation.x = 0.0
        t2_bl.transform.translation.y = 0.0
        t2_bl.transform.translation.z = 0.0
        t2_bl.transform.rotation.x = 0.0
        t2_bl.transform.rotation.y = 0.0
        t2_bl.transform.rotation.z = 0.0
        t2_bl.transform.rotation.w = 1.0
        static_transforms.append(t2_bl)

        return static_transforms

    def tf_from_to(self):
        from_frame_rel = 'drone0'
        #from_frame_rel = 'drone0/base_link'
        to_frame_rel1 = 'camera1/map'
        to_frame_rel2 = 'camera2/map'
        try:
            drone_pose1 = self.buffer.lookup_transform(
                to_frame_rel1,
                from_frame_rel,
                rclpy.time.Time())
            drone_pose2 = self.buffer.lookup_transform(
                to_frame_rel2,
                from_frame_rel,
                rclpy.time.Time())
            print(drone_pose2.transform.translation.x)
            print(drone_pose2.transform.translation.y)
            print(drone_pose2.transform.translation.z)
            self.camera_points_drone(drone_pose1, drone_pose2)
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel1} to {from_frame_rel}: {ex}')
        return

    def camera_points_drone(self, drone_pose1, drone_pose2):

        paneo1, tilteo1 = self.move_cam(self.camera1, drone_pose1)
        paneo2, tilteo2 = self.move_cam(self.camera2, drone_pose2)

        rot1 = Rotation.from_euler('xyz', [0, tilteo1/tilt2, paneo1/pan2], degrees=True)
        rot_quat1 = rot1.as_quat()
        print(f"Cuaternio {rot_quat1}")

        rot2 = Rotation.from_euler('xyz', [0, tilteo2/tilt2, paneo2/pan2], degrees=True)
        rot_quat2 = rot2.as_quat()
        print(f"Cuaternio {rot_quat2}")

        # Define transformacion dinamica con xyz = 000 y el pan y el tilt transformado a qx, qy, qz, qw
        # esa transformacion se la pasas a dynamicbroadcaster.sendtransform
        dynamic_transforms = []

        t1_bl = TransformStamped()
        t1_bl.header.stamp = self.get_clock().now().to_msg()
        t1_bl.header.frame_id = 'camera1/map'
        t1_bl.child_frame_id = 'camera1/base_link'
        t1_bl.transform.translation.x = 0.0
        t1_bl.transform.translation.y = 0.0
        t1_bl.transform.translation.z = 0.0
        t1_bl.transform.rotation.x = rot_quat1[0]
        t1_bl.transform.rotation.y = rot_quat1[1]
        t1_bl.transform.rotation.z = rot_quat1[2]
        t1_bl.transform.rotation.w = rot_quat1[3]
        dynamic_transforms.append(t1_bl)
        self.dynamic_broadcaster.sendTransform(dynamic_transforms)

        t2_bl = TransformStamped()
        t2_bl.header.stamp = self.get_clock().now().to_msg()
        t2_bl.header.frame_id = 'camera2/map'
        t2_bl.child_frame_id = 'camera2/base_link'
        t2_bl.transform.translation.x = 0.0
        t2_bl.transform.translation.y = 0.0
        t2_bl.transform.translation.z = 0.0
        t2_bl.transform.rotation.x = rot_quat2[0]
        t2_bl.transform.rotation.y = rot_quat2[1]
        t2_bl.transform.rotation.z = rot_quat2[2]
        t2_bl.transform.rotation.w = rot_quat2[3]
        dynamic_transforms.append(t2_bl)
        self.dynamic_broadcaster.sendTransform(dynamic_transforms)

        print(f"pan1 {paneo1}, tilt1 {tilteo1}")
        self.camera1.pantilt(24, 24, paneo1, -tilteo1, False)
        print(f"pan2 {paneo2}, tilt2 {tilteo2}")
        self.camera2.pantilt(24, 24, paneo2, -tilteo2, False)

    def move_cam(self, camera, drone_pose):
        # Función para mover la cámara hacia la posición del dron
        dx = drone_pose.transform.translation.x
        dy = drone_pose.transform.translation.y
        dz = drone_pose.transform.translation.z
        dxy = math.sqrt(dx ** 2 + dy ** 2)
        if camera.name == "cam1":
            pan_cam = round((math.degrees(math.atan2(dy, dx))) * pan_step1)
            tilt_cam = -round((math.degrees(math.atan2(dz, dxy))) * tilt_step1)
        else:
            pan_cam = round((math.degrees(math.atan2(dy, dx))) * pan_step2)
            tilt_cam = -round((math.degrees(math.atan2(dz, dxy))) * tilt_step2)

        return pan_cam, tilt_cam

def main (args=None):
    rclpy.init(args=args)
    node = CombinedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()