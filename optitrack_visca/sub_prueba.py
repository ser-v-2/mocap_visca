import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import sys
import os
sys.path.append(os.path.abspath('/home/ser_v/tfg/docker/workspace/src/optitrack_visca/optitrack_visca'))
from Viscaoverip import Camera
from mocap4r2_msgs.msg import RigidBodies
from mocap4r2_msgs.msg import RigidBody

class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            RigidBodies,
            'rigbod',
            self.rb_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.Camera1 = Camera("cam1", "192.168.1.89", 52381)
        self.Camera2 = Camera("cam2", "192.168.1.90", 52381)
        print("Inciando prueba Fase 2")

    def rb_callback(self, msg):
        self.get_logger().info('Recibido: "%s"' % msg)
        for rigid_body in msg.rigidbodies:
            x = rigid_body.pose.position.x
            y = rigid_body.pose.position.y
            z = rigid_body.pose.position.z
            print(f"Apuntando a coordenadas {x}, {y}, {z}.")
            pan1, tilt1 = self.Camera1.calculate_pantilt1(x, y, z)
            pan2, tilt2 = self.Camera2.calculate_pantilt2(x, y, z)
            self.Camera1.zoom(0)
            self.Camera2.zoom(0)
            self.Camera1.pantilt(24, 24, pan1, tilt1, False)
            self.Camera2.pantilt(24, 24, pan2, tilt2, False)
            print(f"pan1 {pan1}, tilt1 {tilt1}")
            print(f"pan2 {pan2}, tilt2 {tilt2}")

def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()