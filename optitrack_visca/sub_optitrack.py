from Viscaoverip import Camera
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from mocap4r2_msgs.msg import RigidBodies
from mocap4r2_msgs.msg import RigidBody
import time


class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            RigidBodies,
            '/rigid_bodies',
            self.rb_callback,
            1000)
        
        self.Camera1 = Camera("cam1", '192.168.1.89', 52381)
        self.Camera2 = Camera("cam2", '192.168.1.90', 52381)
        #self.Camera1.pantilt_home()
        #self.Camera2.pantilt_home()
        self.Camera1.zoom_to(0.4)
        self.Camera2.zoom_to(0.4)
        self.optitrack_origin = (3, 4, 0)
        print("ok")
        self.counter = 0
        #time.sleep(5)

    def rb_callback(self, msg):
        self.counter += 1
        if self.counter % 10 > 0:
            return
        self.counter = 0
        self.get_logger().info('Recibido: "%s"' % msg)
        for rigid_body in msg.rigidbodies:
            if rigid_body.rigid_body_name == '33':
                x = rigid_body.pose.position.x + self.optitrack_origin[0]
                y = rigid_body.pose.position.y + self.optitrack_origin[1]
                z = rigid_body.pose.position.z + self.optitrack_origin[2]
                print(f'{x=} {y=} {z=}')
                pan1, tilt1 = self.Camera1.calculate_pantilt1(x, y, z)
                pan2, tilt2 = self.Camera2.calculate_pantilt2(x, y, z)
                self.Camera1.pantilt(24, 24, pan1, tilt1, False)
                self.Camera2.pantilt(24, 24, pan2, tilt2, False)
                print(f"pan1 {pan1}, tilt1 {tilt1}")
                print(f"pan2 {pan2}, tilt2 {tilt2}")
        return
    
def main(args=None):
    rclpy.init(args=args)
    pose_subscriber = PoseSubscriber()
    rclpy.spin(pose_subscriber)
    pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()