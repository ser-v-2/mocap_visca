import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from mocap4r2_msgs.msg import RigidBodies
from mocap4r2_msgs.msg import RigidBody

class PosePublisher(Node):

    def __init__(self):
        super().__init__('pose_publisher')
        self.declare_parameter('position', [0, 0, 0])
        self.position = self.get_parameter('position')
        self.publisher_rb = self.create_publisher(RigidBodies, 'rigbod', 10)
        timer_period = 1  # segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = RigidBodies()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.frame_number = 42
        rigid_body = RigidBody()
        rigid_body.rigid_body_name = "mi_rigid_body"
        rigid_body.markers = []  # Llena esto con los marcadores adecuados
        rigid_body.pose.position.x = float(input("Introduce x:"))
        rigid_body.pose.position.y = float(input("Introduce y:"))
        rigid_body.pose.position.z = float(input("Introduce z:"))
        msg.rigidbodies.append(rigid_body)
        self.publisher_rb.publish(msg)  # Publica en el tópico 'rigbod'
        self.get_logger().info('Publicando RigidBodies: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin_once(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
