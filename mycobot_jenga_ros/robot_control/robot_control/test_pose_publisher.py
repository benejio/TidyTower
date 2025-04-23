import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class TestPosePublisher(Node):
    def __init__(self):
        super().__init__('test_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        timer_period = 2.0  # publish every 2 seconds
        self.timer = self.create_timer(timer_period, self.publish_pose)

    def publish_pose(self):
        msg = PoseStamped()
        msg.header.frame_id = "g_base"  # use your base frame name
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 0.20
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.15
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Published test target pose:\n"
            f"  Position: x={msg.pose.position.x:.3f}, y={msg.pose.position.y:.3f}, z={msg.pose.position.z:.3f}\n"
            f"  Orientation: x={msg.pose.orientation.x:.3f}, y={msg.pose.orientation.y:.3f}, "
            f"z={msg.pose.orientation.z:.3f}, w={msg.pose.orientation.w:.3f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = TestPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

