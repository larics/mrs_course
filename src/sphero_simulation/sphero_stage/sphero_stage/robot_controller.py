import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer_ = self.create_timer(0.5, self.send_commands)
        self.get_logger().info('Robot Controller Node has started.')

    def send_commands(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.0
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing cmd_vel')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()