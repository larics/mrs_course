import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

NUM_ROBOTS = 6
MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1.0

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        self.publishers_ = []
        for i in range(NUM_ROBOTS):
            topic_name = f'/robot_{i}/cmd_vel'
            pub = self.create_publisher(Twist, topic_name, 10)
            self.publishers_.append(pub)

        self.timer_ = self.create_timer(1.0, self.send_random_commands)
        self.get_logger().info(f'Robot Controller Node has started for {NUM_ROBOTS} robots.')

    def send_random_commands(self):
        for i in range(NUM_ROBOTS):
            msg = Twist()
            
            msg.linear.x = random.uniform(0.0, MAX_LINEAR_SPEED)
            msg.angular.z = random.uniform(-MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            
            self.publishers_[i].publish(msg)
            
            if i == 0:
                self.get_logger().info('Publishing random cmd_vel to all robots...')

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()