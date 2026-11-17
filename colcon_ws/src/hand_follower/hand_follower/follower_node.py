import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class HandFollowerNode(Node):
    def __init__(self):
        super().__init__('hand_follower')
        
        self.subscription = self.create_subscription(
            String,
            '/robot_command',
            self.command_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.target_locked = False
        self.current_distance = 0.0
        self.current_angle = 0.0
        self.safe_distance = 1.2
        
        self.get_logger().info('Hand Follower Node started. Waiting for commands...')

    def command_callback(self, msg):
        try:
            cmd_data = json.loads(msg.data)
            command = cmd_data.get("cmd")
            
            if command == "stop":
                self.target_locked = False
                
            elif command == "go_to_target" or command == "follow":
                self.target_locked = True
                self.current_angle = float(cmd_data.get("angle_deg", 0.0))
                self.current_distance = float(cmd_data.get("distance_m", 0.0))
                self.safe_distance = float(cmd_data.get("stop_distance_m", 1.2))
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def timer_callback(self):
        if self.target_locked:
            twist = Twist()
            
            if self.current_distance <= self.safe_distance:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.15
                twist.angular.z = -self.current_angle * 0.01
            
            self.cmd_vel_pub.publish(twist)
            
        else:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = HandFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_twist = Twist()
        node.cmd_vel_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()