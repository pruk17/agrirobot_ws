#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MotorPublisher(Node):
    def __init__(self):
        super().__init__('motor_command_publisher')
        # Create a publisher for the topic 'motor_command' with String messages
        self.publisher_ = self.create_publisher(String, 'motor_command', 10)
        
        # Create a timer to publish messages periodically (every 0.5s here)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # Read keyboard input
        command = input("Press w/a/s/d to move | e = stop | q = quit: ").strip().lower()

        if command == 'w':
            msg = String()
            msg.data = 'drive forward' # Command to drive forward
            self.publisher_.publish(msg)
            self.get_logger().info("Sent: drive forward")

        elif command == 'a':
            msg = String()
            msg.data = 'drive left'
            self.publisher_.publish(msg)
            self.get_logger().info("Sent: drive left")

        elif command == 'd':
            msg = String()
            msg.data = 'drive right'
            self.publisher_.publish(msg)
            self.get_logger().info("Sent: drive right")

        elif command == 's':
            msg = String()
            msg.data = 'drive backward'
            self.publisher_.publish(msg)
            self.get_logger().info("Sent: drive backward")

        elif command == 'e':
            msg = String()
            msg.data = 'drive stop'
            self.publisher_.publish(msg)
            self.get_logger().warn("Sent: drive STOP")

        elif command == 'q':
            self.get_logger().info("Exiting publisher...")
            rclpy.shutdown()

        else:
            self.get_logger().warn("Unknown key pressed. Use w/a/s/d/e/q.")

def main():
    rclpy.init()
    node = MotorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
