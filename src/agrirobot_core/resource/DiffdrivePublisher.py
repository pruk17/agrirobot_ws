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
        command = input("Press 'w' for forward, 's' for backward, or 'q' to quit: ").strip().lower()

        if command == 'w':
            msg = String()
            msg.data = 'drive forward'
            self.publisher_.publish(msg)
            self.get_logger().info("Sent: drive forward")

        elif command == 's':
            msg = String()
            msg.data = 'drive backward'
            self.publisher_.publish(msg)
            self.get_logger().info("Sent: drive backward")

        elif command == 'q':
            self.get_logger().info("Exiting publisher...")
            rclpy.shutdown()

        else:
            self.get_logger().warn("Unknown key pressed. Use w/s/q.")

def main(args=None):
    rclpy.init(args=args)
    motor_publisher = MotorPublisher()
    try:
        rclpy.spin(motor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        motor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
