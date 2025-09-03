#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoystickPublisher(Node):
    def __init__(self):
        super().__init__('joystick_publisher')

        # Subscriber to 'joy' topic
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # Publisher to '/motor_command'
        self.motor_pub = self.create_publisher(String, '/motor_command', 10)

        # Timer to process joystick at regular intervals
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Store last joystick message
        self.last_joy_msg = None

    def joy_callback(self, msg):
        # Save latest joystick message
        self.last_joy_msg = msg

    def timer_callback(self):
        if self.last_joy_msg is None:
            return  # No joystick message yet

        # Map axes to forward/turn
        forward = self.last_joy_msg.axes[1]
        turn = self.last_joy_msg.axes[0]

        # Prepare motor command
        msg = String()
        msg.data = f"{forward},{turn}"
        self.motor_pub.publish(msg)

        # Print to terminal
        print(f"[TERMINAL] Forward: {forward}, Turn: {turn}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
