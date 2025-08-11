#include <Arduino.h>               // Basic Arduino/ESP32 library for pin control and Serial
#include <micro_ros_arduino.h>    // micro-ROS client library for Arduino/ESP32
                                 // Provides functions to connect micro-ROS Agent via Serial
                                // e.g., set_microros_serial_transports()
#include <rcl/rcl.h>             // Core ROS2 client library (rcl)
                                // Used to create nodes, publishers, subscribers, timers
                               // Includes types like rcl_node_t, rcl_subscription_t
#include <rclc/rclc.h>           // rclc wrapper library on top of rcl
                                // Simplifies usage on microcontrollers (MCUs)
                               // Provides helper functions like rclc_support_init(), rclc_node_init_default()
#include <rclc/executor.h>        // Executor to manage event loop for callbacks, subscriptions, timers
#include <std_msgs/msg/string.h>    // Definition of the ROS2 standard message type std_msgs/msg/String

// Define motor driver control pins for forward and backward directions
const int motorPinForward = 12;
const int motorPinBackward = 13;

// micro-ROS related variables
rcl_subscription_t subscriber;       // Subscription handle to receive messages from a ROS2 topic
std_msgs__msg__String msg;           // Message storage for received String messages
rclc_support_t support;              // Support structure containing ROS2 context and allocator info
rcl_node_t node;                     // ROS2 node handle
rcl_allocator_t allocator;           // Memory allocator for ROS2 operations
rclc_executor_t executor;            // Executor to manage and dispatch callbacks

// Callback function invoked whenever a new message arrives on the subscribed topic
void subscription_callback(const void * msgin) {
  // Implementation to handle incoming message (defined elsewhere)
}

void setup() {
  Serial.begin(115200);                 // Initialize Serial communication for debugging and micro-ROS transport
  pinMode(motorPinForward, OUTPUT);    // Set motor forward pin as output
  pinMode(motorPinBackward, OUTPUT);   // Set motor backward pin as output

  // Configure micro-ROS to use Serial transport (e.g., USB serial connection) to communicate with Agent
  set_microros_transports();
  // Obtain the default allocator used for memory management in ROS2
  allocator = rcl_get_default_allocator();

  // Initialize the ROS2 support structure including context and allocator
  rclc_support_init(&support, 0, NULL, &allocator);

  // Initialize the ROS2 node with name "esp32_motor_node"
  rclc_node_init_default(&node, "esp32_motor_node", "", &support);

  // Initialize a subscriber to the topic "motor_command" that expects std_msgs/String messages
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "motor_command");

  // Initialize the executor with capacity for 1 handle (the subscription callback)
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // Add the subscription and its callback function to the executor for handling incoming messages
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
  // Run the executor to process available callbacks for up to 100 milliseconds (converted to nanoseconds)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
// Control motor based on received messages