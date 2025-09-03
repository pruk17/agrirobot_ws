
#include <Arduino.h>
#include <micro_ros_platformio.h>   // micro-ROS client library for Arduino/ESP32
#include <rcl/rcl.h>             // Core ROS2 client library
#include <rclc/rclc.h>           // rclc wrapper to simplify rcl usage
#include <rclc/executor.h>       // Executor to handle subscription callbacks
#include <std_msgs/msg/string.h> // Standard ROS2 String message type

// --- Motor driver pins ---

//front left
const int M1A_L = 12; // Motor A forward
const int M2A_L = 14; // Motor A forward

//back left
const int M1B_L = 27; // Motor B backward
const int M2B_L = 26; // Motor B backward

//front right
const int M1A_R = 25; // Motor A forward
const int M2A_R = 33; // Motor A forward

//back right
const int M1B_R = 32; // Motor B backward
const int M2B_R = 35; // Motor B backward

// --- Motor control functions ---
void brake() {
  digitalWrite(M1A_L, LOW); digitalWrite(M2A_L, LOW);
  digitalWrite(M1B_L, LOW); digitalWrite(M2B_L, LOW);
}

void forward() {
  digitalWrite(M1A_L, HIGH); digitalWrite(M2A_L, HIGH);
  digitalWrite(M1B_L, LOW); digitalWrite(M2B_L, LOW);
}

void backward() {
  digitalWrite(M1A_L, LOW); digitalWrite(M2A_L, LOW);
  digitalWrite(M1B_L, HIGH); digitalWrite(M2B_L, HIGH);
}

// --- micro-ROS variables ---
rcl_subscription_t subscriber;   // Subscription handle
std_msgs__msg__String msg;       // Storage for received message
rclc_support_t support;          // Support struct (context + allocator)
rcl_node_t node;                 // ROS2 node handle
rcl_allocator_t allocator;       // ROS2 allocator
rclc_executor_t executor;        // Executor to manage callbacks

// --- Callback for subscription ---
// This function is called whenever a new message arrives on "motor_command" topic
void subscription_callback(const void * msgin) {
  const std_msgs__msg__String * incoming_msg = (const std_msgs__msg__String *)msgin;
  String command = String(incoming_msg->data.data);

  // Compare the received string and control motors accordingly
  if (command == "drive forward") {
    Serial.println("ESP32: drive forward");
    forward();
  }
  else if (command == "drive backward") {
    Serial.println("ESP32: drive backward");
    backward();
  }
  else {
    Serial.println("ESP32: stop");
    brake();
  }
}

// --- Arduino setup function ---
void setup() {
  Serial.begin(115200); // Initialize Serial for debugging and micro-ROS transport

  // Configure motor pins as outputs
  pinMode(M1A, OUTPUT); pinMode(M2A, OUTPUT);
  pinMode(M1B, OUTPUT); pinMode(M2B, OUTPUT);
  brake(); // Ensure motors are stopped at startup

  // --- micro-ROS setup ---
  // Initialize serial transport for micro-ROS agent communication
  set_microros_serial_transports(Serial);

  // Get default allocator
  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support struct (context + allocator)
  rclc_support_init(&support, 0, NULL, &allocator);

  // Create ROS2 node with name "esp32_motor_node"
  rclc_node_init_default(&node, "esp32_motor_node", "", &support);

  // Initialize subscriber to topic "motor_command" with message type std_msgs/String
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "motor_command"
  );

  // Initialize executor with capacity for 1 handle (subscription)
  rclc_executor_init(&executor, &support.context, 1, &allocator);

  // Add the subscriber and its callback to the executor
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

// --- Arduino loop function ---
void loop() {
  // Spin executor to process incoming messages (100 ms per spin)
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
