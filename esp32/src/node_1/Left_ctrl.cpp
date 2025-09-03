// ===== ESP32 micro-ROS Motor Node (Node1/Node2 via build flags) =====
// Build flags:
//   [env:esp_node1] build_flags = -D Node1
//   [env:esp_node2] build_flags = -D Node2

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>            // for rcl_get_error_string, rcl_reset_error

#include <std_msgs/msg/string.h>
#include <rosidl_runtime_c/string_functions.h>

// ---------- Simple error-check macros ----------
#include <rcl/error_handling.h>  // rcl_get_error_string, rcl_reset_error

//every  functions of rcl_* --> return rcl_ret_t tell the result (OK/ERROR)
#define CHECK(fn) do { \
  rcl_ret_t _rc = (fn); \
  if (_rc != RCL_RET_OK) { \
    Serial.print("[rcl WARN] "); \
    Serial.println(rcl_get_error_string().str); \
    rcl_reset_error(); \
  } \
} while (0)
  //if error, rcl_get_error_string().str will describe the cause 
  //from every layers of ROS 2 (Ex. DDS, RMW)
  //rcl_reset_error() will reset the error state of rcl

  //if error, rcl_get_error_string().str will describe the cause 
  //from every layers of ROS 2 (Ex. DDS, RMW)

  //CHECK_FATAL will stop the program if error occurs
#define CHECK_FATAL(fn) do { \
  rcl_ret_t _rc = (fn); \
  if (_rc != RCL_RET_OK) { \
    Serial.print("[rcl FATAL] "); \
    Serial.println(rcl_get_error_string().str); \
    rcl_reset_error(); \
    while (true) { delay(100); } \
  } \
} while (0)


// ---------- Compile-time identity ----------
#if defined(Node1)
  static const char* NODE_NAME     = "esp32_motor_node1";
  static const char* STATUS_TOPIC  = "/motor_status/node1";
#elif defined(Node2)
  static const char* NODE_NAME     = "esp32_motor_node2";
  static const char* STATUS_TOPIC  = "/motor_status/node2";
#else
  #error "You must define either -D Node1 or -D Node2 in platformio.ini"
#endif

// ---------- Motor pins ----------
static const int M_LF_IN1 = 12;
static const int M_LF_IN2 = 14;
static const int M_LB_IN1 = 27;
static const int M_LB_IN2 = 26;
static const int M_RF_IN1 = 33;
static const int M_RF_IN2 = 25;
static const int M_RB_IN1 = 4;
// Avoid GPIO0 (BOOT strap). Use GPIO13 instead.
static const int M_RB_IN2 = 13;

// ---------- micro-ROS objects ----------
rcl_allocator_t       allocator;
rclc_support_t        support;
rcl_node_t            node;
rclc_executor_t       executor;

rcl_subscription_t    sub_cmd;
rcl_publisher_t       pub_status;

std_msgs__msg__String msg_cmd;
std_msgs__msg__String msg_status;

// ---------- Motor helpers ----------
static inline void stopMotors() {
  digitalWrite(M_LF_IN1, LOW);  digitalWrite(M_LF_IN2, LOW);
  digitalWrite(M_LB_IN1, LOW);  digitalWrite(M_LB_IN2, LOW);
  digitalWrite(M_RF_IN1, LOW);  digitalWrite(M_RF_IN2, LOW);
  digitalWrite(M_RB_IN1, LOW);  digitalWrite(M_RB_IN2, LOW);
}
static inline void forward() {
  digitalWrite(M_LF_IN1, HIGH); digitalWrite(M_LF_IN2, LOW);
  digitalWrite(M_LB_IN1, HIGH); digitalWrite(M_LB_IN2, LOW);
  digitalWrite(M_RF_IN1, HIGH); digitalWrite(M_RF_IN2, LOW);
  digitalWrite(M_RB_IN1, HIGH); digitalWrite(M_RB_IN2, LOW);
}
static inline void backward() {
  digitalWrite(M_LF_IN1, LOW);  digitalWrite(M_LF_IN2, HIGH);
  digitalWrite(M_LB_IN1, LOW);  digitalWrite(M_LB_IN2, HIGH);
  digitalWrite(M_RF_IN1, LOW);  digitalWrite(M_RF_IN2, HIGH);
  digitalWrite(M_RB_IN1, LOW);  digitalWrite(M_RB_IN2, HIGH);
}
static inline void turnLeft() {
  digitalWrite(M_LF_IN1, LOW);  digitalWrite(M_LF_IN2, HIGH);
  digitalWrite(M_LB_IN1, LOW);  digitalWrite(M_LB_IN2, HIGH);
  digitalWrite(M_RF_IN1, HIGH); digitalWrite(M_RF_IN2, LOW);
  digitalWrite(M_RB_IN1, HIGH); digitalWrite(M_RB_IN2, LOW);
}
static inline void turnRight() {
  digitalWrite(M_LF_IN1, HIGH); digitalWrite(M_LF_IN2, LOW);
  digitalWrite(M_LB_IN1, HIGH); digitalWrite(M_LB_IN2, LOW);
  digitalWrite(M_RF_IN1, LOW);  digitalWrite(M_RF_IN2, HIGH);
  digitalWrite(M_RB_IN1, LOW);  digitalWrite(M_RB_IN2, HIGH);
}

// ---------- Subscription callback ----------
void cmd_callback(const void * msgin) {
  const std_msgs__msg__String *in = (const std_msgs__msg__String *) msgin;
  const String command = String(in->data.data);

  if (command == "drive forward") {
    forward();
    rosidl_runtime_c__String__assign(&msg_status.data, "Forward");
  } else if (command == "drive backward") {
    backward();
    rosidl_runtime_c__String__assign(&msg_status.data, "Backward");
  } else if (command == "drive left") {
    turnLeft();
    rosidl_runtime_c__String__assign(&msg_status.data, "Turn Left");
  } else if (command == "drive right") {
    turnRight();
    rosidl_runtime_c__String__assign(&msg_status.data, "Turn Right");
  } else if (command == "drive stop") {
    stopMotors();
    rosidl_runtime_c__String__assign(&msg_status.data, "STOP");
  } else {
    stopMotors();
    rosidl_runtime_c__String__assign(&msg_status.data, "Unknown");
  }

  // Check the return value to silence -Wunused-result and catch errors
  CHECK(rcl_publish(&pub_status, &msg_status, NULL));

}

// ---------- Setup ----------
void setup() {
  // 1) Motor pins
  pinMode(M_LF_IN1, OUTPUT); pinMode(M_LF_IN2, OUTPUT);
  pinMode(M_LB_IN1, OUTPUT); pinMode(M_LB_IN2, OUTPUT);
  pinMode(M_RF_IN1, OUTPUT); pinMode(M_RF_IN2, OUTPUT);
  pinMode(M_RB_IN1, OUTPUT); pinMode(M_RB_IN2, OUTPUT);
  stopMotors();

  // 2) Serial transport (match agent baudrate)
  Serial.begin(115200);
  delay(50);
  set_microros_serial_transports(Serial);

  // 3) micro-ROS init with Domain ID = 79 (matches your Pi5)
  allocator = rcl_get_default_allocator();

  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  CHECK_FATAL(rcl_init_options_init(&init_options, allocator));
  CHECK_FATAL(rcl_init_options_set_domain_id(&init_options, 79));
  CHECK_FATAL(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  CHECK_FATAL(rclc_node_init_default(&node, NODE_NAME, "", &support));
  CHECK_FATAL(rclc_subscription_init_default(
  &sub_cmd, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
  "/motor_command"));

  CHECK_FATAL(rclc_publisher_init_default(
  &pub_status, &node,
  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
  STATUS_TOPIC));

  CHECK_FATAL(rclc_executor_init(&executor, &support.context, 1, &allocator));
  CHECK_FATAL(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &cmd_callback, ON_NEW_DATA));
}

// ---------- Loop ----------
void loop() {
  // Spin non-blocking
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
}
