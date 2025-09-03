#ifdef Node1
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// ----- Motor pins (2 pins per motor) -----
const int M_LF_IN1 = 12; // Left Front
const int M_LF_IN2 = 14;

const int M_LB_IN1 = 27; // Left Back
const int M_LB_IN2 = 26;

const int M_RF_IN1 = 33; // Right Front
const int M_RF_IN2 = 25;

const int M_RB_IN1 = 4; // Right Back
const int M_RB_IN2 = 0;

// ----- ROS 2 objects -----
rcl_subscription_t subscriber;
rcl_publisher_t publisher_status;       // Publisher for motor status
std_msgs__msg__String msg;
std_msgs__msg__String status_msg;       // Message to publish status
rclc_support_t support;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_executor_t executor;

// ----- Motor control functions -----
void stopMotors() {
    digitalWrite(M_LF_IN1, LOW); digitalWrite(M_LF_IN2, LOW);
    digitalWrite(M_LB_IN1, LOW); digitalWrite(M_LB_IN2, LOW);
    digitalWrite(M_RF_IN1, LOW); digitalWrite(M_RF_IN2, LOW);
    digitalWrite(M_RB_IN1, LOW); digitalWrite(M_RB_IN2, LOW);
}

void forward() {
    digitalWrite(M_LF_IN1, HIGH); digitalWrite(M_LF_IN2, LOW);
    digitalWrite(M_LB_IN1, HIGH); digitalWrite(M_LB_IN2, LOW);
    digitalWrite(M_RF_IN1, HIGH); digitalWrite(M_RF_IN2, LOW);
    digitalWrite(M_RB_IN1, HIGH); digitalWrite(M_RB_IN2, LOW);
}

void backward() {
    digitalWrite(M_LF_IN1, LOW); digitalWrite(M_LF_IN2, HIGH);
    digitalWrite(M_LB_IN1, LOW); digitalWrite(M_LB_IN2, HIGH);
    digitalWrite(M_RF_IN1, LOW); digitalWrite(M_RF_IN2, HIGH);
    digitalWrite(M_RB_IN1, LOW); digitalWrite(M_RB_IN2, HIGH);
}

void turnLeft() {
    digitalWrite(M_LF_IN1, LOW); digitalWrite(M_LF_IN2, HIGH);
    digitalWrite(M_LB_IN1, LOW); digitalWrite(M_LB_IN2, HIGH);
    digitalWrite(M_RF_IN1, HIGH); digitalWrite(M_RF_IN2, LOW);
    digitalWrite(M_RB_IN1, HIGH); digitalWrite(M_RB_IN2, LOW);
}

void turnRight() {
    digitalWrite(M_LF_IN1, HIGH); digitalWrite(M_LF_IN2, LOW);
    digitalWrite(M_LB_IN1, HIGH); digitalWrite(M_LB_IN2, LOW);
    digitalWrite(M_RF_IN1, LOW); digitalWrite(M_RF_IN2, HIGH);
    digitalWrite(M_RB_IN1, LOW); digitalWrite(M_RB_IN2, HIGH);
}

// ----- ROS 2 subscription callback -----
void subscription_callback(const void * msgin) {
    const std_msgs__msg__String *incoming_msg = (const std_msgs__msg__String *)msgin;
    String command = String(incoming_msg->data.data);

    if (command == "drive forward") {
        forward();
        status_msg.data.data = "Forward";
    }
    else if (command == "drive backward") {
        backward();
        status_msg.data.data = "Backward";
    }
    else if (command == "drive left") {
        turnLeft();
        status_msg.data.data = "Turn Left";
    }
    else if (command == "drive right") {
        turnRight();
        status_msg.data.data = "Turn Right";
    }
    else if (command == "drive stop") {
        stopMotors();
        status_msg.data.data = "STOP";
    }
    else {
        stopMotors();
        status_msg.data.data = "Unknown command, STOP";
    }

    // Publish status to ROS2 topic
    std_msgs__msg__String__init(&status_msg);
    rcl_publish(&publisher_status, &status_msg, NULL);
}

void setup() {
    // Motor pins
    pinMode(M_LF_IN1, OUTPUT); pinMode(M_LF_IN2, OUTPUT);
    pinMode(M_LB_IN1, OUTPUT); pinMode(M_LB_IN2, OUTPUT);
    pinMode(M_RF_IN1, OUTPUT); pinMode(M_RF_IN2, OUTPUT);
    pinMode(M_RB_IN1, OUTPUT); pinMode(M_RB_IN2, OUTPUT);
    stopMotors();

    // micro-ROS init
    set_microros_serial_transports(Serial);
    allocator = rcl_get_default_allocator();

    // ตั้งค่า Domain ID = 10
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 79); // <-- Domain ID

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // Node
    rclc_node_init_default(&node, "esp32_motor_node", "", &support);

    // Subscriber for motor command
    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/motor_command"
    );

    // Publisher for motor status
    rclc_publisher_init_default(
        &publisher_status,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/motor_status"
    );

    // Executor
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
#endif
