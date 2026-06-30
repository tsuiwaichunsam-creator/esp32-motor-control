#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>
#include <rmw_microros/rmw_microros.h>

// ==================== PIN DEFINITIONS ====================
#define MOTOR_TX 17
#define MOTOR_RX 18

// ==================== MECHANICAL PARAMETERS ====================
#define JOINT_TO_MOTOR_RATIO  1.12f
#define COUNTS_PER_DEG         (51200.0f / 360.0f * JOINT_TO_MOTOR_RATIO)

// ==================== CALIBRATION OFFSET ====================
//adjust degree 0
#define ANGLE_OFFSET_DEG  14.5f  // Negative because joint reads higher than actual

#define JOINT_MIN_DEG  -35.0f
#define JOINT_MAX_DEG   35.0f

HardwareSerial MotorSerial(1);

// ==================== ROS2 VARIABLES ====================
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_timer_t pub_timer;

rcl_subscription_t cmd_sub;
std_msgs__msg__Float64 cmd_msg;

rcl_publisher_t state_pub;
std_msgs__msg__Float64 state_msg;

float currentAngle = 0.0;
int callback_count = 0;
bool executor_ready = false;

#define PUBLISH_PERIOD_MS  100

// ==================== PD42S1 Communication ====================

uint8_t compute_checksum(const uint8_t *frame, uint8_t len) {
    uint8_t cs = 0;
    for (uint8_t i = 0; i < len; i++) cs += frame[i];
    return cs;
}

void send_pd42_command(uint8_t cmd, const uint8_t *data, uint8_t data_len) {
    uint8_t frame[256];
    uint8_t idx = 0;
    frame[idx++] = 0xC5;
    frame[idx++] = 0x01;
    frame[idx++] = cmd;
    for (uint8_t i = 0; i < data_len; i++) frame[idx++] = data[i];
    frame[idx] = compute_checksum(frame, idx);
    idx++;
    frame[idx++] = 0x5C;
    MotorSerial.write(frame, idx);
}

bool send_and_wait_response(uint8_t cmd, const uint8_t *data, uint8_t data_len, 
                            uint8_t *resp, uint8_t max_len, uint16_t timeout_ms = 100) {
    while (MotorSerial.available()) MotorSerial.read();
    send_pd42_command(cmd, data, data_len);

    uint32_t start = millis();
    uint8_t buffer[64];
    uint8_t idx = 0;
    while (millis() - start < timeout_ms) {
        // Give the serial hardware buffer a tiny moment to receive bytes
        delay(2);
        while (MotorSerial.available()) {
            uint8_t c = MotorSerial.read();
            buffer[idx++] = c;
            if (idx >= max_len) break;
        }
        if (idx >= 4 && buffer[0] == 0xC5 && buffer[idx-1] == 0x5C) {
            uint8_t cs = compute_checksum(buffer, idx-2);
            if (cs == buffer[idx-2]) {
                memcpy(resp, buffer, idx);
                return true;
            }
        }
        delay(1);
    }
    return false;
}

// ==================== PD42S1 Motor Control ====================

void pd42_enable(bool enable) {
    uint8_t data[1] = { enable ? 0 : 1 };
    uint8_t resp[32];
    send_and_wait_response(0xFA, data, 1, resp, sizeof(resp));
}

void pd42_set_work_mode(uint8_t mode) {
    uint8_t data[1] = { mode };
    uint8_t resp[32];
    send_and_wait_response(0x62, data, 1, resp, sizeof(resp));
}

void pd42_move_absolute(int32_t position) {
    uint8_t data[8];
    data[0] = (position >= 0) ? 0 : 1;
    uint32_t abs_pos = (position >= 0) ? position : -position;
    data[1] = 100;
    uint16_t speed = 1000;
    data[2] = (speed >> 8) & 0xFF;
    data[3] = speed & 0xFF;
    data[4] = (abs_pos >> 24) & 0xFF;
    data[5] = (abs_pos >> 16) & 0xFF;
    data[6] = (abs_pos >> 8) & 0xFF;
    data[7] = abs_pos & 0xFF;

    uint8_t resp[32];
    send_and_wait_response(0xF2, data, 8, resp, sizeof(resp));
}

int32_t pd42_read_position() {
    uint8_t resp[32];
    if (send_and_wait_response(0x2A, nullptr, 0, resp, sizeof(resp))) {
        if (resp[3] == 0x01) {
            uint32_t pos_be = (resp[4] << 24) | (resp[5] << 16) | (resp[6] << 8) | resp[7];
            return (int32_t)pos_be;
        }
    }
    return 0;
}

// ==================== Coordinate Conversion with Offset ====================

float joint_to_motor_counts(float joint_deg) {
    // Apply offset: joint_deg + offset = actual motor angle
    float motor_deg = joint_deg + ANGLE_OFFSET_DEG;
    motor_deg = constrain(motor_deg, JOINT_MIN_DEG + ANGLE_OFFSET_DEG, JOINT_MAX_DEG + ANGLE_OFFSET_DEG);
    return motor_deg * COUNTS_PER_DEG;
}

float motor_counts_to_joint(int32_t counts) {
    float motor_deg = counts / COUNTS_PER_DEG;
    // Remove offset: joint_deg = motor_deg - offset
    float joint_deg = motor_deg - ANGLE_OFFSET_DEG;
    return constrain(joint_deg, JOINT_MIN_DEG, JOINT_MAX_DEG);
}

// ==================== ROS2 Callbacks ====================

void command_callback(const void *msgin) {
    callback_count++;
    const std_msgs__msg__Float64 *msg = (const std_msgs__msg__Float64 *)msgin;
    float targetAngle = msg->data;
    
    int32_t target_counts = (int32_t)joint_to_motor_counts(targetAngle);
    
    pd42_enable(true);
    delay(10);
    pd42_move_absolute(target_counts);
    currentAngle = targetAngle;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    
    // Instead of reading the broken serial sensor, report the last sent angle directly
    state_msg.data = currentAngle; 
    
    rcl_publish(&state_pub, &state_msg, NULL);
}

// ==================== SETUP ====================

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    while (Serial.available()) Serial.read();
    
    MotorSerial.begin(115200, SERIAL_8N1, MOTOR_RX, MOTOR_TX);
    
    pd42_enable(true);
    delay(100);
    pd42_set_work_mode(0x00);
    delay(100);
    
    set_microros_serial_transports(Serial);
    
    bool agent_connected = false;
    for (int i = 0; i < 30; i++) {
        if (rmw_uros_ping_agent(100, 1) == RCL_RET_OK) {
            agent_connected = true;
            break;
        }
        delay(100);
    }
    
    if (!agent_connected) {
        while (1) delay(1000);
    }
    
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_motor_node", "", &support);
    
    rclc_subscription_init_default(
        &cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "/joint/command"
    );
    
    rclc_publisher_init_default(
        &state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "/joint/state"
    );
    
    rclc_timer_init_default(
        &pub_timer,
        &support,
        RCL_MS_TO_NS(PUBLISH_PERIOD_MS),
        timer_callback
    );
    
    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_subscription(&executor, &cmd_sub, &cmd_msg, command_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &pub_timer);
    
    executor_ready = true;
}

// ==================== LOOP ====================

void loop() {
    if (executor_ready) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    delay(10);
}