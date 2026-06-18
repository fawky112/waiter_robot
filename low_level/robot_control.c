#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <cmath>
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include "soc/rtc_cntl_reg.h"
// =============================================
//  SECTION 1 — PIN DEFINITIONS & CONSTANTS
// =============================================

// ── Encoder pins ─────────────────────────────
#define RIGHT_ENC_A   16
#define RIGHT_ENC_B   17
#define LEFT_ENC_A    14
#define LEFT_ENC_B    13

// ── Motor driver pins (DRV-1012) ──────────────
#define RIGHT_RPWM    33
#define RIGHT_LPWM    25
#define LEFT_RPWM     26
#define LEFT_LPWM     27

// Stepper Z 
#define DIR_Z 19
#define STEP_Z 18
#define LIMIT_Z_HOME  23
// ── Emergency Stop Switch ─────────────────────
#define EMERGENCY_STOP_PIN 4   

// ── Ultrasonic pins ───────────────────────────
#define LEFT_TRIG     32
#define LEFT_ECHO     34
#define MID_TRIG      12
#define MID_ECHO      35
#define RIGHT_TRIG    15
#define RIGHT_ECHO    36

// ── IMU config ────────────────────────────────
#define IMU_SDA           21
#define IMU_SCL           22
#define IMU_ADDR          0x68
#define IMU_UPDATE_MS     10
#define IMU_CALIB_SAMPLES 200

// ── Ultrasonic timing & threshold ────────────
#define US_UPDATE_MS      50
#define SENSOR_GAP_US     500
#define TIMEOUT_US        23200
#define OBSTACLE_CM       30

// ── ESP32 LEDC PWM config ─────────────────────
#define PWM_FREQ        1000    // 1 kHz
#define PWM_RESOLUTION  8       // 8-bit → 0–255

// ── Motor safety limits ───────────────────────
#define MIN_PWM         120
#define MAX_PWM         250
#define BASE_DRIVE_PWM  150
#define MANUAL_SPEED    180    // Manual mode speed

// ── Encoder physical constants ────────────────
#define ENCODER_PPR         12
#define GEAR_RATIO          260
#define TICKS_PER_REV       (ENCODER_PPR * 4 * GEAR_RATIO)
#define WHEEL_DIAMETER_M    0.065f
#define METRES_PER_TICK     (float)(M_PI * WHEEL_DIAMETER_M / TICKS_PER_REV)

// ── Velocity calculation interval ────────────
#define VELOCITY_CALC_MS    20

// ── DISTANCE PID gains ────────────────────────
#define DIST_PID_KP    10.0f
#define DIST_PID_KI     5.0f
#define DIST_PID_KD     1.0f
#define PID_INTERVAL_MS   20

// ── HEADING PID gains ────────────────────────
#define HEAD_PID_KP     2.5f
#define HEAD_PID_KI     0.2f
#define HEAD_PID_KD     0.1f
#define MAX_HEAD_CORRECTION 150
#define HEADING_TOLERANCE 2.0f

// Stepper parameters
#define Z_motorSteps    200
#define Z_lead          8.0
#define Z_microsteps    1
#define Z_stepDelay     600

// ── Navigation config ─────────────────────────
#define STOP_DURATION_MS    360000
#define PRINT_INTERVAL_MS   200
#define POSITION_TOLERANCE_M 0.01f
#define FINAL_MOVE_DISTANCE_M 0.20f  // 20 cm final move

// ── Obstacle Avoidance Constants ──────────────────
#define OBSTACLE_WAIT_TIME_MS    5000   // 10 seconds max wait
#define OBSTACLE_AVOID_DISTANCE_M 0.50   // 50 cm avoid distance
#define AVOIDANCE_SPEED_PWM       180    // Speed during avoidance
#define AVOIDANCE_RETURN_ATTEMPTS 5      // Max avoidance attempts

// ── Manual mode constants ─────────────────────
#define MANUAL_COMMAND_TIMEOUT_MS  5000  // Auto-return to auto mode after 5s of no commands

// Table Positions
#define TABLE1_X 1.5f
#define TABLE1_Y -0.5f
#define TABLE2_X 1.5f
#define TABLE2_Y 0.5f

// WiFi and micro-ROS configuration
char ssid[] = "FW";
char password[] = "fawky6000";
char agent_ip[] = "172.20.10.6";
const uint16_t agent_port = 8888;

// State definitions for publisher
#define STATE_IDLE              0
#define READ                    1
#define STATE_H_TO_T1           11
#define STATE_T1_TO_H_FIRST     12
#define STATE_H_TO_K            13
#define STATE_K_TO_H            14
#define STATE_H_TO_T1_STEPPER   15
#define STATE_T1_TO_H_FINAL     16
#define STATE_COMPLETE          17
#define STATE_WAITING           18
#define STATE_OBSTACLE          19
#define STATE_STEPPER_UP        20
#define STATE_STEPPER_DOWN      21
#define STATE_AVOIDING_OBSTACLE 22
#define STATE_MANUAL            23
#define STATE_H_TO_T2           41
#define STATE_T2_TO_H_FIRST     42
#define STATE_H_TO_T2_STEPPER   45
#define STATE_T2_TO_H_FINAL     46

// Manual command characters
#define CMD_FORWARD      'F'
#define CMD_BACKWARD     'B'
#define CMD_LEFT         'L'
#define CMD_RIGHT        'R'
#define CMD_STEPPER_UP   'U'
#define CMD_STEPPER_DOWN 'D'
#define CMD_STOP         'S'
#define CMD_EXIT_MANUAL  'E'

// Global control flags
volatile bool start = false;
volatile bool start_navigation_flag = false;
volatile bool continue_sequence_flag = false;
volatile bool complete_rest_mode = false;
volatile int continue_step = 0;
volatile int received_command = 0;
int current_state = STATE_IDLE;

// Detection flags
bool table1_detected = false;
bool table2_detected = false;
bool table1_finish = false;
bool table2_finish = false;
bool seq1_completed = false;
bool seq2_completed = false;
QueueHandle_t publish_queue = NULL;
volatile bool ros_ready = false;

// Current sequence tracking
enum ActiveSequence {
    SEQ_NONE = 0,
    SEQ_1 = 1,  // Table 1 without stepper
    SEQ_2 = 2,  // Table 2 without stepper
    SEQ_3 = 3,  // Table K with stepper (after SEQ1)
    SEQ_4 = 4   // Table K with stepper (after SEQ2)
};
ActiveSequence current_sequence = SEQ_NONE;

// Manual mode variables
bool manual_mode_active = false;
char last_manual_command = 0;
unsigned long last_manual_command_time = 0;

// Saved target state for resuming after manual mode
struct SavedTargetState {
    float target_x_m;
    float target_y_m;
    float final_yaw_deg;
    int current_step;
    bool need_initial_backward;
    bool need_final_forward;
    
} saved_target_state;

// =============================================
//  SECTION 2 — NAVIGATION PHASES
// =============================================

enum NavPhase {
    NAV_IDLE = -1,
    NAV_ROTATE_FOR_X = 0,
    NAV_MOVE_X,
    NAV_ROTATE_FOR_Y,
    NAV_MOVE_Y,
    NAV_ROTATE_TO_FINAL,
    NAV_INITIAL_BACKWARD_MOVE,
    NAV_FINAL_MOVE_FORWARD,
    NAV_STOP,
    NAV_WAIT_STEPPER,
    NAV_DONE
};

// =============================================
//  SECTION 3 — SHARED DATA STRUCTURES
// =============================================

struct UltrasonicData {
    float left_cm;
    float mid_cm;
    float right_cm;
    bool obstacle_detected;
    unsigned long timestamp;
};

struct NavigationData {
    float target_x_m;
    float target_y_m;
    float final_yaw_deg;
    float target_heading_for_x;
    float target_heading_for_y;
    float distance_to_move_x;
    float distance_to_move_y;
    float final_move_distance;
    bool need_initial_backward;
    bool need_final_forward;
    NavPhase phase;
    bool movement_complete;
    bool stepper_activated;
};

struct ObstacleAvoidanceData {
    bool is_avoiding;
    unsigned long obstacle_start_time;
    unsigned long wait_start_time;
    bool waiting_for_clear;
    int avoidance_attempts;
    float original_target_x;
    float original_target_y;
    float original_yaw;
    float original_distance_remaining;
    float remaining_x;
    float remaining_y;
    char current_axis;  // 'X' or 'Y' or 'F' (final)
    bool avoidance_complete;
    int selected_direction;
};

static UltrasonicData shared_ultrasonic;
static NavigationData shared_navigation;
static ObstacleAvoidanceData avoidance_data;
static SemaphoreHandle_t data_mutex = NULL;

// micro-ROS variables
rcl_subscription_t subscriber_detection;
rcl_subscription_t subscriber_screen;
rcl_subscription_t subscriber_gui;
rcl_subscription_t subscriber_manual;
rcl_publisher_t publisher;
std_msgs__msg__Int32 msg_sub_detection;
std_msgs__msg__Int32 msg_sub_screen;
std_msgs__msg__Int32 msg_sub_gui;
std_msgs__msg__Int32 msg_sub_manual;
std_msgs__msg__Int32 msg_pub;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// =============================================
//  SECTION 4 — ENCODER STATE & ISRs
// =============================================

static volatile long  right_ticks       = 0;
static volatile long  left_ticks        = 0;
static long           right_ticks_prev  = 0;
static long           left_ticks_prev   = 0;
static unsigned long  last_enc_update_ms = 0;
static float          right_velocity_ms = 0.0f;
static float          left_velocity_ms  = 0.0f;
static float robot_x_m = 0.0f;
static float robot_y_m = 0.0f;

static void IRAM_ATTR isr_right_enc_a() {
    bool a = digitalRead(RIGHT_ENC_A);
    bool b = digitalRead(RIGHT_ENC_B);
    right_ticks += (a == b) ? 1 : -1;
}

static void IRAM_ATTR isr_right_enc_b() {
    bool a = digitalRead(RIGHT_ENC_A);
    bool b = digitalRead(RIGHT_ENC_B);
    right_ticks += (a != b) ? 1 : -1;
}

static void IRAM_ATTR isr_left_enc_a() {
    bool a = digitalRead(LEFT_ENC_A);
    bool b = digitalRead(LEFT_ENC_B);
    left_ticks += (a == b) ? -1 : 1;
}

static void IRAM_ATTR isr_left_enc_b() {
    bool a = digitalRead(LEFT_ENC_A);
    bool b = digitalRead(LEFT_ENC_B);
    left_ticks += (a != b) ? -1 : 1;
}

// =============================================
//  SECTION 5 — ENCODER FUNCTIONS
// =============================================

void encoders_reset() {
    noInterrupts();
    right_ticks = 0;
    left_ticks  = 0;
    interrupts();
    right_ticks_prev  = 0;
    left_ticks_prev   = 0;
    right_velocity_ms = 0.0f;
    left_velocity_ms  = 0.0f;
    Serial.println("[ENCODER] Encoders reset to zero");
}

void encoders_reset_phase() {
    noInterrupts();
    right_ticks = 0;
    left_ticks = 0;
    interrupts();
    right_ticks_prev = 0;
    left_ticks_prev = 0;
}

void encoders_init() {
    pinMode(RIGHT_ENC_A, INPUT_PULLUP);
    pinMode(RIGHT_ENC_B, INPUT_PULLUP);
    pinMode(LEFT_ENC_A,  INPUT_PULLUP);
    pinMode(LEFT_ENC_B,  INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), isr_right_enc_a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B), isr_right_enc_b, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  isr_left_enc_a,  CHANGE);
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B),  isr_left_enc_b,  CHANGE);

    encoders_reset();
    last_enc_update_ms = millis();
    robot_x_m = 0.0f;
    robot_y_m = 0.0f;
}

void encoders_update() {
    unsigned long now   = millis();
    unsigned long dt_ms = now - last_enc_update_ms;

    if (dt_ms < VELOCITY_CALC_MS) return;

    noInterrupts();
    long r_now = right_ticks;
    long l_now = left_ticks;
    interrupts();

    long  delta_r = r_now - right_ticks_prev;
    long  delta_l = l_now - left_ticks_prev;
    float dt_s    = dt_ms / 1000.0f;

    right_velocity_ms = (delta_r * METRES_PER_TICK) / dt_s;
    left_velocity_ms  = (delta_l * METRES_PER_TICK) / dt_s;

    float avg_distance = ((delta_r + delta_l) * METRES_PER_TICK) / 2.0f;
    float current_yaw_rad = get_yaw() * M_PI / 180.0f;
    
    robot_x_m += avg_distance * cos(current_yaw_rad);
    robot_y_m += avg_distance * sin(current_yaw_rad);

    right_ticks_prev = r_now;
    left_ticks_prev  = l_now;
    last_enc_update_ms = now;
}

float encoder_get_right_distance_m() {
    noInterrupts();
    long t = right_ticks;
    interrupts();
    return t * METRES_PER_TICK;
}

float encoder_get_left_distance_m() {
    noInterrupts();
    long t = left_ticks;
    interrupts();
    return t * METRES_PER_TICK;
}

float encoder_get_avg_distance_m() {
    return (encoder_get_right_distance_m() + encoder_get_left_distance_m()) / 2.0f;
}

float get_robot_x() { return robot_x_m; }
float get_robot_y() { return robot_y_m; }

// =============================================
//  SECTION 6 — MOTOR CONTROL FUNCTIONS
// =============================================

static int clamp(int value, int min_val, int max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

void motors_init() {
    ledcAttach(RIGHT_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(RIGHT_RPWM, 0);
    ledcAttach(RIGHT_LPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(RIGHT_LPWM, 0);
    ledcAttach(LEFT_RPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(LEFT_RPWM, 0);
    ledcAttach(LEFT_LPWM, PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(LEFT_LPWM, 0);
}

void motor_right_set(int speed) {
    speed = clamp(speed, -MAX_PWM, MAX_PWM);
    if (speed > 0 && speed < MIN_PWM)  speed = MIN_PWM;
    if (speed < 0 && speed > -MIN_PWM) speed = -MIN_PWM;

    if (speed > 0) {
        ledcWrite(RIGHT_RPWM, speed);
        ledcWrite(RIGHT_LPWM, 0);
    } else if (speed < 0) {
        ledcWrite(RIGHT_RPWM, 0);
        ledcWrite(RIGHT_LPWM, -speed);
    } else {
        ledcWrite(RIGHT_RPWM, 0);
        ledcWrite(RIGHT_LPWM, 0);
    }
}

void motor_left_set(int speed) {
    speed = clamp(speed, -MAX_PWM, MAX_PWM);
    if (speed > 0 && speed < MIN_PWM)  speed = MIN_PWM;
    if (speed < 0 && speed > -MIN_PWM) speed = -MIN_PWM;

    if (speed > 0) {
        ledcWrite(LEFT_RPWM, speed);
        ledcWrite(LEFT_LPWM, 0);
    } else if (speed < 0) {
        ledcWrite(LEFT_RPWM, 0);
        ledcWrite(LEFT_LPWM, -speed);
    } else {
        ledcWrite(LEFT_RPWM, 0);
        ledcWrite(LEFT_LPWM, 0);
    }
}

void motors_set(int right, int left) {
    motor_right_set(right);
    motor_left_set(left);
}

void motors_stop() {
    ledcWrite(RIGHT_RPWM, 0);
    ledcWrite(RIGHT_LPWM, 0);
    ledcWrite(LEFT_RPWM,  0);
    ledcWrite(LEFT_LPWM,  0);
}

// =============================================
//  SECTION 7 — PID CLASS
// =============================================

class PID {
public:
    PID(float kp, float ki, float kd, float out_min, float out_max)
        : _kp(kp), _ki(ki), _kd(kd),
          _out_min(out_min), _out_max(out_max),
          _integral(0.0f), _last_error(0.0f), _last_output(0.0f) {}

    float compute(float setpoint, float measured, float dt) {
        if (dt <= 0.0f) return _last_output;

        float error  = setpoint - measured;
        float p_term = _kp * error;

        _integral += error * dt;
        float i_clamp = _out_max / (_ki > 0.0f ? _ki : 1.0f);
        if (_integral >  i_clamp) _integral =  i_clamp;
        if (_integral < -i_clamp) _integral = -i_clamp;
        float i_term = _ki * _integral;

        float derivative = (error - _last_error) / dt;
        float d_term     = _kd * derivative;

        float output = p_term + i_term + d_term;
        if (output >  _out_max) output =  _out_max;
        if (output < -_out_max) output = -_out_max;

        _last_error  = error;
        _last_output = output;
        return output;
    }

    void reset() {
        _integral    = 0.0f;
        _last_error  = 0.0f;
        _last_output = 0.0f;
    }

private:
    float _kp, _ki, _kd;
    float _out_min, _out_max;
    float _integral;
    float _last_error;
    float _last_output;
};

static PID pid_distance(DIST_PID_KP, DIST_PID_KI, DIST_PID_KD, -MAX_PWM, MAX_PWM);
static PID pid_heading(HEAD_PID_KP, HEAD_PID_KI, HEAD_PID_KD, -MAX_HEAD_CORRECTION, MAX_HEAD_CORRECTION);

// =============================================
//  SECTION 8 — ULTRASONIC FUNCTIONS
// =============================================

static float    us_left_cm  = 400.0f;
static float    us_mid_cm   = 400.0f;
static float    us_right_cm = 400.0f;
static uint32_t last_us_ms  = 0;

static float measure_cm(int trig_pin, int echo_pin) {
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);

    uint32_t duration_us = pulseIn(echo_pin, HIGH, TIMEOUT_US);
    if (duration_us == 0) return 400.0f;

    float dist = duration_us / 58.0f;
    if (dist <   2.0f) return   2.0f;
    if (dist > 400.0f) return 400.0f;
    return dist;
}

void ultrasonics_init() {
    pinMode(LEFT_TRIG,  OUTPUT); digitalWrite(LEFT_TRIG,  LOW);
    pinMode(MID_TRIG,   OUTPUT); digitalWrite(MID_TRIG,   LOW);
    pinMode(RIGHT_TRIG, OUTPUT); digitalWrite(RIGHT_TRIG, LOW);
    pinMode(LEFT_ECHO,  INPUT);
    pinMode(MID_ECHO,   INPUT);
    pinMode(RIGHT_ECHO, INPUT);
    last_us_ms = millis();
}

void ultrasonics_update() {
    if (millis() - last_us_ms < US_UPDATE_MS) return;
    us_left_cm  = measure_cm(LEFT_TRIG,  LEFT_ECHO);
    delayMicroseconds(SENSOR_GAP_US);
    us_mid_cm   = measure_cm(MID_TRIG,   MID_ECHO);
    delayMicroseconds(SENSOR_GAP_US);
    us_right_cm = measure_cm(RIGHT_TRIG, RIGHT_ECHO);
    last_us_ms  = millis();
}

bool obstacle_detected() {
    return (us_left_cm  < OBSTACLE_CM ||
            us_mid_cm   < OBSTACLE_CM ||
            us_right_cm < OBSTACLE_CM);
}

// =============================================
//  SECTION 9 — IMU FUNCTIONS
// =============================================

static MPU9250  mpu;
static float    yaw_deg        = 0.0f;
static float    gyro_z_bias    = 0.0f;
static float    gyro_z_dps     = 0.0f;
static bool     imu_ok         = false;
static uint32_t last_imu_ms    = 0;

bool imu_init() {
    Wire.begin(IMU_SDA, IMU_SCL);
    Wire.setClock(400000);

    if (!mpu.setup(IMU_ADDR)) {
        Serial.println("[IMU] ERROR: MPU9250 not found!");
        return false;
    }
    Serial.println("[IMU] MPU9250 found OK.");

    Serial.println("[IMU] Calibrating gyro bias - keep robot still!");
    delay(500);

    float bias_sum = 0.0f;
    for (int i = 0; i < IMU_CALIB_SAMPLES; i++) {
        mpu.update();
        bias_sum += mpu.getGyroZ();
        delay(5);
    }
    Serial.println();

    gyro_z_bias = bias_sum / IMU_CALIB_SAMPLES;
    Serial.print("[IMU] Gyro Z bias = ");
    Serial.print(gyro_z_bias, 5);
    Serial.println(" deg/s");

    yaw_deg = 0.0f;
    last_imu_ms = millis();
    imu_ok = true;
    return true;
}

void imu_update() {
    if (!imu_ok) return;

    uint32_t now   = millis();
    uint32_t dt_ms = now - last_imu_ms;
    if (dt_ms < IMU_UPDATE_MS) return;

    mpu.update();
    gyro_z_dps  = mpu.getGyroZ() - gyro_z_bias;
    yaw_deg    += gyro_z_dps * (dt_ms / 1000.0f);
    
    last_imu_ms = now;
}

float get_yaw() { return yaw_deg; }

// =============================================
//  SECTION 10 — NAVIGATION FUNCTIONS
// =============================================

static uint32_t last_pid_ms = 0;
static float target_heading_phase = 0.0f;

float angle_difference(float target, float current) {
    float diff = target - current;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;
    return diff;
}

float get_heading_for_direction(float distance, char axis) {
    if (axis == 'X') {
        if (distance >= 0) return 0.0f;
        else return 180.0f;
    } 
    else if (axis == 'Y') {
        if (distance >= 0) return 90.0f;
        else return -90.0f;
    }
    return 0.0f;
}

// =============================================
//  SECTION 11 — MANUAL MODE FUNCTIONS
// =============================================

void save_target_state() {
    Serial.println("[MANUAL] Saving original target...");
    
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        saved_target_state.target_x_m = shared_navigation.target_x_m;
        saved_target_state.target_y_m = shared_navigation.target_y_m;
        saved_target_state.final_yaw_deg = shared_navigation.final_yaw_deg;
        saved_target_state.need_initial_backward = shared_navigation.need_initial_backward;
        saved_target_state.need_final_forward = shared_navigation.need_final_forward;

        xSemaphoreGive(data_mutex);
    }
    
    saved_target_state.current_step = current_state;
    
    Serial.print("[MANUAL] Target saved: X=");
    Serial.print(saved_target_state.target_x_m, 3);
    Serial.print(" Y=");
    Serial.print(saved_target_state.target_y_m, 3);
    Serial.print(" | Step: ");
    Serial.println(saved_target_state.current_step);
}

void restart_navigation_from_current_position() {
    Serial.println("[MANUAL] Restarting navigation from current position to original target...");
    
    float current_x = -1 * get_robot_x();
    float current_y = -1 * get_robot_y();
    float target_x = saved_target_state.target_x_m;
    float target_y = saved_target_state.target_y_m;
    float final_yaw = saved_target_state.final_yaw_deg;
    
    float delta_x = target_x - current_x;
    float delta_y = target_y - current_y;
    /*
    Serial.print("[NAV] Starting from NEW position: X=");
    Serial.print(current_x, 3);
    Serial.print(" Y=");
    Serial.print(current_y, 3);
    Serial.print(" | Target: X=");
    Serial.print(target_x, 3);
    Serial.print(" Y=");
    Serial.print(target_y, 3);
    Serial.print(" | Delta to move: X=");
    Serial.print(delta_x, 3);
    Serial.print(" Y=");
    Serial.println(delta_y, 3);
    */
    float heading_x = get_heading_for_direction(delta_x, 'X');
    float heading_y = get_heading_for_direction(delta_y, 'Y');
    float abs_dist_x = fabs(delta_x);
    float abs_dist_y = fabs(delta_y);
    
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        shared_navigation.target_x_m = target_x;
        shared_navigation.target_y_m = target_y;
        shared_navigation.final_yaw_deg = final_yaw;
        shared_navigation.target_heading_for_x = heading_x;
        shared_navigation.target_heading_for_y = heading_y;
        shared_navigation.distance_to_move_x = abs_dist_x;
        shared_navigation.distance_to_move_y = abs_dist_y;
        shared_navigation.final_move_distance = FINAL_MOVE_DISTANCE_M;
        shared_navigation.need_initial_backward = saved_target_state.need_initial_backward;
        shared_navigation.need_final_forward = saved_target_state.need_final_forward;
        shared_navigation.stepper_activated = false;
        
        if (shared_navigation.need_initial_backward) {
            shared_navigation.phase = NAV_INITIAL_BACKWARD_MOVE;
        } else {
            if (abs_dist_x > 0.01f) {
                shared_navigation.phase = NAV_ROTATE_FOR_X;
            } 
            else if (abs_dist_y > 0.01f) {
                shared_navigation.phase = NAV_ROTATE_FOR_Y;
            }
            else {
                shared_navigation.phase = NAV_ROTATE_TO_FINAL;
            }
        }
        xSemaphoreGive(data_mutex);
    }
    
    pid_distance.reset();
    pid_heading.reset();
    encoders_reset();
    target_heading_phase = get_yaw();
    
    current_state = saved_target_state.current_step;
    
    Serial.println("[NAV] Navigation restarted from current position!");
    Serial.print("  Will move: X=");
    Serial.print(abs_dist_x, 3);
    Serial.print("m | Y=");
    Serial.print(abs_dist_y, 3);
    Serial.println("m");
}

void exit_manual_to_automatic() {
    Serial.println("[MANUAL] Command: EXIT TO AUTOMATIC MODE");
    motors_stop();
    
    manual_mode_active = false;
    last_manual_command_time = 0;
    if (saved_target_state.current_step == STATE_H_TO_T1_STEPPER || saved_target_state.current_step == STATE_H_TO_T2_STEPPER || saved_target_state.current_step == STATE_K_TO_H)
    {
        homeZ(130);
    } else {
        homeZ(60);
    }

    if (saved_target_state.current_step < 50) {
        Serial.println("[MANUAL] Restarting navigation from current position to original target...");
        restart_navigation_from_current_position();
        
        // Publish appropriate state based on current sequence
      /*  switch(saved_target_state.current_step) {
            case 11: publish_state(STATE_H_TO_T1); break;
            case 12: publish_state(STATE_T1_TO_H_FIRST); break;
            case 24: publish_state(STATE_H_TO_T2); break;
            case 25: publish_state(STATE_T2_TO_H_FIRST); break;
            case 13: publish_state(STATE_H_TO_K); break;
            case 14: publish_state(STATE_K_TO_H); break;
            case 15: publish_state(STATE_H_TO_T1_STEPPER); break;
            case 16: publish_state(STATE_T1_TO_H_FINAL); break;
            case 26: publish_state(STATE_H_TO_T2_STEPPER); break;
            case 27: publish_state(STATE_T2_TO_H_FINAL); break;
            default: publish_state(STATE_IDLE); break;
        }*/
    } else {
        Serial.println("[MANUAL] No saved target. Returning to IDLE state.");
    }
}

void enter_manual_mode() {
    if (!manual_mode_active) {
        Serial.println("[MANUAL] ENTERING MANUAL MODE");
        if (current_state != STATE_IDLE && current_state != STATE_COMPLETE && current_state != STATE_MANUAL) {
            save_target_state();
        }
        manual_mode_active = true;
        motors_stop();
    }
}

void execute_manual_command(char cmd) {
    switch(cmd) {
        case 'F':
           // Serial.println("[MANUAL] Command: FORWARD");
            motors_set(MANUAL_SPEED, MANUAL_SPEED);
            break;
        case 'B':
            //Serial.println("[MANUAL] Command: BACKWARD");
            motors_set(-MANUAL_SPEED, -MANUAL_SPEED);
            break;
        case 'L':
            //Serial.println("[MANUAL] Command: TURN LEFT");
            motors_set(-MANUAL_SPEED, MANUAL_SPEED);
            break;
        case 'R':
            //Serial.println("[MANUAL] Command: TURN RIGHT");
            motors_set(MANUAL_SPEED, -MANUAL_SPEED);
            break;
        case 'S':
            //Serial.println("[MANUAL] Command: STOP");
            motors_stop();
            break;
        case 'U':
            //Serial.println("[MANUAL] Command: STEPPER UP");
            motors_stop();
            activate_stepper_up(20.0);
            cmd = 'S';
            break;
        case 'D':
            //Serial.println("[MANUAL] Command: STEPPER DOWN");
            motors_stop();
            activate_stepper_down(20.0);
            cmd = 'S';
            break;
        case 'E':
            exit_manual_to_automatic();
            break;
        default:
            //Serial.print("[MANUAL] Unknown command: ");
            Serial.println(cmd);
            break;
    }
}

// =============================================
//  SECTION 12 — OBSTACLE AVOIDANCE FUNCTIONS
// =============================================

int get_best_avoidance_direction() {
    float left_dist = 400.0f;
    float right_dist = 400.0f;
    
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        left_dist = shared_ultrasonic.left_cm;
        right_dist = shared_ultrasonic.right_cm;
        xSemaphoreGive(data_mutex);
    }
    
    delay(100);
    ultrasonics_update();
    
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        left_dist = shared_ultrasonic.left_cm;
        right_dist = shared_ultrasonic.right_cm;
        xSemaphoreGive(data_mutex);
    }
    
    /*Serial.print("[AVOID] Left distance: ");
    Serial.print(left_dist, 1);
    Serial.print(" cm | Right distance: ");
    Serial.print(right_dist, 1);
    Serial.println(" cm");*/
    
    if (left_dist > right_dist && left_dist > OBSTACLE_CM) {
       // Serial.println("[AVOID] Choosing LEFT direction");
        return -1;
    } else if (right_dist > left_dist && right_dist > OBSTACLE_CM) {
        //Serial.println("[AVOID] Choosing RIGHT direction");
        return 1;
    } else {
        //Serial.println("[AVOID] Both directions similar, defaulting to RIGHT");
        return 1;
    }
}

void execute_obstacle_avoidance() {
   /* Serial.println("\n========================================");
    Serial.println("[AVOID] Starting obstacle avoidance!");
    Serial.println("========================================");*/
    
    motors_stop();
    delay(500);
    
    float start_x = -get_robot_x();
    float start_y = -get_robot_y();
   // Serial.print("[AVOID] Position before avoidance - X: ");
    Serial.print(start_x, 3);
    Serial.print(" Y: ");
    Serial.println(start_y, 3);
    
    int direction = get_best_avoidance_direction();
    avoidance_data.selected_direction = direction;
    
    float current_yaw = get_yaw();
    float avoid_heading;
    
    if (direction == -1) {
        avoid_heading = current_yaw + 90.0f;
        //Serial.println("[AVOID] Turning LEFT 90 degrees");
    } else {
        avoid_heading = current_yaw - 90.0f;
        //Serial.println("[AVOID] Turning RIGHT 90 degrees");
    }
    
    if (avoid_heading > 180.0f) avoid_heading -= 360.0f;
    if (avoid_heading < -180.0f) avoid_heading += 360.0f;
    
    Serial.print("[AVOID] Rotating to heading: ");
    Serial.println(avoid_heading, 1);
    
    pid_heading.reset();
    
    while (!is_rotation_complete(avoid_heading)) {
        imu_update();
        rotate_to_heading(avoid_heading);
        encoders_update();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    motors_stop();
    delay(200);
    
    Serial.print("[AVOID] Moving ");
    Serial.print(OBSTACLE_AVOID_DISTANCE_M * 100);
    Serial.println(" cm in chosen direction");
    
    encoders_reset_phase();
    pid_distance.reset();
    pid_heading.reset();

    float original_target_heading = target_heading_phase;
    target_heading_phase = avoid_heading;
    
    float target_distance = OBSTACLE_AVOID_DISTANCE_M;
    bool movement_complete = false;
    
    while (!movement_complete) {
        encoders_update();
        imu_update();
        move_forward(target_distance);
        
        float traveled = fabs(encoder_get_avg_distance_m());
        if (traveled >= (target_distance - POSITION_TOLERANCE_M)) {
            movement_complete = true;
        }
        
        ultrasonics_update();
        bool new_obstacle = false;
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            new_obstacle = shared_ultrasonic.obstacle_detected;
            xSemaphoreGive(data_mutex);
        }
        
        if (new_obstacle && traveled > 0.2f) {
            Serial.println("[AVOID] New obstacle during avoidance! Stopping early...");
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    motors_stop();
    vTaskDelay(pdMS_TO_TICKS(10));

    //Serial.print("[AVOID] Returning to original heading: ");
    //Serial.println(avoidance_data.original_yaw, 1);
    
    pid_heading.reset();
    
    while (!is_rotation_complete(avoidance_data.original_yaw)) {
        imu_update();
        rotate_to_heading(avoidance_data.original_yaw);
        encoders_update();
    }
    motors_stop();
    
    target_heading_phase = original_target_heading;
    
    avoidance_data.avoidance_complete = true;
    avoidance_data.avoidance_attempts++;
    
    Serial.println("[AVOID] Obstacle avoidance complete!");
    Serial.println("========================================\n");
}

// =============================================
//  SECTION 13 — SEQUENCE MANAGEMENT
// =============================================

enum SequenceStep {
    STEP_H_TO_T1 = 11,
    STEP_T1_TO_H_FIRST = 12,
    STEP_H_TO_K = 13,
    STEP_K_TO_H = 14,
    STEP_H_TO_T1_WITH_STEPPER = 15,
    STEP_T1_TO_H_FINAL = 16,
    STEP_H_TO_T2 = 41,
    STEP_T2_TO_H_FIRST = 42,
    STEP_H_TO_T2_WITH_STEPPER = 45,
    STEP_T2_TO_H_FINAL = 46,
    STEP_COMPLETE = 17
};

void publish_state(int state) {
    if (!ros_ready || publish_queue == NULL) return;
    int32_t s = (int32_t)state;
    xQueueSend(publish_queue, &s, 0);
}

void start_sequence_1() {
    Serial.println("\n========================================");
    Serial.println("SEQUENCE 1: Table 1 without stepper");
    Serial.println("  H → T1(1.5,-0.5) → H");
    Serial.println("========================================");
    table1_detected = true;
    current_sequence = SEQ_1;
    publish_state(STATE_H_TO_T1);
    current_state = STATE_H_TO_T1;
    start_navigation(TABLE1_X, TABLE1_Y, 0, false, false);
}

void start_sequence_2() {
    Serial.println("\n========================================");
    Serial.println("SEQUENCE 2: Table 2 without stepper");
    Serial.println("  H → T2(1.5,0.5) → H");
    Serial.println("========================================");
    table2_detected = true;
    current_sequence = SEQ_2;
    publish_state(STATE_H_TO_T2);
    current_state = STATE_H_TO_T2;
    start_navigation(TABLE2_X, TABLE2_Y, 0, false, false);
}

void start_sequence_3() {
    publish_state(13);
    Serial.println("\n========================================");
    Serial.println("SEQUENCE 3: Table K with stepper (after Table 1)");
    Serial.println("  H → K(0,1.5) → H → T1(1.5,-0.5) with stepper → H");
    Serial.println("========================================");
    current_sequence = SEQ_3;
    publish_state(STATE_H_TO_K);
    current_state = STATE_H_TO_K;
    start_navigation(0, 1.0, 90, false, true);
}

void start_sequence_4() {
    publish_state(13);
    Serial.println("\n========================================");
    Serial.println("SEQUENCE 4: Table K with stepper (after Table 2)");
    Serial.println("  H → K(0,1.5) → H → T2(1.5,0.5) with stepper → H");
    Serial.println("========================================");
    current_sequence = SEQ_4;
    publish_state(STATE_H_TO_K);
    current_state = STATE_H_TO_K;
    start_navigation(0, 1.0, 90, false, true);
}

void start_navigation(float target_x_m, float target_y_m, float final_yaw_deg, 
                      bool need_initial_backward, bool need_final_forward) {
    float current_x = -1 * get_robot_x();
    float current_y = -1 * get_robot_y();
    
    float delta_x = target_x_m - current_x;
    float delta_y = target_y_m - current_y;
    
    float heading_x = get_heading_for_direction(delta_x, 'X');
    float heading_y = get_heading_for_direction(delta_y, 'Y');
    
    float abs_dist_x = fabs(delta_x);
    float abs_dist_y = fabs(delta_y);
    
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        shared_navigation.target_x_m = target_x_m;
        shared_navigation.target_y_m = target_y_m;
        shared_navigation.final_yaw_deg = final_yaw_deg;
        shared_navigation.target_heading_for_x = heading_x;
        shared_navigation.target_heading_for_y = heading_y;
        shared_navigation.distance_to_move_x = abs_dist_x;
        shared_navigation.distance_to_move_y = abs_dist_y;
        shared_navigation.final_move_distance = FINAL_MOVE_DISTANCE_M;
        
        if (need_initial_backward) {
            shared_navigation.phase = NAV_INITIAL_BACKWARD_MOVE;
            Serial.println("\n===========");
            Serial.println(shared_navigation.phase);
            
        } else {
            if (abs_dist_x > 0.01f) {
                shared_navigation.phase = NAV_ROTATE_FOR_X;
            } 
            else if (abs_dist_y > 0.01f) {
                shared_navigation.phase = NAV_ROTATE_FOR_Y;
            }
            else {
                shared_navigation.phase = NAV_ROTATE_TO_FINAL;
            }
        }
        
        shared_navigation.need_initial_backward = need_initial_backward;
        shared_navigation.need_final_forward = need_final_forward;
        shared_navigation.movement_complete = false;
        shared_navigation.stepper_activated = false;
        xSemaphoreGive(data_mutex);
    }
    
    pid_distance.reset();
    pid_heading.reset();
    encoders_reset();
}

void start_next_sequence_step() {
    switch(current_sequence) {
        case SEQ_1:
            if (current_state == STATE_H_TO_T1) {
                // After H→T1, wait for continue signal
                publish_state(18);
                publish_state(STATE_WAITING);
                current_state = STATE_WAITING;
                Serial.println("\n========== H→T1 COMPLETE ==========");
                Serial.println("Send 5 (from /screen) to continue to T1→H");
                Serial.println("=====================================\n");
                //delay(500);
                publish_state(3);
            } else if (current_state == STATE_T1_TO_H_FIRST) {
                // Sequence 1 complete
                seq1_completed = true;
                table1_detected = true;
                current_sequence = SEQ_NONE;
                publish_state(18);
                //publish_state(STATE_COMPLETE);
                current_state = STATE_COMPLETE;
                Serial.println("\n========== SEQUENCE 1 COMPLETE! ==========");
                Serial.println("Table 1 marked as detected.");
                Serial.println("===========================================\n");
                for (int i =0 ; i<5 ; i++)
                {
                  publish_state(READ);
                }
            }
            break;
            
        case SEQ_2:
            if (current_state == STATE_H_TO_T2) {
                publish_state(18);
                publish_state(STATE_WAITING);
                current_state = STATE_WAITING;
                Serial.println("\n========== H→T2 COMPLETE ==========");
                Serial.println("Send 5 (from /screen) to continue to T2→H");
                Serial.println("=====================================\n");
                //delay(500);
                publish_state(3);
            } else if (current_state == STATE_T2_TO_H_FIRST) {
                seq2_completed = true;
                table2_detected = true;
                current_sequence = SEQ_NONE;
                publish_state(18);
                //publish_state(STATE_COMPLETE);
                current_state = STATE_COMPLETE;
                Serial.println("\n========== SEQUENCE 2 COMPLETE! ==========");
                Serial.println("Table 2 marked as detected.");
                Serial.println("===========================================\n");
                for (int i =0 ; i<5 ; i++)
                {
                   publish_state(READ);
                }
            }
            break;
            
        case SEQ_3:
            if (current_state == STATE_H_TO_K) {
                // After H→K, continue automatically to K→H
                publish_state(STATE_K_TO_H);
                current_state = STATE_K_TO_H;
                Serial.println("===========================================\n");
                start_navigation(0, 0, 0, true, false);
            } else if (current_state == STATE_K_TO_H) {
                // After K→H, continue to H→T1 with stepper
                publish_state(STATE_H_TO_T1_STEPPER);
                current_state = STATE_H_TO_T1_STEPPER;
                start_navigation(TABLE1_X, TABLE1_Y, 0, false, true);
            } else if (current_state == STATE_H_TO_T1_STEPPER) {
                // After stepper operation, wait for continue signal
                publish_state(STATE_WAITING);
                current_state = STATE_WAITING;
                Serial.println("\n========== STEPPER DOWN COMPLETE at T1 ==========");
                Serial.println("Send 6 (from /screen) to continue to final home (H)");
                Serial.println("===================================================\n");
            } else if (current_state == STATE_T1_TO_H_FINAL) {
                // Sequence 3 complete
                publish_state(18);
                //delay(500);
                current_sequence = SEQ_NONE;
                publish_state(20);
                current_state = STATE_COMPLETE;
                publish_state(STATE_COMPLETE);
                table1_finish = true;
                //delay(500);
                publish_state(18);
                Serial.println("\n========== SEQUENCE 3 COMPLETE! ==========");
                Serial.println("All Table 1 operations finished.");
                Serial.println("===========================================\n");
                for (int i =0 ; i<5 ; i++)
                {
                  publish_state(READ);
                }
                
            }
            break;
            
        case SEQ_4:
            if (current_state == STATE_H_TO_K) {
                current_state = STATE_K_TO_H;
                publish_state(STATE_K_TO_H);
                
                start_navigation(0, 0, 0, true, false);
            } else if (current_state == STATE_K_TO_H) {
                publish_state(STATE_H_TO_T2_STEPPER);
                current_state = STATE_H_TO_T2_STEPPER;
                start_navigation(TABLE2_X, TABLE2_Y, 0, false, true);
            } else if (current_state == STATE_H_TO_T2_STEPPER) {
                publish_state(STATE_WAITING);
                current_state = STATE_WAITING;
                Serial.println("\n========== STEPPER DOWN COMPLETE at T2 ==========");
                Serial.println("Send 6 (from /screen) to continue to final home (H)");
                Serial.println("===================================================\n");
            } else if (current_state == STATE_T2_TO_H_FINAL) {
                current_sequence = SEQ_NONE;
                publish_state(18);
                //delay(500);
                publish_state(20);
                current_state = STATE_COMPLETE;
                publish_state(STATE_COMPLETE);
                table2_finish = true;
                //delay(500);
                publish_state(18);
                Serial.println("\n========== SEQUENCE 4 COMPLETE! ==========");
                Serial.println("All Table 2 operations finished.");
                Serial.println("===========================================\n");
                for (int i =0 ; i<5 ; i++)
                {
                  publish_state(READ);
                }
            }
            break;
    }
}

// =============================================
//  SECTION 14 — MOVEMENT FUNCTIONS
// =============================================

void move_forward(float target_distance) {
    uint32_t now = millis();
    uint32_t dt_ms = now - last_pid_ms;
    
    if (dt_ms < PID_INTERVAL_MS) return;
    
    float dt = dt_ms / 1000.0f;
    float current_distance = -1 * encoder_get_avg_distance_m();
    
    float base_pwm = pid_distance.compute(target_distance, current_distance, dt);
    base_pwm = constrain(base_pwm, MIN_PWM, MAX_PWM);
    
    float current_yaw = get_yaw();
    
    float heading_correction = pid_heading.compute(target_heading_phase, current_yaw, dt);
    heading_correction = constrain(heading_correction, -MAX_HEAD_CORRECTION, MAX_HEAD_CORRECTION);
    
    int left_pwm = (int)base_pwm - (int)heading_correction;
    int right_pwm = (int)base_pwm + (int)heading_correction;
    
    motors_set(right_pwm, left_pwm);
    last_pid_ms = now;
}

void move_backward_distance(float target_distance) {
    uint32_t now = millis();
    uint32_t dt_ms = now - last_pid_ms;
    
    if (dt_ms < PID_INTERVAL_MS) return;
    
    float dt = dt_ms / 1000.0f;
    float current_distance = encoder_get_avg_distance_m();
    
    float base_pwm = pid_distance.compute(target_distance, current_distance, dt);
    base_pwm = constrain(base_pwm, MIN_PWM, MAX_PWM);
    
    float current_yaw = get_yaw();
    float heading_correction = pid_heading.compute(target_heading_phase, current_yaw, dt);
    heading_correction = constrain(heading_correction, -MAX_HEAD_CORRECTION, MAX_HEAD_CORRECTION);
    
    int left_pwm = -(int)base_pwm - (int)heading_correction;
    int right_pwm = -(int)base_pwm + (int)heading_correction;
    
    motors_set(right_pwm, left_pwm);
    last_pid_ms = now;
}

void move_forward_distance(float target_distance) {
    uint32_t now = millis();
    uint32_t dt_ms = now - last_pid_ms;
    
    if (dt_ms < PID_INTERVAL_MS) return;
    
    float dt = dt_ms / 1000.0f;
    float current_distance = -1 * encoder_get_avg_distance_m();
    
    float base_pwm = pid_distance.compute(target_distance, current_distance, dt);
    base_pwm = constrain(base_pwm, MIN_PWM, MAX_PWM);
    
    float current_yaw = get_yaw();
    float heading_correction = pid_heading.compute(target_heading_phase, current_yaw, dt);
    heading_correction = constrain(heading_correction, -MAX_HEAD_CORRECTION, MAX_HEAD_CORRECTION);
    
    int left_pwm = (int)base_pwm - (int)heading_correction;
    int right_pwm = (int)base_pwm + (int)heading_correction;
    
    motors_set(right_pwm, left_pwm);
    last_pid_ms = now;
}

void rotate_to_heading(float target_heading) {
    uint32_t now = millis();
    uint32_t dt_ms = now - last_pid_ms;
    
    if (dt_ms < PID_INTERVAL_MS) return;
    
    float dt = dt_ms / 1000.0f;
    float current_yaw = get_yaw();
   
    float rotation_pwm = pid_heading.compute(target_heading, current_yaw, dt);
    rotation_pwm = 2*constrain(rotation_pwm, -MAX_HEAD_CORRECTION, MAX_HEAD_CORRECTION);
    motors_set(rotation_pwm, -rotation_pwm);
    last_pid_ms = now;
}

bool is_rotation_complete(float target_heading) {
    float error = fabs(angle_difference(target_heading, get_yaw()));
    return (error <= HEADING_TOLERANCE);
}

// =============================================
//  SECTION 15 — STEPPER FUNCTIONS
// =============================================

void STEPPER_init() {
    pinMode(DIR_Z, OUTPUT);
    pinMode(STEP_Z, OUTPUT);
    pinMode(LIMIT_Z_HOME, INPUT_PULLUP);
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
    homeZ_blocking();
    Serial.println("Starting homing sequence...");
    Serial.println("Homing completed.");
}

unsigned long stepsZ(float mm) {
    float stepsPerMM = (Z_motorSteps * Z_microsteps) / Z_lead;
    return (unsigned long)(stepsPerMM * mm);
}

void singleStep() {
    digitalWrite(STEP_Z, HIGH);
    delayMicroseconds(Z_stepDelay);
    digitalWrite(STEP_Z, LOW);
    delayMicroseconds(Z_stepDelay);
}

void moveZ(float mm, bool dir) {
    unsigned long st = stepsZ(mm);
    digitalWrite(DIR_Z, dir ? HIGH : LOW);
    for(unsigned long i = 0; i < st; i++) {
        while(digitalRead(EMERGENCY_STOP_PIN) == LOW) {}
        singleStep();
        if (i % 100 == 0) {
            vTaskDelay(1);  // yields to FreeRTOS, resets watchdog
        }
    }
}
void homeZ(float d) {
    // For use in tasks — FreeRTOS-safe
    Serial.println("Homing Z...");
    digitalWrite(DIR_Z, LOW);
    unsigned long step_count = 0;
    while(digitalRead(LIMIT_Z_HOME) == LOW) {
        singleStep();
        if (++step_count % 100 == 0) vTaskDelay(1);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    moveZ(d , true);
    Serial.println("Z homed.");
}
void homeZ_blocking() {
    Serial.println("Homing Z...");
    digitalWrite(DIR_Z, LOW);
    while(digitalRead(LIMIT_Z_HOME) == LOW) {
        singleStep();
    }
    delay(200);
    moveZ(60.0, true);
    Serial.println("Z homed.");
}

void activate_stepper_up(float d ) {
    Serial.println(">> Activating stepper UP 7cm at point K...");
    publish_state(STATE_STEPPER_UP);
    moveZ(d , true);
    //delay(1000);
    Serial.println("Stepper UP complete.");
}

void activate_stepper_down(float d) {
    Serial.println(">> Activating stepper DOWN 7cm at point...");
    publish_state(STATE_STEPPER_DOWN);
    moveZ(d, false);
    //delay(1000);
    Serial.println("Stepper DOWN complete.");
}

// =============================================
//  SECTION 16 — ULTRASONIC TASK
// =============================================

void taskUltrasonicDetection(void *pvParameters) {
    Serial.print("Ultrasonic Detection Task running on core ");
    Serial.println(xPortGetCoreID());
    
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(US_UPDATE_MS);
    
    while(1) {
        ultrasonics_update();
        
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            shared_ultrasonic.left_cm = us_left_cm;
            shared_ultrasonic.mid_cm = us_mid_cm;
            shared_ultrasonic.right_cm = us_right_cm;
            shared_ultrasonic.obstacle_detected = obstacle_detected();
            shared_ultrasonic.timestamp = millis();
            xSemaphoreGive(data_mutex);
        }
        
       /*static unsigned long last_print = 0;
        if (millis() - last_print > 1000) {
            Serial.print("[DETECTION] L:");
            Serial.print(us_left_cm, 1);
            Serial.print(" M:");
            Serial.print(us_mid_cm, 1);
            Serial.print(" R:");
            Serial.print(us_right_cm, 1);
            if (obstacle_detected()) {
                Serial.print(" ⚠️ OBSTACLE!");
            }
            if (manual_mode_active) {
                Serial.print(" [MANUAL MODE ACTIVE]");
            }
            Serial.println();
            last_print = millis();
        }*/
        
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

// =============================================
//  SECTION 17 — MICRO-ROS CALLBACKS
// =============================================

void ros_callback_detection(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    received_command = msg->data;
    
    if (msg->data == 1 && !manual_mode_active && !table1_detected && current_sequence == SEQ_NONE) {
        Serial.println("[ROS /detection] Received: START SEQUENCE 1 (Table 1)");
        start = true;
        start_sequence_1();
    } 
    else if (msg->data == 2 && !manual_mode_active && !table2_detected && current_sequence == SEQ_NONE) {
        Serial.println("[ROS /detection] Received: START SEQUENCE 2 (Table 2)");
        start = true ;
        start_sequence_2();
    }
    else if (msg->data == 1 && table1_detected) {
        Serial.println("[ROS /detection] Table 1 already detected! Ignoring command.");
    }
    else if (msg->data == 2 && table2_detected) {
        Serial.println("[ROS /detection] Table 2 already detected! Ignoring command.");
    }
}

void ros_callback_screen(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    
    if (msg->data == 5 && !manual_mode_active) {
        Serial.println("[ROS /screen] Received: CONTINUE (5)");
        
        if (current_sequence == SEQ_1 && current_state == STATE_WAITING) {
            
            publish_state(STATE_T1_TO_H_FIRST);
            current_state = STATE_T1_TO_H_FIRST;
            //delay(500);
            publish_state(12);
            start_navigation(0, 0, 0, false, false);
        }
        else if (current_sequence == SEQ_2 && current_state == STATE_WAITING) {
            publish_state(STATE_T2_TO_H_FIRST);
            current_state = STATE_T2_TO_H_FIRST;
            //delay(500);
            publish_state(42);
            start_navigation(0, 0, 0, false, false);
        }
    } 
    else if (msg->data == 6 && !manual_mode_active) {
        Serial.println("[ROS /screen] Received: AFTER STEPPER (6)");
        
        if (current_sequence == SEQ_3 && current_state == STATE_WAITING) {
            publish_state(15);
            //delay(500);
            publish_state(STATE_T1_TO_H_FINAL);
            current_state = STATE_T1_TO_H_FINAL;
            start_navigation(0, 0, 0, true, false);
        }
        else if (current_sequence == SEQ_4 && current_state == STATE_WAITING) {
            publish_state(45);
            //delay(500);
            publish_state(STATE_T2_TO_H_FINAL);
            current_state = STATE_T2_TO_H_FINAL;
            publish_state(46);
            start_navigation(0, 0, 0, true, false);
        }
    }
    else if (msg->data == 7 && !manual_mode_active) {
        Serial.println("[ROS /screen] Received: AFTER STEPPER (8)");
        
        if (current_sequence == SEQ_3 && current_state == STATE_WAITING) {
            publish_state(STATE_T1_TO_H_FIRST);
            current_state = STATE_T1_TO_H_FIRST;
            current_sequence = SEQ_1;
            //delay(500);
            publish_state(12);
            start_navigation(0, 0, 0, true, false);
        }
        else if (current_sequence == SEQ_4 && current_state == STATE_WAITING) {
            publish_state(STATE_T2_TO_H_FIRST);
            current_state = STATE_T2_TO_H_FIRST;
            current_sequence = SEQ_2;
            //delay(500);
            publish_state(42);
            start_navigation(0, 0, 0, true, false);
        }
    }
}

void ros_callback_gui(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    
    if (msg->data == 4 && !manual_mode_active && table1_detected && current_sequence == SEQ_NONE) {
        Serial.println("[ROS /gui_topic] Received: START SEQUENCE 3 (after Table 1)");
        start_sequence_3();
    }
    else if (msg->data == 7 && !manual_mode_active && table2_detected && current_sequence == SEQ_NONE) {
        Serial.println("[ROS /gui_topic] Received: START SEQUENCE 4 (after Table 2)");
        start_sequence_4();
    }

    if (msg->data == 5 && !manual_mode_active && table1_finish && current_sequence == SEQ_NONE) {
        Serial.println("[ROS /gui_topic] Received: START SEQUENCE  (clear Table 1)");
        table1_detected = false;
        table1_finish = false;
    }
    else if (msg->data == 8 && !manual_mode_active && table2_finish && current_sequence == SEQ_NONE) {
        Serial.println("[ROS /gui_topic] Received: START SEQUENCE (clear Table 2)");
        table2_detected=false;
        table2_finish = false;
    }

    else if (msg->data == 4 && !table1_detected) {
        Serial.println("[ROS /gui_topic] Cannot start Sequence 3 - Table 1 not detected yet!");
    }
    else if (msg->data == 7 && !table2_detected) {
        Serial.println("[ROS /gui_topic] Cannot start Sequence 4 - Table 2 not detected yet!");
    }
}

void ros_callback_manual(const void * msgin) {
    const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
    int cmd_value = msg->data;
    
    if (cmd_value == 8 && !manual_mode_active) {
        Serial.println("[ROS /manual_commands] Received: ENTER MANUAL MODE (8)");
        enter_manual_mode();
    }
    else if (manual_mode_active) {
        switch(cmd_value) {
            case 1:
                last_manual_command = 'F';
                last_manual_command_time = millis();
                execute_manual_command('F');
                break;
            case 2:
                last_manual_command = 'B';
                last_manual_command_time = millis();
                execute_manual_command('B');
                break;
            case 3:
                last_manual_command = 'R';
                last_manual_command_time = millis();
                execute_manual_command('L');
                break;
            case 4:
                last_manual_command = 'L';
                last_manual_command_time = millis();
                execute_manual_command('R');
                break;
            case 5:
                last_manual_command = 'U';
                last_manual_command_time = millis();
                execute_manual_command('U');
                break;
            case 6:
                last_manual_command = 'D';
                last_manual_command_time = millis();
                execute_manual_command('D');
                break;
            case 7:
                last_manual_command = 'S';
                last_manual_command_time = millis();
                execute_manual_command('S');
                break;
            case 9:
                exit_manual_to_automatic();
                break;
            default:
                Serial.print("[ROS /manual_commands] Unknown command value: ");
                Serial.println(cmd_value);
                break;
        }
    }
}

// =============================================
//  SECTION 18 — MICRO-ROS TASK
// =============================================

void taskMicroROS(void *pvParameters) {
    Serial.print("Micro-ROS Task running on core ");
    Serial.println(xPortGetCoreID());
    
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected!");
    
    delay(2000);
    
    set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
    delay(2000);
    
    allocator = rcl_get_default_allocator();
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_nav_node", "", &support);
    
    rclc_subscription_init_default(
        &subscriber_detection,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/detection"
    );
    
    rclc_subscription_init_default(
        &subscriber_screen,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/screen"
    );
    
    rclc_subscription_init_default(
        &subscriber_gui,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/gui_topic"
    );
    
    rclc_subscription_init_default(
        &subscriber_manual,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/manu"
    );
    
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/table"
    );
    
   
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    
    rclc_executor_add_subscription(
        &executor, 
        &subscriber_detection, 
        &msg_sub_detection, 
        &ros_callback_detection, 
        ON_NEW_DATA
    );
    
    rclc_executor_add_subscription(
        &executor, 
        &subscriber_screen, 
        &msg_sub_screen, 
        &ros_callback_screen, 
        ON_NEW_DATA
    );
    
    rclc_executor_add_subscription(
        &executor, 
        &subscriber_gui, 
        &msg_sub_gui, 
        &ros_callback_gui, 
        ON_NEW_DATA
    );
    
    rclc_executor_add_subscription(
        &executor, 
        &subscriber_manual, 
        &msg_sub_manual, 
        &ros_callback_manual, 
        ON_NEW_DATA
    );
    Serial.printf("[ROS] Stack HWM: %u words\n", uxTaskGetStackHighWaterMark(NULL));
    ros_ready = true;
    Serial.println("[ROS] Publisher ready."); 
    
    while(1) {
    // Drain publish queue (safe — same task as executor)
    int32_t state_to_pub;
    while (xQueueReceive(publish_queue, &state_to_pub, 0) == pdTRUE) {
        msg_pub.data = state_to_pub;
        rcl_publish(&publisher, &msg_pub, NULL);
        Serial.printf("[PUBLISH] State: %d\n", (int)state_to_pub);
    }
    if (!start)
    {
       publish_state(READ);

    }
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20));
    if (ret != RCL_RET_OK) {
        Serial.printf("[ROS] Executor spin error: %d\n", ret);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
}
}

// =============================================
//  SECTION 19 — NAVIGATION TASK
// =============================================

void taskNavigation(void *pvParameters) {
    Serial.print("Navigation Task running on core ");
    Serial.println(xPortGetCoreID());
    
    NavPhase phase = NAV_IDLE;
    uint32_t stop_start_ms = 0;
    bool need_x_movement = true;
    bool need_y_movement = true;
    bool need_initial_backward = false;
    bool need_final_forward = true;
    bool waiting_for_continue = false;
    
    while(1) {
        if (manual_mode_active) {
            encoders_update();
            imu_update();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        if (current_sequence == SEQ_NONE) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if(digitalRead(EMERGENCY_STOP_PIN) == LOW) {
            publish_state(30);
            encoders_update();
            imu_update();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        encoders_update();
        imu_update();
        
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            need_x_movement = (fabs(shared_navigation.distance_to_move_x) > 0.08f);
            need_y_movement = (fabs(shared_navigation.distance_to_move_y) > 0.08f);
            need_initial_backward = shared_navigation.need_initial_backward;
            need_final_forward = shared_navigation.need_final_forward;
            phase = shared_navigation.phase;
            xSemaphoreGive(data_mutex);
        }
       // Serial.println(phase);
        
        bool obstacle = false;
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            obstacle = shared_ultrasonic.obstacle_detected;
            xSemaphoreGive(data_mutex);
        }
        
        if (phase != NAV_INITIAL_BACKWARD_MOVE && phase != NAV_FINAL_MOVE_FORWARD && 
            phase != NAV_STOP && phase != NAV_DONE && phase != NAV_WAIT_STEPPER && phase != NAV_IDLE) {
            
            if (obstacle && !avoidance_data.is_avoiding) {
                motors_stop();
                
                if (!avoidance_data.waiting_for_clear) {
                    avoidance_data.waiting_for_clear = true;
                    avoidance_data.wait_start_time = millis();
                    avoidance_data.obstacle_start_time = millis();
                    
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        avoidance_data.original_target_x = shared_navigation.target_x_m;
                        avoidance_data.original_target_y = shared_navigation.target_y_m;
                        avoidance_data.original_yaw = get_yaw();
                        avoidance_data.original_distance_remaining = fabs(shared_navigation.distance_to_move_x);
                        if (phase == NAV_MOVE_Y) {
                            avoidance_data.original_distance_remaining = fabs(shared_navigation.distance_to_move_y);
                        }
                        xSemaphoreGive(data_mutex);
                    }
                    /*
                    Serial.println("\n========================================");
                    Serial.println("[OBSTACLE] Obstacle detected!");
                    Serial.print("[OBSTACLE] Will wait up to ");
                    Serial.print(OBSTACLE_WAIT_TIME_MS / 1000);
                    Serial.println(" seconds for clearance...");
                    Serial.println("========================================\n");*/
                }
                
                unsigned long wait_elapsed = millis() - avoidance_data.wait_start_time;
                
                if (wait_elapsed < OBSTACLE_WAIT_TIME_MS) {
                    ultrasonics_update();
                    publish_state(19);
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        obstacle = shared_ultrasonic.obstacle_detected;
                        xSemaphoreGive(data_mutex);
                    }
                    
                    if (!obstacle) {
                        Serial.println("\n[OBSTACLE] Obstacle cleared! Resuming movement immediately...");
                        avoidance_data.waiting_for_clear = false;
                        avoidance_data.is_avoiding = false;
                        motors_stop();
                        delay(500);
                        continue;
                    }
                    
                    vTaskDelay(pdMS_TO_TICKS(100));
                } else {
                    /*Serial.println("\n========================================");
                    Serial.println("[OBSTACLE] Wait time expired! Obstacle still present.");
                    Serial.println("[OBSTACLE] Initiating obstacle avoidance...");
                    Serial.println("========================================\n");*/
                    publish_state(22);
                    avoidance_data.waiting_for_clear = false;
                    avoidance_data.is_avoiding = true;
                    
                    if (avoidance_data.avoidance_attempts >= AVOIDANCE_RETURN_ATTEMPTS) {
                        Serial.println("[OBSTACLE] Max avoidance attempts reached!");
                        Serial.println("[OBSTACLE] Stopping sequence...");
                        motors_stop();
                        avoidance_data.is_avoiding = false;
                        avoidance_data.avoidance_attempts = 0;
                        break;
                    }

                    if (phase == NAV_MOVE_X) {
                        avoidance_data.current_axis = 'X';
                    } else if (phase == NAV_MOVE_Y) {
                        avoidance_data.current_axis = 'Y';
                    } else if (phase == NAV_FINAL_MOVE_FORWARD || phase == NAV_ROTATE_TO_FINAL) {
                        avoidance_data.current_axis = 'F';
                    }
                    
                    execute_obstacle_avoidance();
                    
                    if (avoidance_data.avoidance_complete) {
                        encoders_reset_phase();
                        pid_distance.reset();
                        pid_heading.reset();
                        //last_pid_ms = millis(); 
                        avoidance_data.is_avoiding = false;
                        avoidance_data.waiting_for_clear = false;
                        avoidance_data.avoidance_complete = false;
                        
                        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                            float current_x = - get_robot_x();
                            float current_y = - get_robot_y();
                            float target_x = shared_navigation.target_x_m;
                            float target_y = shared_navigation.target_y_m;
                            
                            float remaining_x = target_x - current_x;
                            float remaining_y = target_y - current_y;
                            avoidance_data.remaining_x = remaining_x;
                            avoidance_data.remaining_y = remaining_y;
                            
                            shared_navigation.distance_to_move_x = fabs(remaining_x);
                            shared_navigation.distance_to_move_y = fabs(remaining_y);
                            
                            if (fabs(remaining_x) > 0.05f) {
                                shared_navigation.target_heading_for_x = get_heading_for_direction(remaining_x, 'X');
                            }
                            if (fabs(remaining_y) > 0.05f) {
                                shared_navigation.target_heading_for_y = get_heading_for_direction(remaining_y, 'Y');
                            }
                            
                            if (avoidance_data.current_axis == 'Y') {
                                if (fabs(remaining_y) > 0.01f) {
                                    shared_navigation.phase = NAV_MOVE_Y;
                                    target_heading_phase = shared_navigation.target_heading_for_y;
                                    encoders_reset_phase();
                                } else if (fabs(remaining_x) > 0.01f) {
                                    shared_navigation.phase = NAV_ROTATE_FOR_X;
                                } else {
                                    shared_navigation.phase = NAV_ROTATE_TO_FINAL;
                                }
                            }
                            else if (avoidance_data.current_axis == 'X') {
                                if (fabs(remaining_x) > 0.01f) {
                                    shared_navigation.phase = NAV_MOVE_X;
                                    target_heading_phase = shared_navigation.target_heading_for_x;
                                    encoders_reset_phase();
                                } else if (fabs(remaining_y) > 0.01f) {
                                    shared_navigation.phase = NAV_ROTATE_FOR_Y;
                                } else {
                                    shared_navigation.phase = NAV_ROTATE_TO_FINAL;
                                }
                            }
                            else if (avoidance_data.current_axis == 'F') {
                                if (fabs(remaining_x) > 0.01f || fabs(remaining_y) > 0.01f) {
                                    if (fabs(remaining_x) > 0.01f) {
                                        shared_navigation.phase = NAV_ROTATE_FOR_X;
                                    } else if (fabs(remaining_y) > 0.01f) {
                                        shared_navigation.phase = NAV_ROTATE_FOR_Y;
                                    }
                                } else {
                                    shared_navigation.phase = NAV_ROTATE_TO_FINAL;
                                }
                            }
                            
                            phase = shared_navigation.phase;
                            xSemaphoreGive(data_mutex);
                          //  Serial.println(phase);
                        }
                        
                        Serial.println("[OBSTACLE] Avoidance complete! Continuing original navigation...");
                        motors_stop();
                        delay(500);
                    }
                }
            } else if (!obstacle && avoidance_data.waiting_for_clear) {
                avoidance_data.waiting_for_clear = false;
                avoidance_data.is_avoiding = false;
            }
        }
        
        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            need_x_movement = (fabs(shared_navigation.distance_to_move_x) > 0.08f);
            need_y_movement = (fabs(shared_navigation.distance_to_move_y) > 0.08f);
            need_initial_backward = shared_navigation.need_initial_backward;
            need_final_forward = shared_navigation.need_final_forward;
            phase = shared_navigation.phase;
            xSemaphoreGive(data_mutex);
            //Serial.println(phase);
        }
        //Serial.println(phase);
        switch(phase) {
            case NAV_INITIAL_BACKWARD_MOVE: {
                Serial.println("\n>> Performing initial BACKWARD 20cm move...");
                move_backward_distance(FINAL_MOVE_DISTANCE_M);
                
                float traveled = fabs(encoder_get_avg_distance_m());
                if (traveled >= (FINAL_MOVE_DISTANCE_M - POSITION_TOLERANCE_M)) {
                    motors_stop();
                    encoders_reset_phase();
                    
                    if (need_x_movement) {
                        phase = NAV_ROTATE_FOR_X;
                    } else if (need_y_movement) {
                        phase = NAV_ROTATE_FOR_Y;
                    } else {
                        phase = NAV_ROTATE_TO_FINAL;
                    }
                    
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.phase = phase;
                        xSemaphoreGive(data_mutex);
                    }
                    
                    Serial.println("\n>> Initial backward move complete.\n");
                }
                break;
            }
                
            case NAV_ROTATE_FOR_X: {
                if (!need_x_movement) {
                    if (need_y_movement) {
                        phase = NAV_ROTATE_FOR_Y;
                    } else {
                        phase = NAV_ROTATE_TO_FINAL;
                    }
                    
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.phase = phase;
                        xSemaphoreGive(data_mutex);
                    }
                    break;
                }
                float heading_x=0.0;
                if (!obstacle || avoidance_data.is_avoiding) {
                    
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        heading_x = shared_navigation.target_heading_for_x;
                        xSemaphoreGive(data_mutex);
                    }
                    rotate_to_heading(heading_x);
                }
                
                if (is_rotation_complete(heading_x)) {
                    motors_stop();
                    phase = NAV_MOVE_X;
                    encoders_reset_phase();
                    target_heading_phase =  heading_x;
                    pid_distance.reset();
                    pid_heading.reset();
                    
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.phase = NAV_MOVE_X;
                        xSemaphoreGive(data_mutex);
                    }
                }
                break;
            }
                
            case NAV_MOVE_X: {
                avoidance_data.current_axis = 'X'; 
                if ((!obstacle || avoidance_data.is_avoiding) && need_x_movement) {
                    float distance_x;
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        distance_x = shared_navigation.distance_to_move_x;
                        xSemaphoreGive(data_mutex);
                    }
                    move_forward(distance_x);
                }
                
                float traveled_x = fabs(encoder_get_avg_distance_m());
                float target_x_dist;
                if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                    target_x_dist = shared_navigation.distance_to_move_x;
                    xSemaphoreGive(data_mutex);
                }
                
                if (traveled_x >= fabs(target_x_dist - POSITION_TOLERANCE_M)) {
                    motors_stop();
                    avoidance_data.remaining_x = 0;

                    bool y_still_needed = false;
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.distance_to_move_x = 0.0f;
                        float current_y = -get_robot_y();
                        float remaining_y = shared_navigation.target_y_m - current_y;
                        shared_navigation.distance_to_move_y = fabs(remaining_y);
                        if (fabs(remaining_y) > 10*POSITION_TOLERANCE_M) {
                            shared_navigation.target_heading_for_y = get_heading_for_direction(remaining_y, 'Y');
                            y_still_needed = true;
                        }
                        xSemaphoreGive(data_mutex);
                    }

                    if (y_still_needed) {
                        phase = NAV_ROTATE_FOR_Y;
                    } else {
                        phase = NAV_ROTATE_TO_FINAL;
                    }
                    pid_heading.reset();

                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.phase = phase;
                        xSemaphoreGive(data_mutex);
                    }
                }
                break;
            }
                
            case NAV_ROTATE_FOR_Y: {
                if (!need_y_movement) {
                    phase = NAV_ROTATE_TO_FINAL;
                    
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.phase = phase;
                        xSemaphoreGive(data_mutex);
                    }
                    break;
                }
                float heading_y=0.0;
                if (!obstacle || avoidance_data.is_avoiding) {
                    
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        heading_y = shared_navigation.target_heading_for_y;
                        xSemaphoreGive(data_mutex);
                    }
                    rotate_to_heading(heading_y);
                }
                
                if (is_rotation_complete(heading_y)) {
                    motors_stop();
                    phase = NAV_MOVE_Y;
                    encoders_reset_phase();
                    target_heading_phase = heading_y;
                    pid_distance.reset();
                    pid_heading.reset();
                    
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.phase = NAV_MOVE_Y;
                        xSemaphoreGive(data_mutex);
                    }
                }
                break;
            }
                
            case NAV_MOVE_Y: {
                avoidance_data.current_axis = 'Y';
                if ((!obstacle || avoidance_data.is_avoiding) && need_y_movement) {
                    float distance_y;
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        distance_y = shared_navigation.distance_to_move_y;
                        xSemaphoreGive(data_mutex);
                    }
                    move_forward(distance_y);
                }
                
                float traveled_y = fabs(encoder_get_avg_distance_m());
                float target_y_dist;
                if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                    target_y_dist = shared_navigation.distance_to_move_y;
                    xSemaphoreGive(data_mutex);
                }
                
                if (traveled_y >= fabs(target_y_dist - POSITION_TOLERANCE_M)) {
                    motors_stop();
                    need_y_movement = false;

                    bool x_still_needed = false;
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.distance_to_move_x = fabs(avoidance_data.remaining_x);
                        if (fabs(avoidance_data.remaining_x) > 10 * POSITION_TOLERANCE_M) {
                            shared_navigation.target_heading_for_x = get_heading_for_direction(avoidance_data.remaining_x, 'X');
                            x_still_needed = true;
                        }
                        xSemaphoreGive(data_mutex);
                    }

                    if (x_still_needed) {
                        phase = NAV_ROTATE_FOR_X;
                    } else {
                        phase = NAV_ROTATE_TO_FINAL;
                    }
                    pid_heading.reset();

                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.phase = phase;
                        xSemaphoreGive(data_mutex);
                    }
                }
                break;
            }
                
            case NAV_ROTATE_TO_FINAL: {
                avoidance_data.current_axis = 'F';
                float final_yaw=0.0;
                if (!obstacle || avoidance_data.is_avoiding) {
                    
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        final_yaw = shared_navigation.final_yaw_deg;
                        xSemaphoreGive(data_mutex);
                    }
                    rotate_to_heading(final_yaw);
                }
                
                if (is_rotation_complete(final_yaw)) {
                    motors_stop();
                    
                    if (need_final_forward) {
                        phase = NAV_FINAL_MOVE_FORWARD;
                        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.phase = phase;
                        xSemaphoreGive(data_mutex);
                         }
                    } else {
                        start_next_sequence_step();
                        if(current_state!=STATE_H_TO_T2_STEPPER && current_state!=STATE_H_TO_T1_STEPPER){
                        phase = NAV_STOP;
                        stop_start_ms = millis();
                        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.phase = phase;
                        xSemaphoreGive(data_mutex);
                           }
                        }
                        
                    }
                    
                    encoders_reset_phase();
                    target_heading_phase = get_yaw();
                    pid_distance.reset();
                    pid_heading.reset();
                    
                    
                }
                break;
            }
                
            case NAV_FINAL_MOVE_FORWARD: {
                move_forward_distance(FINAL_MOVE_DISTANCE_M);
                
                float traveled = fabs(encoder_get_avg_distance_m());
                if (traveled >= (FINAL_MOVE_DISTANCE_M - POSITION_TOLERANCE_M)) {
                    motors_stop();
                    
                    if ((current_sequence == SEQ_3 && current_state == STATE_H_TO_K) ||
                        (current_sequence == SEQ_4 && current_state == STATE_H_TO_K) ) {
                        phase = NAV_WAIT_STEPPER;
                    } else if((current_sequence == SEQ_3 && current_state == STATE_H_TO_T1_STEPPER) ||
                        (current_sequence == SEQ_4 && current_state == STATE_H_TO_T2_STEPPER) ) {
                        phase = NAV_WAIT_STEPPER;
                    }  else {
                        start_next_sequence_step();
                        phase = NAV_STOP;
                        stop_start_ms = millis();
                    }
                    
                    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                        shared_navigation.phase = phase;
                        xSemaphoreGive(data_mutex);
                    }
                }
                break;
            }
                
            case NAV_WAIT_STEPPER: {
    if (!shared_navigation.stepper_activated) {
        bool new_nav_started = false;

        if ((current_sequence == SEQ_3 && current_state == STATE_H_TO_K) ||
            (current_sequence == SEQ_4 && current_state == STATE_H_TO_K)) {
            publish_state(18);
            activate_stepper_up(70.0);
            publish_state(14);
            start_next_sequence_step();  // calls start_navigation → sets new phase
            new_nav_started = true;      // ← K→H navigation is now armed
        } 
        else if ((current_sequence == SEQ_3 && current_state == STATE_H_TO_T1_STEPPER) ||
                 (current_sequence == SEQ_4 && current_state == STATE_H_TO_T2_STEPPER)) {
            publish_state(18);
            activate_stepper_down(70.0);
            start_next_sequence_step();  // goes to STATE_WAITING, no start_navigation
            new_nav_started = false;
        }

        if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            shared_navigation.stepper_activated = true;
            xSemaphoreGive(data_mutex);
        }

        if (!new_nav_started) {
            // Only park in NAV_STOP if we're waiting for external signal (screen msg)
            phase = NAV_STOP;
            stop_start_ms = millis();
            if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                shared_navigation.phase = phase;
                xSemaphoreGive(data_mutex);
            }
        }
        // If new_nav_started, phase was already set by start_navigation() — don't touch it
    }
    break;
}
                
            case NAV_STOP: {
                             if (millis() - stop_start_ms >= STOP_DURATION_MS) {
                                 phase = NAV_IDLE;
                                 if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
                                 shared_navigation.phase = NAV_IDLE;
                                 xSemaphoreGive(data_mutex);
                                 }
                           }
                        break;
                    }
                
            case NAV_DONE:{
                motors_stop();
                publish_state(20);

                waiting_for_continue = false;

                start_navigation_flag = false;
                complete_rest_mode = false;
                break;
            }
            case NAV_IDLE:
            default:
                break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =============================================
//  SECTION 20 — SETUP & LOOP
// =============================================

void setup() {
    Serial.begin(115200);
    delay(500);
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    Serial.println("============================================");
    Serial.println("  ROBOT CONTROL SYSTEM - MULTI-TABLE MODE");
    Serial.println("  With Table Detection Flags");
    Serial.println("============================================");
    Serial.println();
    
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        Serial.println("ERROR: Failed to create mutex!");
        while(1);
    }
    publish_queue = xQueueCreate(16, sizeof(int32_t));
    STEPPER_init();
    encoders_init();
    motors_init();
    ultrasonics_init();
    imu_init();
    
    shared_ultrasonic.left_cm = 400.0f;
    shared_ultrasonic.mid_cm = 400.0f;
    shared_ultrasonic.right_cm = 400.0f;
    shared_ultrasonic.obstacle_detected = false;
    shared_ultrasonic.timestamp = 0;
    
    saved_target_state.target_x_m = 0.0f;
    saved_target_state.target_y_m = 0.0f;
    saved_target_state.final_yaw_deg = 0.0f;
    saved_target_state.current_step = STEP_COMPLETE;
    saved_target_state.need_initial_backward = false;
    saved_target_state.need_final_forward = false;
    
    shared_navigation.target_x_m = 0.0f;
    shared_navigation.target_y_m = 0.0f;
    shared_navigation.final_yaw_deg = 0.0f;
    shared_navigation.target_heading_for_x = 0.0f;
    shared_navigation.target_heading_for_y = 90.0f;
    shared_navigation.distance_to_move_x = 0.0f;
    shared_navigation.distance_to_move_y = 0.0f;
    shared_navigation.final_move_distance = FINAL_MOVE_DISTANCE_M;
    shared_navigation.need_initial_backward = false;
    shared_navigation.need_final_forward = true;
    shared_navigation.phase = NAV_IDLE;
    shared_navigation.movement_complete = false;
    shared_navigation.stepper_activated = false;
    
    avoidance_data.is_avoiding = false;
    avoidance_data.obstacle_start_time = 0;
    avoidance_data.wait_start_time = 0;
    avoidance_data.waiting_for_clear = false;
    avoidance_data.avoidance_attempts = 0;
    avoidance_data.original_target_x = 0.0f;
    avoidance_data.original_target_y = 0.0f;
    avoidance_data.original_yaw = 0.0f;
    avoidance_data.original_distance_remaining = 0.0f;
    avoidance_data.current_axis = 'X';
    avoidance_data.avoidance_complete = false;
    avoidance_data.selected_direction = 0;
    
    table1_detected = false;
    table2_detected = false;
    seq1_completed = false;
    seq2_completed = false;
    current_sequence = SEQ_NONE;
    
    delay(500);
    
    xTaskCreatePinnedToCore(
        taskUltrasonicDetection,
        "UltrasonicTask",
        8192,
        NULL,
        2,
        NULL,
        1
    );
    
    xTaskCreatePinnedToCore(
        taskNavigation,
        "NavigationTask",
        40960,
        NULL,
        1,
        NULL,
        1
    );
    
    xTaskCreatePinnedToCore(
        taskMicroROS,
        "MicroROSTask",
        32768,
        NULL,
        3,
        NULL,
        0
    );
   /*
    Serial.println("[SYSTEM] Tasks created. Robot ready!");
    Serial.println("\n📡 Listening for commands:");
    Serial.println("   /detection   → 1=Table1, 2=Table2");
    Serial.println("   /screen      → 5=Continue without stepper, 6=After stepper");
    Serial.println("   /gui_topic   → 4=Sequence 3 (after Table1), 7=Sequence 4 (after Table2)");
    Serial.println("   /manu        → Manual mode commands");
    Serial.println("\n📊 Table detection flags:");
    Serial.println("   Table 1 detected: NO");
    Serial.println("   Table 2 detected: NO");
    Serial.println("============================================");
    Serial.println();
    */
    vTaskDelete(NULL);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
