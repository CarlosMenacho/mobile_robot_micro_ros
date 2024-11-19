#include <stdio.h>
#include <math.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/quaternion.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <rmw_microros/rmw_microros.h>

#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/pwm.h>

#include "../pico_uart_transports.h"
#include "driver_motor.h"
#include "mpu9250.h"

const uint LED_PIN = 25;

#define LOOPTIME 100

// robot parameters in [m]
#define wheelbase 0.2
#define radius 0.04
#define encoder_cpr 200

const double PI = 3.1416;

// timming vars
absolute_time_t current_time;
absolute_time_t last_time;

// odometry vars
double dt, d_left, d_right;
double dx, dr;
double left_vel_fltr, right_vel_fltr;
double th, d, x, y ;
double last_speed_left = 0;
double last_speed_right = 0;
double x_final=0, y_final=0, theta_final=0;

// robot PID
double speed_setpoint_left = 0;
double speed_current_left = 0;
int16_t int_error_left = 0;
int16_t last_error_left = 0;
float kp_left = 5;
float ki_left = 0.2;
float kd_left = 2;

double speed_setpoint_right = 0;
double speed_current_right = 0;
int16_t int_error_right = 0;
int16_t last_error_right = 0;
float kp_right = 5;
float ki_right = 0.2;
float kd_right = 2;

float pid_out = 0;

int do_PID(int id, float target_value_pid, float current_value_pid)
{
    float error = target_value_pid - current_value_pid;
    if (id == 1){
        int_error_left += error;
        pid_out = kp_left * error + kd_left * (error - last_error_left) + ki_left * int_error_left;
        last_error_left = error;
    }
    else if (id == 2){
        int_error_right += error;
        pid_out = kp_right * error + kd_right * (error - last_error_right) + ki_right * int_error_right;
        last_error_right = error;
    }
    if (pid_out >= 255){
        pid_out = 255;
    }
    else if (pid_out < 0){
        pid_out = 0;
    }
    return (int)(pid_out);
}

rcl_subscription_t cmd_sub;
rcl_publisher_t odom_pub;
rcl_publisher_t imu_pub;
nav_msgs__msg__Odometry odom_msg_pub;
geometry_msgs__msg__Twist cmd_msg_sub;
geometry_msgs__msg__TransformStamped odom_tf;
geometry_msgs__msg__Quaternion odom_quat;
sensor_msgs__msg__Imu imu_msg_pub;
geometry_msgs__msg__Vector3 vels_msg_pub;

void get_cmd(const void *msgin)
{
    const geometry_msgs__msg__Twist *cmd_msg_sub = (const geometry_msgs__msg__Twist *)msgin;
    double linear = (double)cmd_msg_sub->linear.x;
    double angular = (double)cmd_msg_sub->angular.z;
    speed_setpoint_left = linear - angular * (wheelbase / 2);
    speed_setpoint_right = linear + angular * (wheelbase / 2);
}

void imu_pub_callback(rcl_timer_t *imu_time, int64_t last_call_time)
{
    RCL_UNUSED(last_call_time);
    RCL_UNUSED(imu_time);
}
void pub_vels_callback(rcl_timer_t *vels_time, int64_t last_call_time)
{
    RCL_UNUSED(last_call_time);
    RCL_UNUSED(vels_time);
}

int main()
{
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    start_motor_driver();
    start_spi();
    int16_t acceleration[3], gyro[3], gyroCal[3], eulerAngles[2], fullAngles[2], mag[3];
    calibrate_gyro(gyroCal, 100);

    // microros config
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;

    rclc_executor_t executor_cmd;   // subscriber
    rclc_executor_t executor_odom;  // publisher
    rclc_executor_t executor_imu;
    rcl_timer_t vels_timer;         // publisher timer
    rcl_timer_t imu_timer;
    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.<joint name="head_swivel" type="continuous">
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    // odometry publisher
    rclc_publisher_init_default(
        &odom_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
        "wheel_vels");

    rclc_timer_init_default(
        &vels_timer,
        &support,
        RCL_MS_TO_NS(LOOPTIME),
        pub_vels_callback);

    rclc_executor_init(&executor_odom, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor_odom, &vels_timer);

    // imu publisher
    rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_data"
    );
    rclc_timer_init_default(
        &imu_timer,
        &support,
        RCL_MS_TO_NS(LOOPTIME),
        imu_pub_callback
    );
    rclc_executor_init(&executor_imu, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor_imu, &imu_timer);
    // cmd subscriber
    rclc_subscription_init_default(
        &cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    rclc_executor_init(&executor_cmd, &support.context, 1, &allocator);
    rclc_executor_add_subscription(
        &executor_cmd,
        &cmd_sub,
        &cmd_msg_sub,
        &get_cmd,
        ON_NEW_DATA);

    gpio_put(LED_PIN, 1);

    last_time = get_absolute_time();

    while (true){
        rclc_executor_spin_some(&executor_cmd, RCL_MS_TO_NS(100));
        motor_right_forward(100);
        motor_left_forward(100);
        
        current_time = get_absolute_time();
        if(absolute_time_diff_us(last_time, current_time)>= LOOPTIME*1000){

            last_time = get_absolute_time();
            
            int32_t contLeftEnc = get_encoder_left();
            int32_t contRightEnc = get_encoder_right();

            speed_current_left = (((double)-contLeftEnc / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius;  // units  [m/s]
            speed_current_right = (((double)contRightEnc / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // units  [m/s]
            
            set_encoder_left(0);
            set_encoder_right(0);

            int pwm_left_value = do_PID(1, (float)speed_setpoint_left, (float)speed_current_left);
            if (speed_setpoint_left > 0){
                motor_left_forward(pwm_left_value);
            }
            else{
                motor_left_backward(pwm_left_value);
            }

            int pwm_right_value = do_PID(2, (float)speed_setpoint_right, (float)speed_current_right);
            if (speed_setpoint_right > 0){
                motor_right_forward(pwm_right_value);
            }
            else{
                motor_right_backward(pwm_right_value);
            }

            vels_msg_pub.x = (double)speed_current_left;
            vels_msg_pub.y = (double)speed_current_right;
            vels_msg_pub.z = (double)speed_setpoint_left;

            rcl_ret_t ret = rcl_publish(&odom_pub, &vels_msg_pub, NULL);

            //Publish imu data
            mpu9250_read_raw_accel(acceleration);
            mpu9250_read_raw_gyro(gyro);
            gyro[0] -= gyroCal[0];  //Applies the calibration
            gyro[1] -= gyroCal[1];
            gyro[2] -= gyroCal[2];
            // imu_msg_pub.header.stamp.sec = 
            imu_msg_pub.header.frame_id.data = "imu_data_mpu9250";

            imu_msg_pub.linear_acceleration.x = (double)acceleration[0]/16384.0;
            imu_msg_pub.linear_acceleration.y = (double)acceleration[1]/16384.0;
            imu_msg_pub.linear_acceleration.z = (double)acceleration[2]/16384.0;

            imu_msg_pub.angular_velocity.x = (double)gyro[0] * 0.017453;
            imu_msg_pub.angular_velocity.y = (double)gyro[1] * 0.017453;
            imu_msg_pub.angular_velocity.z = (double)gyro[2] * 0.017453;

            rcl_ret_t ret2 = rcl_publish(&imu_pub, &imu_msg_pub, NULL);

        }
        // rclc_executor_spin_some(&executor_odom, RCL_MS_TO_NS(100));
    }
    return 0;
}