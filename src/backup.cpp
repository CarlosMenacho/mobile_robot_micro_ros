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
int16_t int_error_lef = 0;
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
        int_error_lef += error;
        pid_out = kp_left * error + kd_left * (error - last_error_left) + ki_left * int_error_lef;
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
// tf2_msgs__msg__TFMessage * odom_tf;

void get_cmd(const void *msgin)
{
    const geometry_msgs__msg__Twist *cmd_msg_sub = (const geometry_msgs__msg__Twist *)msgin;
    double linear = cmd_msg_sub->linear.x;
    double angular = cmd_msg_sub->angular.z;
    speed_setpoint_left = linear - angular * (wheelbase / 2);
    speed_setpoint_right = linear + angular * (wheelbase / 2);
}

void odom_timer_callback(rcl_timer_t *odom_timer, int64_t last_call_time)
{
    // move motors
    int32_t contLeftEnc = get_encoder_left();
    int32_t contRightEnc = get_encoder_right();

    speed_current_left = (((double)contLeftEnc / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius;  // units  [m/s]
    speed_current_right = (((double)contRightEnc / encoder_cpr) * 2 * PI) * (1000 / LOOPTIME) * radius; // units  [m/s]
    
    set_encoder_left(0);
    set_encoder_right(0);

    int pwm_left_value = do_PID(1, speed_setpoint_left, speed_current_left);
    if (speed_setpoint_left > 0){
        motor_left_forward(pwm_left_value);
    }
    else{
        motor_left_backward(pwm_left_value);
    }

    int pwm_right_value = do_PID(2, speed_setpoint_right, speed_current_right);
    if (speed_setpoint_right > 0){
        motor_right_forward(pwm_right_value);
    }
    else{
        motor_right_backward(pwm_right_value);
    }
    // calc odometry
    current_time = get_absolute_time();
    dt = absolute_time_diff_us(last_time,current_time)/1000000; //seconds

    if(dt>0){
        right_vel_fltr = (speed_current_right + last_speed_right)/2 ;
        left_vel_fltr = (speed_current_left + last_speed_left)/2;

        d = (right_vel_fltr + left_vel_fltr) * dt / 2;
        th = (right_vel_fltr - left_vel_fltr)* dt / wheelbase;

        dx = d / dt;
        dr = th/ dt;

        if(d != 0){
            x = cos(th)*d;
            y = -sin(th)*d;
            x_final = x_final + ( cos(theta_final)*x - sin(theta_final)*y ); 
            y_final = y_final + ( sin(theta_final)*x + sin(theta_final)*y );
        }

        if(th != 0){
            theta_final = theta_final+ th;
        }
        //publish odom
        odom_quat.x = 0;
        odom_quat.y = 0;
        odom_quat.z = 0;

        odom_quat.z = sin(theta_final /2);
        odom_quat.w = cos(theta_final /2);

        // odom_tf.header.frame_id.data = "raw_odom";
        // odom_tf.child_frame_id.data = "base_footprint";

        // odom_tf.transform.translation.x = x_final;
        // odom_tf.transform.translation.y = y_final;
        // odom_tf.transform.translation.z = 0;
        // odom_tf.transform.rotation = odom_quat;

        odom_msg_pub.header.stamp.sec = dt;
        odom_msg_pub.header.frame_id.data = "raw_odom";
        odom_msg_pub.pose.pose.position.x = speed_current_left;
        odom_msg_pub.pose.pose.position.y = speed_current_right;
        odom_msg_pub.pose.pose.position.z = 0;
        odom_msg_pub.pose.pose.orientation = odom_quat;

        odom_msg_pub.child_frame_id.data = "base_footprint";
        odom_msg_pub.twist.twist.linear.x = dx;
        odom_msg_pub.twist.twist.linear.y = 0;
        odom_msg_pub.twist.twist.angular.z = dr;

        rcl_ret_t ret = rcl_publish(&odom_pub, &odom_msg_pub, NULL);

        last_time = current_time;
        last_speed_left = speed_current_left; 
        last_speed_right = speed_current_right;
    }
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

    // microros config
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;

    rclc_executor_t executor_cmd;   // subscriber
    rclc_executor_t executor_odom;  // publisher
    rcl_timer_t odom_timer;         // publisher timer
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
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "raw_odom");

    rclc_timer_init_default(
        &odom_timer,
        &support,
        RCL_MS_TO_NS(LOOPTIME),
        odom_timer_callback);

    rclc_executor_init(&executor_odom, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor_odom, &odom_timer);

    // cmd subscriber
    rclc_subscription_init_default(
        &cmd_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel");

    rclc_executor_add_subscription(
        &executor_cmd,
        &cmd_sub,
        &cmd_msg_sub,
        &get_cmd,
        ON_NEW_DATA);
    rclc_executor_init(&executor_cmd, &support.context, 1, &allocator);

    gpio_put(LED_PIN, 1);

    last_time = get_absolute_time();

    while (true){
        rclc_executor_spin_some(&executor_cmd, RCL_MS_TO_NS(100));
        rclc_executor_spin_some(&executor_odom, RCL_MS_TO_NS(100));
    }
    return 0;
}
