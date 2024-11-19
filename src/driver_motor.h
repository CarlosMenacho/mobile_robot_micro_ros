#include <stdio.h>
#include <pico/stdlib.h>
#include <hardware/pwm.h>

#ifndef DRIVER_MOTOR_H
#define DRIVER_MOTOR_H

void encoder_interrupt(uint gpio, uint32_t events);

void motor_left_forward(uint vel);
void motor_left_backward(uint vel);
void motor_right_forward(uint vel);
void motor_right_backward(uint vel);

void set_encoder_right(int32_t new_val);
void set_encoder_left(int32_t new_val);
int32_t get_encoder_right();
int32_t get_encoder_left();

void start_motor_driver();

#endif
