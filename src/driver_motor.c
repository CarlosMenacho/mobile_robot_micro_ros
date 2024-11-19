
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "driver_motor.h"

// drivers
#define motorA1_left 22
#define motorA2_left 21
#define pwm_left 26

#define motorA1_right 19
#define motorA2_right 20
#define pwm_right 18

// encoders
#define encRight_a 2
#define encRight_b 3

#define encLeft_a 14
#define encLeft_b 15

// counters
int32_t contRightEnc = 0;
int32_t contLeftEnc = 0;

// reading encoders
void encoder_interrupt(uint gpio, uint32_t events)
{
    if (gpio == encRight_a)
    {
        if (gpio_get(encRight_b))
        {
            contRightEnc++;
        }
        else
        {
            contRightEnc--;
        }
    }
    if (gpio == encLeft_a)
    {
        if (gpio_get(encLeft_b))
        {
            contLeftEnc++;
        }
        else
        {
            contLeftEnc--;
        }
    }
}

// drive motors
void motor_left_forward(uint vel)
{
    gpio_put(motorA1_left, 1);
    gpio_put(motorA2_left, 0);
    pwm_set_gpio_level(pwm_left, (uint16_t)(vel));
}
void motor_left_backward(uint vel)
{
    gpio_put(motorA1_left, 0);
    gpio_put(motorA2_left, 1);
    pwm_set_gpio_level(pwm_left, (uint16_t)(vel));
}
void motor_right_forward(uint vel)
{
    gpio_put(motorA1_right, 1);
    gpio_put(motorA2_right, 0);
    pwm_set_gpio_level(pwm_right, (uint16_t)(vel));
}
void motor_right_backward(uint vel)
{
    gpio_put(motorA1_right, 0);
    gpio_put(motorA2_right, 1);
    pwm_set_gpio_level(pwm_right, (uint16_t)(vel));
}

void set_encoder_right(int32_t new_val)
{
    contRightEnc = new_val;
}

// set and get encoder counters
void set_encoder_left(int32_t new_val)
{
    contLeftEnc = new_val;
}

int32_t get_encoder_right()
{
    return contRightEnc;
}

int32_t get_encoder_left()
{
    return contLeftEnc;
}

void start_motor_driver()
{
    // configure hardware interrupts
    gpio_init(encRight_a);
    gpio_set_dir(encRight_a, GPIO_IN);
    gpio_pull_up(encRight_a);

    gpio_init(encRight_b);
    gpio_set_dir(encRight_b, GPIO_IN);
    gpio_pull_up(encRight_b);

    gpio_init(encLeft_a);
    gpio_set_dir(encLeft_a, GPIO_IN);
    gpio_pull_up(encLeft_a);

    gpio_init(encLeft_b);
    gpio_set_dir(encLeft_b, GPIO_IN);
    gpio_pull_up(encLeft_b);

    // set interrupt callback
    gpio_set_irq_enabled_with_callback(encRight_a, GPIO_IRQ_EDGE_RISE, true, &encoder_interrupt);
    // enable left encoder pin as interrupt
    gpio_set_irq_enabled(encLeft_a, GPIO_IRQ_EDGE_RISE, true);

    // init motor pin controllers
    gpio_init(motorA1_left);
    gpio_set_dir(motorA1_left, GPIO_OUT);
    gpio_init(motorA2_left);
    gpio_set_dir(motorA2_left, GPIO_OUT);
    gpio_init(motorA1_right);
    gpio_set_dir(motorA1_right, GPIO_OUT);
    gpio_init(motorA2_right);
    gpio_set_dir(motorA2_right, GPIO_OUT);

    // configure pwm pins
    gpio_set_function(pwm_right, GPIO_FUNC_PWM);
    uint sliceNum_right = pwm_gpio_to_slice_num(pwm_right);
    pwm_set_wrap(sliceNum_right, 255);
    pwm_set_chan_level(sliceNum_right, PWM_CHAN_B, 200);
    pwm_set_enabled(sliceNum_right, true);

    gpio_set_function(pwm_left, GPIO_FUNC_PWM);
    uint sliceNum_left = pwm_gpio_to_slice_num(pwm_left);
    pwm_set_wrap(sliceNum_left, 255);
    pwm_set_chan_level(sliceNum_left, PWM_CHAN_B, 200);
    pwm_set_enabled(sliceNum_left, true);
}