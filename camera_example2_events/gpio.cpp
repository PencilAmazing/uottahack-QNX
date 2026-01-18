#include "gpio.h"

#include <stdio.h>
#include <sys/neutrino.h>
extern "C" {
#include "rpi_gpio.h"
}    

static const int LEFT_PIN = 18;
static const int RIGHT_PIN = 19;

void init_gpio() {
    printf("Init GPIO\n");

    rpi_gpio_setup_pwm(LEFT_PIN, 1000, GPIO_PWM_MODE_PWM);
    rpi_gpio_set_pwm_duty_cycle(LEFT_PIN, 0);
    
    rpi_gpio_setup_pwm(RIGHT_PIN, 1000, GPIO_PWM_MODE_PWM);
    rpi_gpio_set_pwm_duty_cycle(RIGHT_PIN, 0);
}

void deinit_gpio() {
    rpi_gpio_cleanup();
}


void paddle_left() {
    rpi_gpio_set_pwm_duty_cycle(LEFT_PIN, 0.8);
    rpi_gpio_set_pwm_duty_cycle(RIGHT_PIN, 0);
}

void paddle_right() {
    rpi_gpio_set_pwm_duty_cycle(LEFT_PIN, 0);
    rpi_gpio_set_pwm_duty_cycle(RIGHT_PIN, 0.8);
}
