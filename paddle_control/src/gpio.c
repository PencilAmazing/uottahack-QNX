#include "gpio.h"

#include <stdio.h>
#include <sys/neutrino.h>

#include "rpi_gpio.h"

 
// PWM frequency (in Hz) via rpi_gpio.
#define PWM_FREQ 1000


static const int LEFT_PIN = 18;
static const int RIGHT_PIN = 19;

static int setup_pwm_pin(int gpio_pin) {
  int rc;

  // Set up the GPIO pin as an output.
  rc = rpi_gpio_setup(gpio_pin, GPIO_OUT);
  if (rc != GPIO_SUCCESS) {
    printf("ERROR: rpi_gpio_setup() failed for pin %d, rc=%d\n", gpio_pin, rc);
    return rc;
  }

  // Initialize PWM on this pin using Mark/Space mode at PWM_FREQ.
  rc = rpi_gpio_setup_pwm(gpio_pin, PWM_FREQ, GPIO_PWM_MODE_MS);
  if (rc != GPIO_SUCCESS) {
    printf("ERROR: rpi_gpio_setup_pwm() failed for pin %d, rc=%d\n", gpio_pin,
           rc);
    return rc;
  }

  // Set the initial PWM duty cycle to 0% (motor off).
  rc = rpi_gpio_set_pwm_duty_cycle(gpio_pin, 0.0f);
  if (rc != GPIO_SUCCESS) {
    printf("ERROR: rpi_gpio_set_pwm_duty_cycle() failed for pin %d, rc=%d\n",
           gpio_pin, rc);
    return rc;
  }

  return GPIO_SUCCESS;
}

void init_gpio() {
  printf("Init GPIO\n");

  rpi_gpio_setup_pwm(LEFT_PIN, 1000, GPIO_PWM_MODE_PWM);
  rpi_gpio_set_pwm_duty_cycle(LEFT_PIN, 0);

  rpi_gpio_setup_pwm(RIGHT_PIN, 1000, GPIO_PWM_MODE_PWM);
  rpi_gpio_set_pwm_duty_cycle(RIGHT_PIN, 0);
}

void deinit_gpio() { rpi_gpio_cleanup(); }

void paddle_left() {
  rpi_gpio_set_pwm_duty_cycle(LEFT_PIN, 0.8);
  rpi_gpio_set_pwm_duty_cycle(RIGHT_PIN, 0);
}

void paddle_right() {
  rpi_gpio_set_pwm_duty_cycle(LEFT_PIN, 0);
  rpi_gpio_set_pwm_duty_cycle(RIGHT_PIN, 0.8);
}
