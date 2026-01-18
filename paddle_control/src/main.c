#include <unistd.h> // Required for usleep()
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <rpi_gpio.h>
#include <sys/neutrino.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>

static const int RIGHT_PIN = 18;
static const int LEFT_PIN = 19;

void run_command(const char* cmd) {
    int ret = system(cmd);
    if (ret != 0) {
        printf("error executing ");
        printf(cmd);
        printf("\n");
    }
}

void go_right() {
    run_command("gpio-rp1 set 18 op dh");
    run_command("gpio-rp1 set 19 op dl");
    
    /* rpi_gpio_set_pwm_duty_cycle(RIGHT_PIN, 0.8); */
    /* rpi_gpio_set_pwm_duty_cycle(LEFT_PIN, 0); */
}

void go_left() {
    run_command("gpio-rp1 set 18 op dl");
    run_command("gpio-rp1 set 19 op dh");
    /* rpi_gpio_set_pwm_duty_cycle(RIGHT_PIN, 0); */
    /* rpi_gpio_set_pwm_duty_cycle(LEFT_PIN, 0.5); */
}

void stop() {
    run_command("gpio-rp1 set 19 op dl");
    run_command("gpio-rp1 set 18 op dl");
}

int main(int argc, char *argv[]) {
    name_attach_t *attach;
    int duty_cycle;
    char msg[256];
    int rcvid;

    printf("QNX PWM Controller starting...\n");
    run_command("gpio-rp1 set 18 op dl");
    run_command("gpio-rp1 set 19 op dl");
    
    if (rpi_gpio_setup_pwm(LEFT_PIN, 1000, GPIO_PWM_MODE_PWM)) {
        fprintf(stderr, "Failed to initialize PWM\n");
        return EXIT_FAILURE;
    }
    rpi_gpio_set_pwm_duty_cycle(LEFT_PIN, 0);

    attach = name_attach(NULL, "pwm_controller", 0);
    if (attach == NULL) {
        perror("name_attach failed");
        /* cleanup_pwm(&ctrl); */
        return EXIT_FAILURE;
    }

    printf("PWM Controller ready. Waiting for messages...\n");

    while (1) {
        rcvid = MsgReceive(attach->chid, &msg, sizeof(msg), NULL);
        duty_cycle = *(int*)msg;
        
        if (rcvid == -1) {
            perror("MsgReceive failed");
            break;
        }

        if (rcvid == 0) continue;  // Pulse

        MsgReply(rcvid, EOK, NULL, 0);

        if (duty_cycle == 262422) {
            // Skip this, I don't know why or how
            continue;
        }
        
        // Set duty cycle and reply
        printf("Received %d\n", duty_cycle);

        if (duty_cycle < -100) {
            printf("left\n");
            go_left();
        } else if(duty_cycle > 100) {
            printf("right\n");
            go_right();
        } else {
            printf("stop\n");
            stop();
        }
        
        /* if (set_duty_cycle(&ctrl, (float)duty_cycle) == 0) { */
        /*     MsgReply(rcvid, EOK, NULL, 0); */
        /* } else { */
        /*     MsgReply(rcvid, EIO, NULL, 0); */
        /* } */
    }

    name_detach(attach, 0);
    /* cleanup_pwm(&ctrl); */
    return EXIT_SUCCESS;
}
