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

static const int LEFT_PIN = 18;
static const int RIGHT_PIN = 19;

int main(int argc, char *argv[]) {
    name_attach_t *attach;
    int duty_cycle;
    char msg[256];
    int rcvid;

    printf("QNX PWM Controller starting...\n");

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
