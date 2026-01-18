#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

int main(int argc, char **argv) {
    if (argc < 3) {
        printf("Usage: %s <attach_point> <duty_cycle>\n", argv[0]);
        return 1;
    }
    
    int coid = name_open(argv[1], 0);
    if (coid == -1) {
        perror("open failed");
        return 1;
    }
    
    int duty = atoi(argv[2]);
    printf("Sending duty cycle: %d to %s\n", duty, argv[1]);
    
    int status = MsgSend(coid, &duty, sizeof(int), NULL, 0);
    if (status == -1) {
        perror("MsgSend failed");
        close(coid);
        return 1;
    }
    
    printf("Message sent successfully (status: %d)\n", status);
    close(coid);
    return 0;
}
