//------------------------------------------------------------
// File name: utils.c
//------------------------------------------------------------
#include "utils.h"
#include <string.h>
#include <stdio.h>
#include "usbd_cdc_if.h"

void print_controller_state(unsigned int seq, int motorIndex, int velocity, int targetVelocity, int pwm)
{
    char msg[100];
    sprintf(msg, "{\"seq\":%i,\"motorIndex\":%i,\"velocity\":%i,\"targetVelocity\":%i,\"pwm\":%i}\n", seq, motorIndex, velocity, targetVelocity, pwm);
    CDC_Transmit_FS(msg, strlen(msg));
}