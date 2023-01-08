//------------------------------------------------------------
// File name: utils.c
//------------------------------------------------------------

void print_controller_state(unsigned int seq, int velocity, int pwm)
{
    char s[30];
    sprintf(s, "%i,%i,%i\n", seq, velocity, pwm);
    CDC_Transmit_FS(s, strlen(s));
}