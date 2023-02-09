//------------------------------------------------------------
// File name: utils.c
//------------------------------------------------------------

void print_controller_state(unsigned int seq, int velocity, int pwm)
{
    char s[30];
    sprintf(s, "%i,%i,%i\n", seq, velocity, pwm);
    CDC_Transmit_FS(s, strlen(s));
}

double decode_double(char* bytes)
{
    double d = 0;
    // Copy bypes and converting from little endian 
    char* dBypes = (char*)(&d);
    for(int i = 0; i < 8; i++)
    {
        dBypes[8 - i - 1] = bytes[i];
    }
    return d;
}