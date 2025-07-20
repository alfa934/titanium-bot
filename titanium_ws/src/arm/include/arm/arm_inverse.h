#ifndef ARM_INVERSE_H
#define ARM_INVERSE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

typedef struct
{
    float theta_pos_deg;
    float length_pos_px;
} inv_output_t;


void get_pos_output(inv_output_t *pos_output, float x_input, float y_input);


#endif