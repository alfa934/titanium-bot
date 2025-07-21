#include "arm/arm_inverse.h"

void get_pos_output(inv_output_t *pos_output, float x_input, float y_input)
{
    pos_output -> length_pos_px = sqrt((x_input * x_input) + (y_input * y_input));
    pos_output -> theta_pos_deg = (acos(x_input / pos_output -> length_pos_px)) * (180/M_PI);
    
    if(x_input == 0)
    {
        pos_output -> theta_pos_deg = 90;
    }
    else if(x_input < 0)
    {
        pos_output -> theta_pos_deg = 180 - (pos_output -> theta_pos_deg);
    }
}
