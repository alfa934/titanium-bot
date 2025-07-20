#include "arm/arm_inverse.h"

void get_pos_output(inv_output_t *pos_output, float x_input, float y_input)
{
    pos_output -> length_pos_px = sqrt((x_input * x_input) + (y_input * y_input));
    pos_output -> theta_pos_deg = (atan(x_input / y_input)) * (180/M_PI); //tihs outputs radian, not degrees
}
