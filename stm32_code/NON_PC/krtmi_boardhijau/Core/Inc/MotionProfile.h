/*
 * MotionProfile.h
 *
 *  Created on: Oct 1, 2025
 *      Author: ALFA
 */

#ifndef INC_MOTIONPROFILE_H_
#define INC_MOTIONPROFILE_H_

#include "main.h"

typedef struct {
    float position;
    float velocity;
    float acceleration;
    float target_position;
    float max_velocity;
    float max_acceleration;
    float max_deceleration;
    uint8_t finished;
    float tolerance;
} MotionProfile;

void 	MotionProfile_init(MotionProfile* mp, float max_vel, float max_accel, float max_decel, float tolerance);
void 	MotionProfile_update(MotionProfile* mp, float dt);
uint8_t MotionProfile_finished(MotionProfile* mp);
void 	MotionProfile_set_target(MotionProfile* mp, float new_target);
float 	MotionProfile_get_position(MotionProfile* mp);
float 	MotionProfile_get_velocity(MotionProfile* mp);


#endif /* INC_MOTIONPROFILE_H_ */
