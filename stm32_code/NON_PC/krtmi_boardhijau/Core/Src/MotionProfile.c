/*
 * MotionProfile.c
 *
 *  Created on: Oct 1, 2025
 *      Author: ALFA
 */


#include "MotionProfile.h"
#include "math.h"


void MotionProfile_init(MotionProfile* mp, float max_vel, float max_accel, float max_decel, float tolerance)
{
    mp->position = 0;
    mp->velocity = 0;
    mp->acceleration = 0;
    mp->target_position = 0;
    mp->max_velocity = max_vel;
    mp->max_acceleration = max_accel;
    mp->max_deceleration = max_decel;
    mp->tolerance = tolerance;
    mp->finished = 1;
}

void MotionProfile_update(MotionProfile* mp, float dt)
{
    if (mp->finished)
    {
        mp->velocity = 0;
        mp->acceleration = 0;
        return;
    }

    float position_error = mp->target_position - mp->position;
    int direction = (position_error > 0) ? 1 : -1;

    float stopping_distance = (mp->velocity * mp->velocity) / (2.0f * mp->max_deceleration);

    int moving_toward_target = (mp->velocity * position_error) > 0;

    if (moving_toward_target && fabsf(position_error) <= stopping_distance)
    {
        mp->acceleration = (mp->velocity > 0) ? -mp->max_deceleration : mp->max_deceleration;
    }
    else
    {
        if (fabsf(mp->velocity) < mp->max_velocity)
        {
            mp->acceleration = direction * mp->max_acceleration;
        }
        else
        {
            mp->acceleration = 0;
        }
    }

    mp->velocity += mp->acceleration * dt;

    if (fabsf(mp->velocity) > mp->max_velocity)
    {
        mp->velocity = (mp->velocity > 0) ? mp->max_velocity : -mp->max_velocity;
    }

    mp->position += mp->velocity * dt;

    if ((position_error > 0 && mp->velocity < -0.5f) || (position_error < 0 && mp->velocity > 0.5f))
    {
        mp->acceleration = (mp->velocity > 0) ? -mp->max_deceleration : mp->max_deceleration;
        mp->velocity += mp->acceleration * dt;
        mp->position += mp->velocity * dt;
    }

    float new_position_error = mp->target_position - mp->position;
    if ((new_position_error > 0 && mp->position > mp->target_position) ||
        (new_position_error < 0 && mp->position < mp->target_position))
    {
        mp->acceleration = (new_position_error > 0) ? -mp->max_deceleration : mp->max_deceleration;
        mp->velocity = 0;
    }

    if (fabsf(new_position_error) <= mp->tolerance && fabsf(mp->velocity) < 1.0f)
    {
        mp->position = mp->target_position;
        mp->velocity = 0;
        mp->acceleration = 0;
        mp->finished = 1;
    }
}

uint8_t MotionProfile_finished(MotionProfile* mp)
{
    return mp->finished;
}

void MotionProfile_set_target(MotionProfile* mp, float new_target)
{
    mp->target_position = new_target;
    mp->finished = 0; //--- false
}

float MotionProfile_get_position(MotionProfile* mp)
{
    return mp->position;
}

float MotionProfile_get_velocity(MotionProfile* mp)
{
    return mp->velocity;
}
