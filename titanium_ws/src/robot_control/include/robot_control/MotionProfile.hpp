#pragma once

class MotionProfile
{
private:
    float m_maxAccel;
    float m_maxVel;
    float m_distance;
    float m_sign;
    
    float m_accelTime;
    float m_cruiseTime;
    float m_totalTime;
    float m_accelDistance;
    float m_cruiseDistance;

    void computeProfileParameters();

public:
    MotionProfile(float maxAcceleration, float maxVelocity, float distance);
    ~MotionProfile();
    
    struct State
    {
        float position;
        float velocity;
        float acceleration;
    };

    State calculate(float elapsedTime) const;
    float getTotalTime() const;
};