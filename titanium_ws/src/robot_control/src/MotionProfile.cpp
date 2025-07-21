//--- https://www.ctrlaltftc.com/advanced/motion-profiling

#include "robot_control/MotionProfile.hpp"
#include <cmath>
#include <algorithm>

MotionProfile::MotionProfile(float maxAcceleration, float maxVelocity, float distance)
    : m_maxAccel(std::abs(maxAcceleration)),
      m_maxVel(std::abs(maxVelocity)),
      m_distance(distance),
      m_sign(distance >= 0 ? 1.0 : -1.0)
{
    computeProfileParameters();
}

MotionProfile::~MotionProfile()
{
}

void MotionProfile::computeProfileParameters()
{
    const float absDistance = std::abs(m_distance);
    
    if (absDistance < 1e-9) 
    {
        m_accelTime = 0;
        m_cruiseTime = 0;
        m_totalTime = 0;
        m_accelDistance = 0;
        m_cruiseDistance = 0;
        return;
    }

    m_accelTime = m_maxVel / m_maxAccel;
    m_accelDistance = 0.5 * m_maxAccel * std::pow(m_accelTime, 2);

    if (2 * m_accelDistance > absDistance)
    {
        m_accelTime = std::sqrt(absDistance / m_maxAccel);
        m_accelDistance = 0.5 * m_maxAccel * std::pow(m_accelTime, 2);
        m_maxVel = m_maxAccel * m_accelTime;
    }

    m_cruiseDistance = absDistance - 2 * m_accelDistance;
    m_cruiseTime = m_cruiseDistance / m_maxVel;
    m_totalTime = 2 * m_accelTime + m_cruiseTime;
}

MotionProfile::State MotionProfile::calculate(float elapsedTime) const
{
    State state{0.0, 0.0, 0.0};
    float t = elapsedTime;
    
    if (t < 0)
    {
        t = 0;
    }
    if (t > m_totalTime)
    {
        t = m_totalTime;
    }

    
    if (t < m_accelTime) //--- accelerate
    {
        state.position = 0.5 * m_maxAccel * t * t;
        state.velocity = m_maxAccel * t;
        state.acceleration = m_maxAccel;
    }
    else if (t < m_accelTime + m_cruiseTime) //--- cruise
    {
        const float cruiseT = t - m_accelTime;
        state.position = m_accelDistance + m_maxVel * cruiseT;
        state.velocity = m_maxVel;
        state.acceleration = 0.0;
    }
    else if (t < m_totalTime) //--- decelerate
    {
        const float decelT = t - (m_accelTime + m_cruiseTime);
        const float decelVel = m_maxVel - m_maxAccel * decelT;
        state.position = m_accelDistance + m_cruiseDistance + 
                         (m_maxVel + decelVel) * 0.5 * decelT;
        state.velocity = decelVel;
        state.acceleration = -m_maxAccel;
    }
    else //--- done
    {
        state.position = std::abs(m_distance);
        state.velocity = 0.0;
        state.acceleration = 0.0;
    }
    
    state.position *= m_sign;
    state.velocity *= m_sign;
    state.acceleration *= m_sign;
    
    return state;
}

float MotionProfile::getTotalTime() const
{
    return m_totalTime;
}