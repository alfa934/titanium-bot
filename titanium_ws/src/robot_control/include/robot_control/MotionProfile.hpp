#pragma once

class MotionProfile {
private:
    double m_maxAccel;
    double m_maxVel;
    double m_distance;
    double m_sign;
    
    double m_accelTime;
    double m_cruiseTime;
    double m_totalTime;
    double m_accelDistance;
    double m_cruiseDistance;

    void computeProfileParameters();

public:
    MotionProfile(double maxAcceleration, double maxVelocity, double distance);
    
    struct State
    {
        double position;
        double velocity;
        double acceleration;
    };

    State calculate(double elapsedTime) const;
    double getTotalTime() const;
};