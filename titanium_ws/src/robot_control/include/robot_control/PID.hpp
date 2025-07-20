#pragma once

#include <cmath>

class PID
{

private:
    float m_kp;
    float m_ki;
    float m_kd;
    float m_proportional;
    float m_integral;
    float m_derivative;
    float m_error;
    float m_prev_error;
    float m_setpoint;
    float m_feedback;
    float m_output;
    float m_max_output;

public:
    PID(float kp_input, float ki_input, float kd_input);
    ~PID();

    float update(float setpoint, float feedback, float max_output, float dt);
    
    float getError();
    float getProportional();
    float getIntegral();
    float getDerivative();
    float getOutput();

    void  setGains(float new_kp, float new_ki, float new_kd);
    void  reset();
};