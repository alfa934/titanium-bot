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
    float m_max_windup;
    float m_max_output;

public:
    PID(float kp_input, float ki_input, float kd_input);
    ~PID();

    float update(float setpoint, float feedback, float dt);
    
    float getKp();
    float getKi();
    float getKd();
    float getProportional();
    float getIntegral();
    float getDerivative();
    float getError();
    float getPrevError();
    float getSetpoint();
    float getFeedback();
    float getOutput();
    float getMaxWindup();
    float getMaxOutput();

    void  setGains(float new_kp, float new_ki, float new_kd);
    void  setMaxOutput(float new_max_output);
    void  setMaxWindup(float new_max_windup);
    void  reset();
};