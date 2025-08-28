#include "robot_control/PID.hpp"

PID::PID(float kp_input, float ki_input, float kd_input)
    : m_kp(kp_input),
      m_ki(ki_input),
      m_kd(kd_input),
      m_proportional(0.0f),
      m_integral(0.0f),
      m_derivative(0.0f),
      m_error(0.0f),
      m_prev_error(0.0f),
      m_setpoint(0.0f),
      m_feedback(0.0f),
      m_output(0.0f),
      m_max_output(0.0f)
{
}

PID::~PID()
{
}

float PID::update(float setpoint, float feedback, float max_output, float dt)
{
    m_setpoint = setpoint;
    m_feedback = feedback;
    m_max_output = max_output;

    m_error = m_setpoint - m_feedback;

    m_proportional = m_kp * m_error;
	m_integral    += m_ki * m_error * dt;
	m_derivative   = m_kd * (m_error - m_prev_error) * dt;
	m_prev_error   = m_error;

	if(m_integral >= m_max_output)        { m_integral =   m_max_output;  }
	else if(m_integral < -(m_max_output)) { m_integral = -(m_max_output); }

	m_output = (m_proportional) + (m_integral) + (m_derivative);

	if(m_output >= m_max_output) 			{ m_output =   m_max_output;  }
	else if(m_output < -(m_max_output)) 	{ m_output = -(m_max_output); }

	return m_output;
}

float PID::getError()
{
    return m_error;
}

float PID::getProportional()
{
    return m_proportional;
}

float PID::getIntegral()
{
    return m_integral;
}

float PID::getDerivative()
{
    return m_derivative;
}

float PID::getOutput()
{
    return m_output;
}

void PID::setGains(float new_kp, float new_ki, float new_kd)
{
    m_kp = new_kp;
    m_ki = new_ki;
    m_kd = new_kd;
}

void PID::reset()
{
    m_proportional = 0;
    m_integral = 0;
    m_derivative = 0;
    m_error = 0;
    m_prev_error = 0;
    m_setpoint = 0;
    m_feedback = 0;
    m_output = 0;
    m_max_output = 0;
}