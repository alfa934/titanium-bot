#include "ros/ros.h"
#include "robot_msgs/pid.h"
#include "robot_msgs/motor.h"
#include "robot_msgs/limit_switch.h"
#include "robot_control/PID.hpp"

ros::Publisher pub_motor, pub_debug_rotation, pub_debug_horizontal, pub_debug_vertical;
ros::Subscriber sub_rotation, sub_horizontal, sub_vertical;
ros::Timer tim_1kHz;
ros::Timer timer_timeout;
ros::Time last_rotation_time, last_horizontal_time, last_vertical_time;
const double TIMEOUT = 0.5;

robot_msgs::limit_switch lim;
robot_msgs::motor motor;
robot_msgs::pid rotation_param, horizontal_param, vertical_param;
robot_msgs::pid pid_debug_rotation, pid_debug_horizontal, pid_debug_vertical;

PID arm_rotation(0, 0, 0);
PID arm_horizontal(0, 0, 0);
PID arm_vertical(0, 0, 0);

void rotationParamPIDCallback(const robot_msgs::pidConstPtr &msg)
{
    last_rotation_time = ros::Time::now();
    rotation_param = *msg;
}

void horizontalParamPIDCallback(const robot_msgs::pidConstPtr &msg)
{
    last_horizontal_time = ros::Time::now();
    horizontal_param = *msg;
}

void verticalParamPIDCallback(const robot_msgs::pidConstPtr &msg)
{
    last_vertical_time = ros::Time::now();
    vertical_param = *msg;
}

void debugPID()
{
    pid_debug_rotation.kp = arm_rotation.getKp();
    pid_debug_rotation.ki = arm_rotation.getKi();
    pid_debug_rotation.kd = arm_rotation.getKd();
    pid_debug_rotation.proportional = arm_rotation.getProportional();
    pid_debug_rotation.integral = arm_rotation.getIntegral();
    pid_debug_rotation.derivative = arm_rotation.getDerivative();
    pid_debug_rotation.error = arm_rotation.getError();
    pid_debug_rotation.prev_error = arm_rotation.getPrevError();
    pid_debug_rotation.setpoint = arm_rotation.getSetpoint();
    pid_debug_rotation.feedback = arm_rotation.getFeedback();
    pid_debug_rotation.max_output = arm_rotation.getMaxOutput();
    pid_debug_rotation.max_windup = arm_rotation.getMaxWindup();
    pid_debug_rotation.output = motor.motor_1;
    pid_debug_rotation.resetPID = rotation_param.resetPID;
    pid_debug_rotation.tolerance = rotation_param.tolerance;

    pid_debug_horizontal.kp = arm_horizontal.getKp();
    pid_debug_horizontal.ki = arm_horizontal.getKi();
    pid_debug_horizontal.kd = arm_horizontal.getKd();
    pid_debug_horizontal.proportional = arm_horizontal.getProportional();
    pid_debug_horizontal.integral = arm_horizontal.getIntegral();
    pid_debug_horizontal.derivative = arm_horizontal.getDerivative();
    pid_debug_horizontal.error = arm_horizontal.getError();
    pid_debug_horizontal.prev_error = arm_horizontal.getPrevError();
    pid_debug_horizontal.setpoint = arm_horizontal.getSetpoint();
    pid_debug_horizontal.feedback = arm_horizontal.getFeedback();
    pid_debug_horizontal.max_output = arm_horizontal.getMaxOutput();
    pid_debug_horizontal.max_windup = arm_horizontal.getMaxWindup();
    pid_debug_horizontal.output = motor.motor_2;
    pid_debug_horizontal.resetPID = horizontal_param.resetPID;
    pid_debug_horizontal.tolerance = horizontal_param.tolerance;

    pid_debug_vertical.kp = arm_vertical.getKp();
    pid_debug_vertical.ki = arm_vertical.getKi();
    pid_debug_vertical.kd = arm_vertical.getKd();
    pid_debug_vertical.proportional = arm_vertical.getProportional();
    pid_debug_vertical.integral = arm_vertical.getIntegral();
    pid_debug_vertical.derivative = arm_vertical.getDerivative();
    pid_debug_vertical.error = arm_vertical.getError();
    pid_debug_vertical.prev_error = arm_vertical.getPrevError();
    pid_debug_vertical.setpoint = arm_vertical.getSetpoint();
    pid_debug_vertical.feedback = arm_vertical.getFeedback();
    pid_debug_vertical.max_output = arm_vertical.getMaxOutput();
    pid_debug_vertical.max_windup = arm_vertical.getMaxWindup();
    pid_debug_vertical.output = motor.motor_3;
    pid_debug_vertical.resetPID = vertical_param.resetPID;
    pid_debug_vertical.tolerance = vertical_param.tolerance;

    pub_debug_rotation.publish(pid_debug_rotation);
    pub_debug_horizontal.publish(pid_debug_horizontal);
    pub_debug_vertical.publish(pid_debug_vertical);
}

void timer1msCallback(const ros::TimerEvent &event)
{
    static int16_t cnt_ms = 0;

    if(cnt_ms >= 9)
    {
        if(rotation_param.resetPID)
        {
            arm_rotation.reset();
        }

        if(horizontal_param.resetPID)
        {
            arm_horizontal.reset();
        }

        if(vertical_param.resetPID)
        {
            arm_vertical.reset();
        }

        arm_rotation.setGains(rotation_param.kp, rotation_param.ki, rotation_param.kd);
        arm_rotation.setMaxOutput(rotation_param.max_output);
        arm_rotation.setMaxWindup(rotation_param.max_windup);
        motor.motor_1 = arm_rotation.update(rotation_param.setpoint, rotation_param.feedback, 10);

        arm_horizontal.setGains(horizontal_param.kp, horizontal_param.ki, horizontal_param.kd);
        arm_horizontal.setMaxOutput(horizontal_param.max_output);
        arm_horizontal.setMaxWindup(horizontal_param.max_windup);
        motor.motor_2 = arm_horizontal.update(horizontal_param.setpoint, horizontal_param.feedback, 10);

        arm_vertical.setGains(vertical_param.kp, vertical_param.ki, vertical_param.kd);
        arm_vertical.setMaxOutput(vertical_param.max_output);
        arm_vertical.setMaxWindup(vertical_param.max_windup);
        motor.motor_3 = arm_vertical.update(vertical_param.setpoint, vertical_param.feedback, 10);

        if(abs(arm_rotation.getError()) < rotation_param.tolerance)
        {
            motor.motor_1 = 0;
        }
        if(abs(arm_horizontal.getError()) < horizontal_param.tolerance)
        {
            motor.motor_2 = 0;
        }
        if(abs(arm_vertical.getError()) < vertical_param.tolerance)
        {
            motor.motor_3 = 0;
        }

        debugPID();

        cnt_ms = 0;
    }
    cnt_ms++;

    pub_motor.publish(motor);
}

void timeoutCallback(const ros::TimerEvent& event)
{
    if ((ros::Time::now() - last_rotation_time).toSec() > TIMEOUT)
    {
        rotation_param.kp = 0;
        rotation_param.ki = 0;
        rotation_param.kd = 0;
        rotation_param.max_output = 0;
        rotation_param.max_windup = 0;
        rotation_param.setpoint = 0;
        rotation_param.feedback = 0;
    }

    if ((ros::Time::now() - last_horizontal_time).toSec() > TIMEOUT)
    {
        horizontal_param.kp = 0;
        horizontal_param.ki = 0;
        horizontal_param.kd = 0;
        horizontal_param.max_output = 0;
        horizontal_param.max_windup = 0;
        horizontal_param.setpoint = 0;
        horizontal_param.feedback = 0;
    }

    if ((ros::Time::now() - last_vertical_time).toSec() > TIMEOUT)
    {
        vertical_param.kp = 0;
        vertical_param.ki = 0;
        vertical_param.kd = 0;
        vertical_param.max_output = 0;
        vertical_param.max_windup = 0;
        vertical_param.setpoint = 0;
        vertical_param.feedback = 0;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_arm_node");
    ros::NodeHandle nh;

    ROS_INFO("PID Arm Node Started!");

    pub_motor = nh.advertise<robot_msgs::motor>("/actuator/motor/arm", 10);
    pub_debug_rotation = nh.advertise<robot_msgs::pid>("pid/debug/rotation", 10);
    pub_debug_horizontal = nh.advertise<robot_msgs::pid>("pid/debug/horizontal", 10);
    pub_debug_vertical = nh.advertise<robot_msgs::pid>("pid/debug/vertical", 10);
    
    sub_rotation = nh.subscribe<robot_msgs::pid>("/pid/param/arm/rotation", 10, rotationParamPIDCallback);
    sub_horizontal = nh.subscribe<robot_msgs::pid>("/pid/param/arm/horizontal", 10, horizontalParamPIDCallback);
    sub_vertical = nh.subscribe<robot_msgs::pid>("/pid/param/arm/vertical", 10, verticalParamPIDCallback);

    tim_1kHz = nh.createTimer(ros::Duration(0.001), timer1msCallback);
    timer_timeout = nh.createTimer(ros::Duration(0.001), timeoutCallback);

    ros::spin();

    return 0;
}
