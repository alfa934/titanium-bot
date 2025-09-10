#include "ros/ros.h"
#include "robot_msgs/controller.h"
#include "robot_msgs/motor.h"
#include "robot_msgs/ultrasonic.h"
#include "robot_msgs/button.h"
#include "robot_msgs/robot_system.h"
#include "robot_msgs/yaw.h"
#include "robot_msgs/pid.h"
#include "robot_control/PID.hpp"
#include "std_msgs/Bool.h"
#include <cmath>

float calculate_safe_dt(float dt)
{
    return std::max(dt, 0.001f);
}

typedef struct 
{
    float kp;
    float ki;
    float kd;
    float max_windup;
    float max_output;
    float tolerance;
} PID_Gains_t;

PID_Gains_t w_gains = {.kp = 0.25, .ki = 0, .kd = 0.05, .max_windup = 5, .max_output = 5, .tolerance = 2};

uint8_t left_lost = 0;
uint8_t right_lost = 0;

ros::Publisher pub_motor, pub_pid_w, pub_base;
ros::Subscriber sub_controller, sub_ultra, sub_button, sub_yaw;
ros::Timer tim_100Hz;

std_msgs::Bool base_ready;
robot_msgs::controller controller;
robot_msgs::motor motor;
robot_msgs::ultrasonic ultra;
robot_msgs::button button;
robot_msgs::yaw yaw;
robot_msgs::robot_system robot_system;
robot_msgs::pid pid_debug_w;

PID pid_w(0, 0, 0);

static float yaw_degree = 0;
static float yaw_radian = 0;
static float yaw_raw_flip = 0;
static float yaw_adjust = 0;

float w_setpoint = 0;
float w_feedback = 0;
uint8_t robot_state = 0;

int16_t robot_cnt_10ms = 0;

ros::Time last_callback_time;

bool prev_tri_state = false;

int8_t Controller_Drift(int8_t value, int8_t max)
{
    if(abs(value) < max)
    {
        return 0;
    }
    else if(value > 0)
    {
        return value - max;
    }
    else
    {
        return value + max;
    }
}

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

typedef struct {
    int8_t x;
    int8_t y;
} DirectionalMove;

DirectionalMove parseDirectionalMove(robot_msgs::controller c) {
    int8_t x = Controller_Drift(controller.lX, 12);
    int8_t y = Controller_Drift(controller.lY, 12);

    x = map(x, -128, 127, -25, 25);
    y = map(y, -128, 127, -25, 25);
    
    return DirectionalMove { .x = x, .y = y };
}

int16_t vx = 0;
int16_t vy = 0;
int16_t vw = 0;

int16_t Kinematics_Triangle(uint8_t motor, int16_t vx, int16_t vy, int16_t vw)
{
    switch(motor)
    {
        case 0: //-- A
            return vx + vw;
        case 1: //-- B
            return (-0.5 * vx) + (0.866025 * vy) + vw;
        case 2: //-- C
            return (-0.5 * vx) - (0.866025 * vy) + vw;
        default:
            return 0;
    }
    return 0;
}

void subControllerCallback(const robot_msgs::controllerConstPtr &msg)
{
    controller = *msg;
}

void subUltrasonicCallback(const robot_msgs::ultrasonicConstPtr &msg)
{
    ultra = *msg;
}

void buttonCallback(const robot_msgs::buttonConstPtr &msg)
{
    button = *msg;
    if(msg->start)
    {
        robot_system.start = 1;
        robot_system.reset = 0;
    }

    if(msg->reset)
    {
        robot_system.reset = 1;
        robot_system.start = 0;
    }
}

void subYawCallback(const robot_msgs::yawConstPtr &msg)
{
    yaw = *msg;
}

void debugPID()
{
    pid_debug_w.kp = pid_w.getKp();
    pid_debug_w.ki = pid_w.getKi();
    pid_debug_w.kd = pid_w.getKd();
    pid_debug_w.proportional = pid_w.getProportional();
    pid_debug_w.integral = pid_w.getIntegral();
    pid_debug_w.derivative = pid_w.getDerivative();
    pid_debug_w.error = pid_w.getError();
    pid_debug_w.prev_error = pid_w.getPrevError();
    pid_debug_w.setpoint = pid_w.getSetpoint();
    pid_debug_w.feedback = pid_w.getFeedback();
    pid_debug_w.max_output = pid_w.getMaxOutput();
    pid_debug_w.max_windup = pid_w.getMaxWindup();
    pid_debug_w.output = pid_w.getOutput();
    pid_debug_w.resetPID = 0;
    pid_debug_w.tolerance = w_gains.tolerance;

    pub_pid_w.publish(pid_debug_w);
}

float flip_yaw(float original_yaw)
{
   return 360.0f - fmodf(original_yaw, 360.0f);
}

void yaw_process()
{
    static uint8_t offset_once = 0;

    float inverted_yaw = -yaw.degree;
    
    yaw_raw_flip = fmodf(inverted_yaw + 180.0f, 360.0f);
    if (yaw_raw_flip < 0) yaw_raw_flip += 360.0f;
    yaw_raw_flip -= 180.0f;

    if (!offset_once)
    {
        yaw_adjust = yaw_raw_flip;
        offset_once++;
    }

    yaw_degree = yaw_raw_flip - yaw_adjust;

    yaw_degree = fmodf(yaw_degree + 180.0f, 360.0f);
    if (yaw_degree < 0) yaw_degree += 360.0f;
    yaw_degree -= 180.0f;

    yaw_radian = yaw_degree * M_PI / 180.0;
}

void timer10msCallback(const ros::TimerEvent &event)
{
    ros::Time current_time = event.current_real;
    float dt = (current_time - last_callback_time).toSec();
    dt = calculate_safe_dt(dt);
    last_callback_time = current_time;

    yaw_process();

    if(!robot_system.start)
    {
        yaw_adjust = yaw_raw_flip; 
        return;
    }

    if(controller.tri && !prev_tri_state)
    {
        yaw_adjust = yaw_raw_flip;
        w_setpoint = 0.0f;
        pid_w.reset();
        
        ROS_INFO("TRI button pressed: Resetting yaw and setpoint to 0");
    }
    prev_tri_state = controller.tri;

    int8_t lx = Controller_Drift(controller.lX, 12);
    int8_t ly = Controller_Drift(controller.lY, 12);
    int8_t rx = Controller_Drift(controller.rX, 12);
    int8_t ry = Controller_Drift(controller.rY, 12);

    lx = map(lx, -127, 127, -10, 10);
    ly = map(ly, -127, 127, -10, 10);
    rx = map(rx, -127, 127, -2, 2);
    ry = map(ry, -127, 127, -25, 25);

    vx = lx;
    vy = ly;

    w_setpoint += static_cast<float>(-rx) * 0.3f;
    
    w_setpoint = fmodf(w_setpoint, 360.0f);
    if (w_setpoint > 180.0f)
    {
        w_setpoint -= 360.0f;
    }
    else if (w_setpoint < -180.0f)
    {
        w_setpoint += 360.0f;
    }
    
    w_feedback = static_cast<int16_t>(yaw_degree);

    pid_w.setGains(w_gains.kp, w_gains.ki, w_gains.kd);
    pid_w.setMaxOutput(w_gains.max_output);
    pid_w.setMaxWindup(w_gains.max_windup);
    
    float pid_output = pid_w.update_rotate(w_setpoint, w_feedback, dt);

    if (std::isnan(pid_output))
    {
        ROS_WARN("PID output is NaN, resetting PID");
        pid_w.reset();
        vw = 0;
    } 
    else
    {
        vw = static_cast<int16_t>(pid_output);
    }

    motor.motor_a = Kinematics_Triangle(0, vx, vy, vw);
    motor.motor_b = Kinematics_Triangle(1, vx, vy, vw);
    motor.motor_c = Kinematics_Triangle(2, vx, vy, vw);
    
    pub_base.publish(base_ready);
    pub_motor.publish(motor);

    debugPID();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_base_node");
    ros::NodeHandle nh;

    ROS_INFO("Manual Base Node Started!");
    
    ros::AsyncSpinner spinner(0);

    pub_motor = nh.advertise<robot_msgs::motor>("/actuator/motor/base", 10);
    pub_pid_w = nh.advertise<robot_msgs::pid>("/pid/debug/base/w", 10);
    pub_base = nh.advertise<std_msgs::Bool>("/base/ready_state", 10);

    sub_controller = nh.subscribe("/input/controller", 10, subControllerCallback);
    sub_button = nh.subscribe("/input/button", 10, buttonCallback);
    sub_ultra = nh.subscribe("/sensor/ultrasonic", 10, subUltrasonicCallback);
    sub_yaw = nh.subscribe("/sensor/yaw", 10, subYawCallback);

    last_callback_time = ros::Time::now();
    
    tim_100Hz = nh.createTimer(ros::Duration(0.01), timer10msCallback);

    spinner.start();

    ros::waitForShutdown();

    spinner.stop();

    // ros::spin()

    return 0;
}