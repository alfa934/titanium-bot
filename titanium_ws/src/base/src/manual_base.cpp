#include "ros/ros.h"
#include "robot_msgs/controller.h"
#include "robot_msgs/motor.h"
#include "robot_msgs/ultrasonic.h"
#include "robot_msgs/button.h"
#include "robot_msgs/robot_system.h"
#include "robot_msgs/yaw.h"
#include "robot_msgs/pid.h"
#include "robot_control/PID.hpp"


#define WALL_LOST_THRESHOLD 20
#define WALL_DISTANCE_SETPOINT 10
#define SCANNING_SPEED 2

typedef struct 
{
    float kp;
    float ki;
    float kd;
    float max_windup;
    float max_output;
    float tolerance;
} PID_Gains_t;

PID_Gains_t y_gains = {.kp = 0.65, .ki = 0, .kd = 0, .max_windup = 3, .max_output = 3, .tolerance = 2};
PID_Gains_t w_gains = {.kp = 0.45, .ki = 0, .kd = 1, .max_windup = 5, .max_output = 5, .tolerance = 2};

typedef enum
{
  MOVING_LEFT,
  MOVING_RIGHT
} MovementDirection;

MovementDirection current_direction = MOVING_RIGHT;

uint8_t left_lost = 0;
uint8_t right_lost = 0;

ros::Publisher pub_motor, pub_pid_y, pub_pid_w;
ros::Subscriber sub_controller, sub_ultra, sub_button, sub_yaw;
ros::Timer tim_1kHz;

robot_msgs::controller controller;
robot_msgs::motor motor;
robot_msgs::ultrasonic ultra;
robot_msgs::button button;
robot_msgs::yaw yaw;
robot_msgs::robot_system robot_system;
robot_msgs::pid pid_debug_y, pid_debug_w;

PID pid_y(0, 0, 0);
PID pid_w(0, 0, 0);

int16_t y_feedback = 0;
float yaw_degree = 0;
float yaw_radian = 0;
float yaw_adjust = 0;

uint8_t robot_state = 0;
int16_t robot_cnt_ms = 0;

uint8_t control_state = 0;
int16_t control_cnt_ms = 0;

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
    if(msg->startButton == 0)
    {
        robot_system.start = 1;
    }

    if(msg->resetButton == 0)
    {
        robot_system.reset = 1;
    }
}

void subYawCallback(const robot_msgs::yawConstPtr &msg)
{
    yaw = *msg;
}

void debugPID()
{
    pid_debug_y.kp = pid_y.getKp();
    pid_debug_y.ki = pid_y.getKi();
    pid_debug_y.kd = pid_y.getKd();
    pid_debug_y.proportional = pid_y.getProportional();
    pid_debug_y.integral = pid_y.getIntegral();
    pid_debug_y.derivative = pid_y.getDerivative();
    pid_debug_y.error = pid_y.getError();
    pid_debug_y.prev_error = pid_y.getPrevError();
    pid_debug_y.setpoint = pid_y.getSetpoint();
    pid_debug_y.feedback = pid_y.getFeedback();
    pid_debug_y.max_output = pid_y.getMaxOutput();
    pid_debug_y.max_windup = pid_y.getMaxWindup();
    pid_debug_y.output = pid_y.getOutput();
    pid_debug_y.resetPID = 0;
    pid_debug_y.tolerance = y_gains.tolerance;

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

    pub_pid_y.publish(pid_debug_y);
    pub_pid_w.publish(pid_debug_w);
}

float flip_yaw(float original_yaw)
{
   return 360.0f - fmodf(original_yaw, 360.0f);
}

void yaw_process()
{
	static uint8_t offset_once = 0;

	yaw_degree = flip_yaw(yaw.degree);

	if(yaw_degree > 180.001)
	{
		yaw_degree -= 360.001;
	}
	else if(yaw_degree < -180.001)
	{
		yaw_degree += 360.001;
	}

	if(!offset_once)
	{
		yaw_adjust = yaw_degree;
		offset_once++;
	}

	yaw_degree = yaw_degree - yaw_adjust;

    // ROS_INFO("YAW: %.2f", yaw_degree);

	yaw_radian = yaw_degree * M_PI/180.0;
}

void timer1msCallback(const ros::TimerEvent &event)
{
    // int8_t lx = Controller_Drift(controller.lX, 12);
    // int8_t ly = Controller_Drift(controller.lY, 12);
    // int8_t rx = Controller_Drift(controller.rX, 12);
    // int8_t ry = Controller_Drift(controller.rY, 12);

    // lx = map(lx, -128, 127, -25, 25);
    // ly = map(ly, -128, 127, -25, 25);
    // rx = map(rx, -128, 127, -15, 15);
    // ry = map(ry, -128, 127, -25, 25);

    if(!robot_system.start)
    {
        return;
    }

    yaw_process();

    static int16_t cnt_ms = 0;

    if(cnt_ms >= 9)
    {
        int16_t vx = 0;
        int16_t vy = 0;
        int16_t vw = 0;

        bool doPID = true;

        left_lost = (ultra.ultra_c >= WALL_LOST_THRESHOLD);
        right_lost = (ultra.ultra_a >= WALL_LOST_THRESHOLD);

        if (!left_lost && !right_lost) //--- both avg of ultra
        {
            y_feedback = (ultra.ultra_c + ultra.ultra_a) / 2.0f;
        } 
        else if (!left_lost && right_lost) //--- left ultra only
        {
            y_feedback = ultra.ultra_c;

            if (current_direction == MOVING_RIGHT) //-- change direction
            {
                current_direction = MOVING_LEFT;
            }
        }
        else if (left_lost && !right_lost) //--- right ultra only
        {
            y_feedback = ultra.ultra_a;

            if (current_direction == MOVING_LEFT)
            {
                current_direction = MOVING_RIGHT;
            }
        }
        else
        {
            doPID = false;
        }

        if(doPID)
        {
            pid_y.setGains(y_gains.kp, y_gains.ki, y_gains.kd);
            pid_y.setMaxOutput(y_gains.max_output);
            pid_y.setMaxWindup(y_gains.max_windup);
            
            if(current_direction == MOVING_RIGHT)
            {
                vx = SCANNING_SPEED;
            }
            else
            {
                vx = -SCANNING_SPEED;
            }

            vy = -pid_y.update(WALL_DISTANCE_SETPOINT, y_feedback, 10);
        }
        else
        {
            vx = 0;
            vy = 2;
        }
        
        switch (control_state)
        {
            case 0:
                if(controller.crs)
                {
                    control_state++;
                }
                break;
            case 1:
                if(control_cnt_ms >= 3)
                {
                    control_state++;
                    control_cnt_ms = 0;
                }
                control_cnt_ms++;
                break;
            case 2:
                vx = 0;
                if(controller.cir)
                {
                    control_state = 0;
                }
                break;
        default:
            break;
        }

        pid_w.setGains(w_gains.kp, w_gains.ki, w_gains.kd);
        pid_w.setMaxOutput(w_gains.max_output);
        pid_w.setMaxWindup(w_gains.max_windup);
        vw = pid_w.update(-1, yaw_degree, 10);

        // ROS_INFO("VX, VY, VW: %d, %d, %d", vx, vy, vw);

        debugPID();

        motor.motor_a = Kinematics_Triangle(0, vx, vy, vw);
        motor.motor_b = Kinematics_Triangle(1, vx, vy, vw);
        motor.motor_c = Kinematics_Triangle(2, vx, vy, vw);

        pub_motor.publish(motor);

        cnt_ms = 0;
    }
    cnt_ms++;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_base_node");
    ros::NodeHandle nh;

    ROS_INFO("Manual Base Node Started!");

    pub_motor = nh.advertise<robot_msgs::motor>("/actuator/motor/base", 10);
    pub_pid_y = nh.advertise<robot_msgs::pid>("/pid/debug/y", 10);
    pub_pid_w = nh.advertise<robot_msgs::pid>("/pid/debug/w", 10);


    sub_controller = nh.subscribe("/input/controller", 10, subControllerCallback);
    sub_button = nh.subscribe("/input/button", 10, buttonCallback);
    sub_ultra = nh.subscribe("/sensor/ultrasonic", 10, subUltrasonicCallback);
    sub_yaw = nh.subscribe("/sensor/yaw", 10, subYawCallback);

    tim_1kHz = nh.createTimer(ros::Duration(0.001), timer1msCallback);

    ros::spin();

    return 0;
}
