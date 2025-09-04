#include "ros/ros.h"
#include "robot_msgs/controller.h"
#include "robot_msgs/motor.h"
#include "robot_msgs/ultrasonic.h"
#include "robot_msgs/button.h"
#include "robot_msgs/robot_system.h"
#include "robot_msgs/yaw.h"
#include "robot_msgs/pid.h"
#include "robot_control/PID.hpp"

/*
 * In the beginning, instead of moving forward blindly, it will use
 * the ultrasonics for y and w pid. Once it satisfies the threshold,
 * it will reset the yaw_degree. So that for side to side motion, it
 * will be based on the MPU. It resets again when the user stops
 */

#define WALL_LOST_THRESHOLD 20
#define WALL_DISTANCE_SETPOINT 10
#define SCANNING_SPEED 2

    int16_t vx = 0;
    int16_t vy = 0;
    int16_t vw = 0;

typedef enum
{
    MPU_YAW_PID,
    ULTRASONIC_YAW_PID
} PID_Type;

PID_Type current_pid = MPU_YAW_PID;

typedef struct 
{
    float kp;
    float ki;
    float kd;
    float max_windup;
    float max_output;
    float tolerance;
} PID_Gains_t;

PID_Gains_t y_gains = {.kp = 0.5, .ki = 0, .kd = 0.65, .max_windup = 5, .max_output = 5, .tolerance = 5};
PID_Gains_t w_ultra_gains = {.kp = 0.5, .ki = 0, .kd = 1, .max_windup = 5, .max_output = 5, .tolerance = 2};
PID_Gains_t w_mpu_gains = {.kp = 0.5, .ki = 0, .kd = 1, .max_windup = 5, .max_output = 5, .tolerance = 2};
PID_Gains_t w_gains;


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
ros::Timer tim_100Hz;

robot_msgs::controller controller;
robot_msgs::motor motor;
robot_msgs::ultrasonic ultra;
robot_msgs::button button;
robot_msgs::yaw yaw;
robot_msgs::robot_system robot_system;
robot_msgs::pid pid_debug_y, pid_debug_w;

PID pid_y(0, 0, 0);
PID pid_w(0, 0, 0);

static float yaw_degree = 0;
static float yaw_radian = 0;
static float yaw_raw_flip = 0;
static float yaw_adjust = 0;

int16_t y_setpoint = 0, y_feedback = 0;
int16_t w_setpoint = 0, w_feedback = 0;
uint8_t robot_state = 0;

int16_t robot_cnt_10ms = 0;

enum BaseCommand : uint8_t {
    none,
    armaction,
    allowmove,
    switchmode
};

BaseCommand parseBaseCommand (robot_msgs::controller c) {
    if (c.crs) return armaction;
    if (c.cir) return allowmove;
    if (c.ps) return switchmode;
    return none;
}

enum ControlMode : uint8_t {
    auto_scanning,
    manual_side,
    manual_all
};
ControlMode robot_control_mode = manual_side;

enum BaseState : uint8_t {
    wait_start,
    homing,
    adjustultraPID,
    stayinplace,
    scanning,
    usercontrol_side,
    usercontrol_all // for later
};
BaseState robot_base_state = wait_start;

BaseState processBaseState(BaseCommand cmd, BaseState s, ControlMode &m) {
    if (cmd == armaction) {
        return stayinplace;
    }
    if (cmd == allowmove) {
        if (m == auto_scanning) return scanning;
        if (m == manual_side) return usercontrol_side;
        if (m == manual_all) return usercontrol_all;
    }
    if (cmd == switchmode) {
        if (m == auto_scanning) return usercontrol_side;
        if (m == manual_side) return scanning;
    }
    return s;
}

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
    if(msg->startButton)
    {
        robot_system.start = 1;
        robot_system.reset = 0;
    }

    if(msg->resetButton)
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

	yaw_raw_flip = flip_yaw(yaw.degree);

	if(yaw_raw_flip > 180.001)
	{
		yaw_raw_flip -= 360.001;
	}
	else if(yaw_raw_flip < -180.001)
	{
        yaw_raw_flip += 360.001;
	}

	if(!offset_once)
	{
		yaw_adjust = yaw_raw_flip;
		offset_once++;
	}

	yaw_degree = yaw_raw_flip - yaw_adjust;

    ROS_INFO("YAW: %.2f", yaw_degree);

	yaw_radian = yaw_degree * M_PI/180.0;
}

// return:
// 0 if wall not detected
// 1 if wall detected on both ultras
// 2 if wall detected only on left
// 3 if wall detected only on right
uint8_t maintainDistanceY (uint16_t ultra_left, uint16_t ultra_right) {
    uint8_t left_lost = (ultra_left >= WALL_LOST_THRESHOLD);
    uint8_t right_lost = (ultra_right >= WALL_LOST_THRESHOLD);

    uint8_t ret;
    if (!left_lost && !right_lost) {
        y_feedback = (ultra_left + ultra_right) / 2.0f;
        ret = 1;
    }
    else if (!left_lost && right_lost) {
        y_feedback = ultra_left;
        ret = 2;
    }
    else if (left_lost && !right_lost) {
        y_feedback = ultra_right;
        ret = 3;
    }
    else {
        return 0;
    }

    pid_y.setGains(y_gains.kp, y_gains.ki, y_gains.kd);
    pid_y.setMaxOutput(y_gains.max_output);
    pid_y.setMaxWindup(y_gains.max_windup);
    return ret;
}

void timer10msCallback(const ros::TimerEvent &event)
{

    //--- process the MPU yaw
    yaw_process();

    // reset all state first
    // int16_t vx = 0;
    // int16_t vy = 0;
    // int16_t vw = 0;

    //--- parse the controller base command
    BaseCommand command = parseBaseCommand(controller);
    DirectionalMove user_move = parseDirectionalMove(controller);

    //--- check for input
    if (command != none)
    {
        if (command == switchmode)
        {
            if (robot_control_mode == manual_side && robot_base_state == usercontrol_side)
                robot_control_mode = auto_scanning;
            else if (robot_control_mode == auto_scanning && robot_base_state == scanning)
                robot_control_mode = manual_side;
        }
        BaseState next_base_state = processBaseState(command, robot_base_state, robot_control_mode);
    }

    //--- check ultrasound states
    uint16_t ultra_left = ultra.ultra_c;
    uint16_t ultra_right = ultra.ultra_a;
    
    // //--- execute current action
    // switch (robot_base_state)
    // {
    //     case wait_start:
    //         if (robot_system.start)
    //             robot_base_state = homing;
    //         break;

    //     // initial homing
    //     case homing:
    //         // TODO: ADD HOMING PID

    //         //--- move forward
    //         vx = 0;
    //         vy = 2;
            
    //         //--- if both ultrasound not lost,
    //         //--- go to ultrasound PID adjustment for MPU reset

    //         break;
        
    //     case adjustultraPID:
    //         //--- adjust ultrasound PID

    //         //--- if ultrasound PID within tolerance, robot ready
    //         //--- go to either usercontrol_side or scanning depending on current robot_control_mode

    //         break;

    //     case usercontrol_side:
    //     {
    //         //--- check ultrasound first
    //         uint8_t doYPID = maintainDistanceY(ultra_left, ultra_right);

    //         //--- if wall detection fails, do homing
    //         if (!doYPID) {
    //             robot_base_state = homing;
    //         }
    //         //--- if both walls detected, allow any movement
    //         //--- if only left ultra detected, user can only move left
    //         //--- if only right ultra detected, user can only move right
    //         else if (doYPID == 1
    //                 || doYPID == 2 && user_move.x < 0
    //                 || doYPID == 3 && user_move.x > 0)
    //             vx = user_move.x;
    //             vy = -pid_y.update(WALL_DISTANCE_SETPOINT, y_feedback, 10);
    //         break;
    //     }

    //     // scanning
    //     case scanning:
    //     {
    //         // initialize forward / rotation PID
    //         bool do_scanning = true;
    //         uint8_t doYPID = maintainDistanceY(ultra_left, ultra_right);
    //         if (doYPID == 2 && current_direction == MOVING_RIGHT)
    //             current_direction == MOVING_LEFT;
    //         else if (doYPID == 3 && current_direction == MOVING_LEFT)
    //             current_direction == MOVING_RIGHT;
    //         if (doYPID != 0) {
    //             if (current_direction == MOVING_RIGHT)
    //                 vx = SCANNING_SPEED;
    //             else
    //                 vx = -SCANNING_SPEED;
    //             vy = -pid_y.update(WALL_DISTANCE_SETPOINT, y_feedback, 10);
    //         }
    //         // go to homing
    //         else
    //         {
    //             robot_base_state = homing;
    //         }
    //         break;
    //     }
    //     default:
    //         break;
    // }



    switch (robot_state)
    {
        case 0: //--- wait for start
            yaw_adjust = yaw_raw_flip; // makes sure yaw is at 0
            if(robot_system.start)
            {
                robot_state++;
            }
            break;
        case 1: // go to front of conveyor and adjust orientation (all with ultrasonic pid)
            vx = 0;
            current_pid = ULTRASONIC_YAW_PID;

            y_setpoint = WALL_DISTANCE_SETPOINT;
            y_feedback = (ultra_left + ultra_right) / 2.0f;
            pid_y.setGains(y_gains.kp, y_gains.ki, y_gains.kd);
            pid_y.setMaxOutput(y_gains.max_output);
            pid_y.setMaxWindup(y_gains.max_windup);
            vy = -pid_y.update(y_setpoint, y_feedback, 10);

            // once done, switch to MPU pid
            if(abs(pid_y.getError()) <= y_gains.tolerance && abs(pid_w.getError()) <= w_gains.tolerance)
            {
                if(robot_cnt_10ms >= 150)
                {
                    current_pid = MPU_YAW_PID;
                    yaw_adjust = yaw_raw_flip;
                    robot_state++;
                    robot_cnt_10ms = 0;
                }
                robot_cnt_10ms++;;
            }
            break;
        case 2:
            //-- hold distance (since it's not always on)
            y_setpoint = WALL_DISTANCE_SETPOINT;
            y_feedback = (ultra_left + ultra_right) / 2.0f;
            pid_y.setGains(y_gains.kp, y_gains.ki, y_gains.kd);
            pid_y.setMaxOutput(y_gains.max_output);
            pid_y.setMaxWindup(y_gains.max_windup);
            vy = -pid_y.update(y_setpoint, y_feedback, 10);

            //--- add different buttons for auto/manual (currently this is just a test for auto)
            if(button.button1) //--- this should be Circle button since it moves side-side
            {
                robot_state++;
            }
            break;
        case 3:
        {
            if(button.button2) //--- this should be X button since it does the homing WHILE the arm takes something
            {
                robot_state = 1;
            }

            bool doPID = true;

            uint8_t left_lost = (ultra_left >= WALL_LOST_THRESHOLD);
            uint8_t right_lost = (ultra_right >= WALL_LOST_THRESHOLD);

            if (!left_lost && !right_lost)
            {
                y_feedback = (ultra_left + ultra_right) / 2.0f;
            }
            else if (!left_lost && right_lost)
            {
                y_feedback = ultra_left;
                
                if(current_direction == MOVING_RIGHT) //-- change direction
                {
                    current_direction = MOVING_LEFT;
                }
            }
            else if (left_lost && !right_lost)
            {
                y_feedback = ultra_right;
                if(current_direction == MOVING_LEFT) //-- change direction
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
                //----- I AM NOT SURE ABOUT THIS, I THINK ITS BETTER TO GO TO STATE 1
                robot_state = 1;
                // vx = 0;
                // vy = 2;
            }
        }    
            break;

        default:
            break;
    }

    if(robot_system.start)
        switch(current_pid)
        {
            case MPU_YAW_PID:
                w_setpoint = 0;
                w_feedback = yaw_degree;
                memcpy(&w_gains, &w_mpu_gains, sizeof(PID_Gains_t));
                pid_w.setGains(w_gains.kp, w_gains.ki, w_gains.kd);
                pid_w.setMaxOutput(w_gains.max_output);
                pid_w.setMaxWindup(w_gains.max_windup);
                vw = pid_w.update_rotate(w_setpoint, w_feedback, 10);
                break;

            case ULTRASONIC_YAW_PID:
                w_setpoint = 0;
                w_feedback = ultra_left - ultra_right;
                memcpy(&w_gains, &w_ultra_gains, sizeof(PID_Gains_t));
                pid_w.setGains(w_gains.kp, w_gains.ki, w_gains.kd);
                pid_w.setMaxOutput(w_gains.max_output);
                pid_w.setMaxWindup(w_gains.max_windup);
                vw = pid_w.update(w_setpoint, w_feedback, 10);
                break;
        }


    ROS_INFO("VX, VY, VW: %d, %d, %d", vx, vy, vw);

    debugPID();

    motor.motor_a = Kinematics_Triangle(0, vx, vy, vw);
    motor.motor_b = Kinematics_Triangle(1, vx, vy, vw);
    motor.motor_c = Kinematics_Triangle(2, vx, vy, vw);

    pub_motor.publish(motor);
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

    tim_100Hz = nh.createTimer(ros::Duration(0.01), timer10msCallback);

    ros::spin();

    return 0;
}
