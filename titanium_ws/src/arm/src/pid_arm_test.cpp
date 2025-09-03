#include "ros/ros.h"
#include "robot_msgs/pid.h"
#include "robot_msgs/motor.h"
#include "robot_msgs/encoder.h"
#include "robot_msgs/limit_switch.h"
#include "robot_msgs/button.h"
#include "robot_msgs/robot_system.h"
#include "robot_control/PID.hpp"
#include "robot_msgs/camera.h"
#include "robot_msgs/controller.h"


#define MAX_ROTATION_PULSE 826
#define MAX_HORIZONTAL_PULSE 1850
#define MAX_VERTICAL_PULSE 2100
#define SEARCH_HORIZONTAL_PULSE 900

typedef struct 
{
    float kp;
    float ki;
    float kd;
    float max_windup;
    float max_output;
    float tolerance;
} PID_Gains_t;

PID_Gains_t rot_enc_pid = {.kp = 0.04, .ki = 0, .kd = 0.01, .max_windup = 5, .max_output = 5, .tolerance = 25};
PID_Gains_t hor_enc_pid = {.kp = 0.06, .ki = 0, .kd = 0, .max_windup = 10, .max_output = 10, .tolerance = 25};
PID_Gains_t ver_enc_pid = {.kp = 0.04, .ki = 0, .kd = 0, .max_windup = 50, .max_output = 50, .tolerance = 25};
PID_Gains_t rot_cam_pid = {.kp = 0.04, .ki = 0, .kd = 0, .max_windup = 10, .max_output = 1, .tolerance = 15};
PID_Gains_t hor_cam_pid = {.kp = 0.06, .ki = 0, .kd = 0, .max_windup = 10, .max_output = 10, .tolerance = 15};
PID_Gains_t rot_gain_pid;
PID_Gains_t hor_gain_pid;
PID_Gains_t ver_gain_pid;

ros::Publisher pub_rotation, pub_horizontal, pub_vertical, pub_system, pub_relay;
ros::Subscriber sub_encoder, sub_button, sub_limit, sub_camera, sub_controller;
ros::Timer tim_1kHz, tim_100Hz;

robot_msgs::encoder feedback;
robot_msgs::pid rotation_param, horizontal_param, vertical_param;
robot_msgs::robot_system robot_system;
robot_msgs::limit_switch lim;
robot_msgs::motor relay;
robot_msgs::camera camera;
robot_msgs::button button;
robot_msgs::controller controller;

enum ArmCommand : uint8_t {
    none,
    pickplace,
    cancel
};

int16_t total_enc_rotation = 0;
int16_t total_enc_horizontal = 0;
int16_t total_enc_vertical = 0;

enum ArmState : uint8_t {
    wait_start,
    init_vert,
    init_rot,
    init_hor,
    ready,
    pick_search,
    pick_down,
    pick_up,
    pick_extend,
    holding,
    place_down,
};

ArmState robot_arm_state = wait_start;

ArmCommand parseArmCommand (robot_msgs::controller c) {
    if (c.crs) return pickplace;
    if (c.sqr) return cancel;
    return none;
}

ArmState processArmState (ArmCommand cmd, ArmState s) {
    if (cmd == pickplace) {
        if (s == ready) return pick_search;
        if (s == holding) return place_down;
    }
    if (cmd == cancel) {
        if (s >= pick_search && s <= pick_down ||
            s == holding) return init_vert;
    }
    if (cmd == none) {
        return s;
    }
    return s;
}

int16_t robot_cnt_ms = 0;

int16_t rotation_setpoint = 0;
int16_t horizontal_setpoint = 0;
int16_t vertical_setpoint = 0;
int16_t rotation_feedback = 0;
int16_t horizontal_feedback = 0;
int16_t vertical_feedback = 0;
int16_t rotation_tolerance = 0;
int16_t horizontal_tolerance = 0;
int16_t vertical_tolerance = 0;

void setPID(robot_msgs::pid &pid_msg, PID_Gains_t *gain, float setpoint, float feedback)
{
    pid_msg.kp = gain->kp;
    pid_msg.ki = gain->ki;
    pid_msg.kd = gain->kd;
    pid_msg.setpoint = setpoint; //--- 0 to 2100
    pid_msg.feedback = feedback;
    pid_msg.max_output = gain -> max_output;
    pid_msg.max_windup = gain -> max_windup;
    pid_msg.tolerance = gain ->tolerance;
}

void encoderCallback(const robot_msgs::encoderConstPtr &msg)
{
    feedback = *msg;
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

    pub_system.publish(robot_system);
}

void limitCallback(const robot_msgs::limit_switchConstPtr &msg)
{
    lim = *msg;
}

void subCameraCallback(const robot_msgs::cameraConstPtr msg)
{
    camera = *msg;
}

void subControllerCallback(const robot_msgs::controllerConstPtr msg)
{
    controller = *msg;
}

void timer10msCallback(const ros::TimerEvent &event)
{
    total_enc_rotation += feedback.enc_1;
    total_enc_horizontal += feedback.enc_2;
    total_enc_vertical += feedback.enc_3;
    
    if(lim.lim_2 == 0)
    {
        total_enc_horizontal = 0;
    }

    if(lim.lim_3 == 0)
    {
        total_enc_vertical = 0;
        vertical_param.resetPID = true;
    }
    else
    {
        vertical_param.resetPID = false;
    }
}

void timer1msCallback(const ros::TimerEvent &event)
{
    //--- parse the controller arm command
    ArmCommand command = parseArmCommand(controller);

    //--- check for input change
    if (command != none) {
        robot_arm_state = processArmState(command, robot_arm_state);
    }

    //--- execute current action
    switch(robot_arm_state)
    {
        case wait_start: //--- wait until start button
            if(robot_system.start)
            {
                robot_arm_state = init_vert;
            }
            break;
        case init_vert: //--- go up
            if (relay.relay_state != 0) {
                relay.relay_state = 0;
            }

            memcpy(&rot_gain_pid, &rot_enc_pid, sizeof(PID_Gains_t));
            memcpy(&hor_gain_pid, &hor_enc_pid, sizeof(PID_Gains_t));
            memcpy(&ver_gain_pid, &ver_enc_pid, sizeof(PID_Gains_t));
            
            if(vertical_setpoint <= MAX_VERTICAL_PULSE - 5)
            {
                vertical_setpoint += 5; 
            } 
            
            rotation_feedback = total_enc_rotation;
            horizontal_feedback = total_enc_horizontal;
            vertical_feedback = total_enc_vertical;

            if((MAX_VERTICAL_PULSE - vertical_feedback) < ver_enc_pid.tolerance)
            {
                robot_arm_state = init_rot;
            }
            break;
        case init_rot: //--- ensure rotation at 0
            rotation_setpoint = 0;
            rotation_feedback = total_enc_rotation;
            horizontal_feedback = total_enc_horizontal;
            vertical_feedback = total_enc_vertical;

            if(abs(rotation_setpoint - rotation_feedback) < rot_enc_pid.tolerance)
            {
                robot_arm_state = init_hor;
            }
            break;
        case init_hor: //--- go forward
            horizontal_setpoint = SEARCH_HORIZONTAL_PULSE;
            
            rotation_feedback = total_enc_rotation;
            horizontal_feedback = total_enc_horizontal;
            vertical_feedback = total_enc_vertical;

            if(abs(horizontal_setpoint - horizontal_feedback) < hor_enc_pid.tolerance)
            {
                rot_gain_pid.max_output = rot_gain_pid.max_windup = 0;
                hor_gain_pid.max_output = hor_gain_pid.max_windup = 0;
                ver_gain_pid.max_output = ver_gain_pid.max_windup = 0;
                robot_arm_state = ready;
            }
            break;
        case pick_search: //---- SEARCH MODE
            memcpy(&rot_gain_pid, &rot_cam_pid, sizeof(PID_Gains_t));
            memcpy(&hor_gain_pid, &hor_cam_pid, sizeof(PID_Gains_t));
            memcpy(&ver_gain_pid, &ver_enc_pid, sizeof(PID_Gains_t));
            
            vertical_setpoint = MAX_VERTICAL_PULSE; 
            vertical_feedback = total_enc_vertical;

            if(camera.trashDetected)
            {
                rotation_setpoint = 40;
                horizontal_setpoint = 40;
                rotation_feedback = camera.closestTrashX;
                horizontal_feedback = camera.closestTrashY;
            }
            else
            {
                rot_gain_pid.max_output = rot_gain_pid.max_windup = 0;
                hor_gain_pid.max_output = hor_gain_pid.max_windup = 0;
            }

            if(abs(rotation_setpoint - rotation_feedback) <= rot_cam_pid.tolerance && abs(horizontal_setpoint - horizontal_feedback) <= hor_cam_pid.tolerance)
            {
                rot_gain_pid.max_output = rot_gain_pid.max_windup = 0;
                hor_gain_pid.max_output = hor_gain_pid.max_windup = 0;
                robot_arm_state = pick_down;
            }

            break;

        case pick_down: //--- go down

            if(vertical_setpoint >= 25)
            {
                vertical_setpoint -= 5; 
            }
            vertical_feedback = total_enc_vertical;

            if(vertical_feedback < 250)
            {
                relay.relay_state = 1;
            }

            if(vertical_feedback < 30) //--- hit limit switch
            {
                if(lim.lim_3 == 0)
                {
                    vertical_setpoint = 0;
                }
                if(robot_cnt_ms >= 249)
                {
                    robot_arm_state = pick_up;
                    robot_cnt_ms = 0;
                }
                robot_cnt_ms++;
            }
            break;

        case pick_up: //--- go up
            if(vertical_setpoint <= MAX_VERTICAL_PULSE - 5)
            {
                vertical_setpoint += 5; 
            }
            vertical_feedback = total_enc_vertical;

            if((MAX_VERTICAL_PULSE - vertical_feedback) < ver_enc_pid.tolerance)
            {
                robot_arm_state = pick_extend;
            }
            break;
        case pick_extend: //--- extend, get ready for trash bin
            memcpy(&rot_gain_pid, &rot_enc_pid, sizeof(PID_Gains_t));
            memcpy(&hor_gain_pid, &hor_enc_pid, sizeof(PID_Gains_t));
            
            rotation_setpoint = 0;
            horizontal_setpoint = MAX_HORIZONTAL_PULSE;

            rotation_feedback = total_enc_rotation;
            horizontal_feedback = total_enc_horizontal;

            if(abs(rotation_setpoint - rotation_feedback) <= rot_enc_pid.tolerance && abs(horizontal_setpoint - horizontal_feedback) <= hor_enc_pid.tolerance)
            {
                rot_gain_pid.max_output = rot_gain_pid.max_windup = 0;
                hor_gain_pid.max_output = hor_gain_pid.max_windup = 0;
                ver_gain_pid.max_output = ver_gain_pid.max_windup = 0;
                robot_arm_state = holding;
            }

            break;
        case place_down: //--- go down
            memcpy(&ver_gain_pid, &ver_enc_pid, sizeof(PID_Gains_t));
            rot_gain_pid.max_output = rot_gain_pid.max_windup = 0;
            hor_gain_pid.max_output = hor_gain_pid.max_windup = 0;
            
            if(vertical_setpoint >= 25)
            {
                vertical_setpoint -= 5; 
            }
            vertical_feedback = total_enc_vertical;
            
            relay.relay_state = 0;


            if(vertical_feedback < 30) //--- hit limit switch
            {
                if(lim.lim_3 == 0)
                {
                    vertical_setpoint = 0;
                }
                if(robot_cnt_ms >= 2500)
                {
                    robot_arm_state = init_vert;
                    robot_cnt_ms = 0;
                }
                robot_cnt_ms++;
            }
          
            break;
        // case 9:
        //     rot_gain_pid.max_output = rot_gain_pid.max_windup = 0;
        //     hor_gain_pid.max_output = hor_gain_pid.max_windup = 0;
        //     ver_gain_pid.max_output = ver_gain_pid.max_windup = 0;
        //     if(button.startButton == 0)
        //     {
        //         robot_state = 1;
        //     }
        //     break;

    }

    setPID(rotation_param, &rot_gain_pid, rotation_setpoint, rotation_feedback);
    setPID(horizontal_param, &hor_gain_pid, horizontal_setpoint, horizontal_feedback);
    setPID(vertical_param, &ver_gain_pid, vertical_setpoint, vertical_feedback);

    pub_rotation.publish(rotation_param);
    pub_horizontal.publish(horizontal_param);
    pub_vertical.publish(vertical_param);
    pub_relay.publish(relay);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pid_arm_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle priv_nh("~");

    double setpoint, kp, ki, kd;
    priv_nh.param<double>("setpoint", setpoint, 0.0);
    priv_nh.param<double>("kp", kp, 0.0);
    priv_nh.param<double>("ki", ki, 0.0);
    priv_nh.param<double>("kd", kd, 0.0);

    ROS_INFO("PID Arm Test Node Started!");

    pub_rotation = nh.advertise<robot_msgs::pid>("/pid/param/arm/rotation", 10);
    pub_horizontal = nh.advertise<robot_msgs::pid>("/pid/param/arm/horizontal", 10);
    pub_vertical = nh.advertise<robot_msgs::pid>("/pid/param/arm/vertical", 10);
    pub_system = nh.advertise<robot_msgs::robot_system>("/system/robot_system", 10);
    pub_relay = nh.advertise<robot_msgs::motor>("/actuator/relay", 10);

    sub_encoder = nh.subscribe("/sensor/encoder", 10, encoderCallback);
    sub_button = nh.subscribe("/input/button", 10, buttonCallback);
    sub_limit = nh.subscribe("/sensor/limit_switch", 10, limitCallback);
    sub_camera = nh.subscribe("/sensor/camera", 10, subCameraCallback);
    sub_controller = nh.subscribe("/input/controller", 10, subControllerCallback);
    tim_1kHz = nh.createTimer(ros::Duration(0.001), timer1msCallback);
    tim_100Hz = nh.createTimer(ros::Duration(0.01), timer10msCallback);

    ros::spin();

    return 0;
}
