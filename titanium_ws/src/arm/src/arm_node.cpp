#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "robot_msgs/pid.h"
#include "robot_msgs/motor.h"
#include "robot_msgs/encoder.h"
#include "robot_msgs/limit_switch.h"
#include "robot_msgs/button.h"
#include "robot_msgs/robot_system.h"
#include "robot_msgs/camera.h"
#include "robot_msgs/ultrasonic.h"
#include "robot_msgs/controller.h"
#include "robot_control/PID.hpp"

#define MAX_ROTATION_PULSE 826
#define MAX_HORIZONTAL_PULSE 1800
#define MAX_VERTICAL_PULSE 2080
#define SEARCH_HORIZONTAL_PULSE 900

typedef enum
{
    enc_pid,
    cam_pid
} PID_Type;

PID_Type rot_type = enc_pid;
PID_Type hor_type = enc_pid;

typedef struct 
{
    int16_t setpoint;
    int16_t feedback;
    float dt;
} PID_Input_t ;

PID_Input_t rot_input;
PID_Input_t hor_input;
PID_Input_t ver_input;

typedef struct 
{
    float kp;
    float ki;
    float kd;
    float max_windup;
    float max_output;
    float tolerance;
} PID_Gains_t;

PID_Gains_t rot_enc_gain = {.kp = 0.04, .ki = 0, .kd = 0.01, .max_windup = 5, .max_output = 5, .tolerance = 25};
PID_Gains_t hor_enc_gain = {.kp = 0.06, .ki = 0, .kd = 0, .max_windup = 10, .max_output = 10, .tolerance = 25};
PID_Gains_t ver_enc_gain = {.kp = 0.035, .ki = 0, .kd = 0, .max_windup = 50, .max_output = 50, .tolerance = 50};
// PID_Gains_t rot_cam_gain = {.kp = 0.01, .ki = 0, .kd = 0, .max_windup = 1, .max_output = 1, .tolerance = 10};
// PID_Gains_t hor_cam_gain = {.kp = 0.04, .ki = 0, .kd = 0, .max_windup = 1, .max_output = 1, .tolerance = 10};
PID_Gains_t rot_cam_gain = {.kp = 0.04, .ki = 0, .kd = 0, .max_windup = 1, .max_output = 1, .tolerance = 15};
PID_Gains_t hor_cam_gain = {.kp = 0.06, .ki = 0, .kd = 0, .max_windup = 10, .max_output = 10, .tolerance = 15};

PID_Gains_t rot_gain_pid;
PID_Gains_t hor_gain_pid;
PID_Gains_t ver_gain_pid;

PID rot_pid(0, 0, 0);
PID hor_pid(0, 0, 0);
PID ver_pid(0, 0, 0);


ros::Publisher  pub_rotation,
                pub_horizontal,
                pub_vertical,
                pub_system,
                pub_relay,
                pub_motor;
ros::Subscriber sub_encoder,
                sub_button,
                sub_limit,
                sub_camera,
                sub_controller,
                sub_ultra,
                sub_base;
ros::Timer      tim_system,
                tim_pid;
ros::Time       last_encClbk_time,
                last_ultra_time,
                last_btn_time,
                last_lim_time,
                last_cam_time,
                last_ctrl_time;
std_msgs::Bool base_ready;
robot_msgs::encoder         feedback, feedback_empty;
robot_msgs::pid             rot_debug,
                            hor_debug,
                            ver_debug;
robot_msgs::robot_system    robot_system;
robot_msgs::limit_switch    lim, lim_empty;
robot_msgs::motor           relay, relay_empty;
robot_msgs::motor           motor;
robot_msgs::camera          camera, camera_empty;
robot_msgs::button          button, button_empty;
robot_msgs::controller      controller, controller_empty;
robot_msgs::ultrasonic      ultra, ultra_empty;

int16_t total_rot_enc = 0;
int16_t total_hor_enc = 0;
int16_t total_ver_enc = 0;

uint8_t robot_state = 0;
uint16_t robot_cnt_ms = 0;

const double TIMEOUT = 0.5;
ros::Time last_rot_time;
ros::Time last_hor_time;
ros::Time last_ver_time;
ros::Time last_enc_time;

bool manual_arm = false;

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

float calculate_dt(ros::Time current, ros::Time previous)
{
    float dt = (current - previous).toSec();
    return std::max(dt, 0.001f);
}

void subBaseReadyCallback(const std_msgs::BoolConstPtr &msg)
{
    base_ready = *msg;
}

void subEncoderCallback(const robot_msgs::encoderConstPtr &msg)
{
    last_encClbk_time = ros::Time::now();
    feedback = *msg;
}

void subUltrasonicCallback(const robot_msgs::ultrasonicConstPtr &msg)
{
    last_ultra_time = ros::Time::now();
    ultra = *msg;
}

void subButtonCallback(const robot_msgs::buttonConstPtr &msg)
{
    last_btn_time = ros::Time::now();

    button = *msg;
    
    if(msg->start)
    {
        robot_system.start = 1;
        robot_system.reset = 0;
    }

    if(msg->reset)
    {
        robot_system.start = 0;
        robot_system.reset = 1;
    }

    pub_system.publish(robot_system);
}

void subLimitCallback(const robot_msgs::limit_switchConstPtr &msg)
{
    last_lim_time = ros::Time::now();
    lim = *msg;
}

void subCameraCallback(const robot_msgs::cameraConstPtr msg)
{
    last_cam_time = ros::Time::now();
    camera = *msg;
}

void subControllerCallback(const robot_msgs::controllerConstPtr msg)
{
    last_ctrl_time = ros::Time::now();
    controller = *msg;
}

void timeoutCallback()
{
    if ((ros::Time::now() - last_enc_time).toSec() > TIMEOUT)
    {
        feedback = feedback_empty;
    }

    if ((ros::Time::now() - last_btn_time).toSec() > TIMEOUT)
    {
        button = button_empty;
    }

    if ((ros::Time::now() - last_lim_time).toSec() > TIMEOUT)
    {
        lim = lim_empty;
    }

    if ((ros::Time::now() - last_cam_time).toSec() > TIMEOUT)
    {
        camera = camera_empty;
    }

    if ((ros::Time::now() - last_ctrl_time).toSec() > TIMEOUT)
    {
        controller = controller_empty;
    }

    if ((ros::Time::now() - last_ultra_time).toSec() > TIMEOUT)
    {
        ultra = ultra_empty;
    }
}

void rotationPID(PID_Gains_t *gains, PID_Input_t *input)
{
    rot_pid.setGains(gains->kp, gains->ki, gains->kd);
    rot_pid.setMaxWindup(gains->max_windup);
    rot_pid.setMaxOutput(gains->max_output);

    motor.motor_1 = rot_pid.update(input->setpoint, input->feedback, input->dt);;

    if(abs(rot_pid.getError()) < gains->tolerance)
    {
        motor.motor_1 = 0;
    }

    if(rot_type == cam_pid || manual_arm)
    {
        if(total_rot_enc < -300 && rot_pid.getOutput() <= 0)
        {
            motor.motor_1 = 0;
        }
        if(total_rot_enc > 300 && rot_pid.getOutput() >= 0)
        {
            motor.motor_1 = 0;
        }
    }
}

void horizontalPID(PID_Gains_t *gains, PID_Input_t *input)
{
    hor_pid.setGains(gains->kp, gains->ki, gains->kd);
    hor_pid.setMaxWindup(gains->max_windup);
    hor_pid.setMaxOutput(gains->max_output);
    motor.motor_2 = hor_pid.update(input->setpoint, input->feedback, input->dt);

    if(abs(hor_pid.getError()) < gains->tolerance)
    {
        motor.motor_2 = 0;
    }

    if(hor_type == cam_pid)
    {
        if(total_hor_enc < 200 && hor_pid.getOutput() <= 0)
        {
            motor.motor_2 = 0;
        }
        if(total_hor_enc > 1400 && hor_pid.getOutput() >= 0)
        {
            motor.motor_2 = 0;
        }
    }

    if(manual_arm)
    {
        if(total_hor_enc < 10 && hor_pid.getOutput() <= 0)
        {
            motor.motor_2 = 0;
        }
        if(total_hor_enc > MAX_HORIZONTAL_PULSE && hor_pid.getOutput() >= 0)
        {
            motor.motor_2 = 0;
        }
    }
}

void verticalPID(PID_Gains_t *gains, PID_Input_t *input)
{
    ver_pid.setGains(gains->kp, gains->ki, gains->kd);
    ver_pid.setMaxWindup(gains->max_windup);
    ver_pid.setMaxOutput(gains->max_output);
    motor.motor_3 = ver_pid.update(input->setpoint, input->feedback, input->dt);

    if(abs(ver_pid.getError()) < gains->tolerance)
    {
        motor.motor_3 = 0;
    }
}

void debugPID()
{
    rot_debug.kp = rot_pid.getKp();
    rot_debug.ki = rot_pid.getKi();
    rot_debug.kd = rot_pid.getKd();
    rot_debug.proportional = rot_pid.getProportional();
    rot_debug.integral = rot_pid.getIntegral();
    rot_debug.derivative = rot_pid.getDerivative();
    rot_debug.error = rot_pid.getError();
    rot_debug.prev_error = rot_pid.getPrevError();
    rot_debug.setpoint = rot_pid.getSetpoint();
    rot_debug.feedback = rot_pid.getFeedback();
    rot_debug.max_output = rot_pid.getMaxOutput();
    rot_debug.max_windup = rot_pid.getMaxWindup();
    // rot_debug.output = motor.motor_1;
    rot_debug.output = rot_pid.getOutput();
    rot_debug.tolerance = rot_gain_pid.tolerance;

    hor_debug.kp = hor_pid.getKp();
    hor_debug.ki = hor_pid.getKi();
    hor_debug.kd = hor_pid.getKd();
    hor_debug.proportional = hor_pid.getProportional();
    hor_debug.integral = hor_pid.getIntegral();
    hor_debug.derivative = hor_pid.getDerivative();
    hor_debug.error = hor_pid.getError();
    hor_debug.prev_error = hor_pid.getPrevError();
    hor_debug.setpoint = hor_pid.getSetpoint();
    hor_debug.feedback = hor_pid.getFeedback();
    hor_debug.max_output = hor_pid.getMaxOutput();
    hor_debug.max_windup = hor_pid.getMaxWindup();
    // hor_debug.output = motor.motor_2;
    hor_debug.output = hor_pid.getOutput();
    hor_debug.tolerance = hor_gain_pid.tolerance;

    ver_debug.kp = ver_pid.getKp();
    ver_debug.ki = ver_pid.getKi();
    ver_debug.kd = ver_pid.getKd();
    ver_debug.proportional = ver_pid.getProportional();
    ver_debug.integral = ver_pid.getIntegral();
    ver_debug.derivative = ver_pid.getDerivative();
    ver_debug.error = ver_pid.getError();
    ver_debug.prev_error = ver_pid.getPrevError();
    ver_debug.setpoint = ver_pid.getSetpoint();
    ver_debug.feedback = ver_pid.getFeedback();
    ver_debug.max_output = ver_pid.getMaxOutput();
    ver_debug.max_windup = ver_pid.getMaxWindup();
    // ver_debug.output = motor.motor_3;
    ver_debug.output = ver_pid.getOutput();
    ver_debug.tolerance = ver_gain_pid.tolerance;

    pub_rotation.publish(rot_debug);
    pub_horizontal.publish(hor_debug);
    pub_vertical.publish(ver_debug);
}

void timSystemCallback1ms(const ros::TimerEvent &event)
{
    if(!robot_system.start)
    {
        return;
    }

    timeoutCallback();

    static int16_t cnt_ms = 0;

    switch (robot_state)
    {
        case 0: //--- go up
            relay.relay_state = 0;

            if(ver_input.setpoint <= MAX_VERTICAL_PULSE - 5)
            {
                ver_input.setpoint += 5; 
            }

            if(abs(MAX_VERTICAL_PULSE - ver_input.feedback) < ver_gain_pid.tolerance)
            {
                robot_state++;
            }
            break;

        case 1: //--- hold rotation

            rot_input.setpoint = 0;

            if(abs(rot_pid.getError()) < rot_gain_pid.tolerance)
            {
                robot_state++;
            }
            break;
        
        case 2://--- go forward
            hor_input.setpoint = SEARCH_HORIZONTAL_PULSE;

            if(abs(hor_pid.getError()) < hor_gain_pid.tolerance)
            {
                robot_state++;
            }
            break;

        case 3:
            if(controller.option)
            {
                manual_arm = true;
                robot_state = 99;
            }
            if(controller.down)
            {
                hor_pid.reset();
                rot_pid.reset();
                hor_type = cam_pid;
                rot_type = cam_pid;
                robot_state++;
            }
            break;
        case 4:
            if(controller.up)
            {
                hor_pid.reset();
                rot_pid.reset();
                hor_type = enc_pid;
                rot_type = enc_pid;
                robot_state = 0;
            }
            
            if(camera.trashDetected)
            {
                rot_input.setpoint = 40;
                hor_input.setpoint = 40;
                if(abs(rot_pid.getError()) <= rot_gain_pid.tolerance && abs(hor_pid.getError()) <= hor_gain_pid.tolerance)
                {
                    if(robot_cnt_ms >= 500)
                    {
                        hor_pid.reset();
                        rot_pid.reset();
                        hor_type = enc_pid;
                        rot_type = enc_pid;
                        robot_state++;
                        robot_cnt_ms = 0;
                    }
                    else
                    {
                        robot_cnt_ms++;
                    }
                }
            }
            else
            {
                rot_input.setpoint = rot_input.feedback;
                hor_input.setpoint = hor_input.feedback;
            }

            break;

        case 5:
            if(controller.up)
            {
                robot_state = 0;
            }

            rot_input.setpoint = rot_input.feedback;
            hor_input.setpoint = hor_input.feedback;

            if(ver_input.feedback < 250)
            {
                relay.relay_state = 1;
            }

            if(ver_input.setpoint >= 15)
            {
                ver_input.setpoint -= 5; 
            }

            if(lim.lim_3 == 0)
            {
                ver_input.setpoint = total_ver_enc = 0;
                ver_pid.reset();
            }

            if(abs(10 - ver_input.feedback) < ver_gain_pid.tolerance) //--- hit limit switch
            {
                if(lim.lim_3 == 0)
                {
                    ver_input.setpoint = total_ver_enc = 0;
                    ver_pid.reset();
                }

                if(robot_cnt_ms >= 249)
                {
                    robot_state++;
                    robot_cnt_ms = 0;
                }
                else
                {
                    robot_cnt_ms++;
                }
                
            }
            break;
        
        case 6:
            if(ver_input.setpoint <= MAX_VERTICAL_PULSE - 5)
            {
                ver_input.setpoint += 5; 
            }

            if(abs(MAX_VERTICAL_PULSE - ver_input.feedback) < ver_gain_pid.tolerance)
            {
                robot_state++;
            }

            break;
        
        case 7:
            rot_input.setpoint = 0;
            hor_input.setpoint = MAX_HORIZONTAL_PULSE;
            if(abs(rot_pid.getError()) <= rot_gain_pid.tolerance && abs(hor_pid.getError()) <= hor_gain_pid.tolerance)
            {
                robot_state++;
            }
            break;
            
        case 8:

            if(controller.down)
            {
                robot_state++;
            }
            if(controller.up)
            {
                robot_state = 0;
            }
            break;

        case 9:
            if(ver_input.setpoint >= 15)
            {
                ver_input.setpoint -= 5; 
            }

            relay.relay_state = 0;

            if(lim.lim_3 == 0)
            {
                ver_input.setpoint = total_ver_enc = 0;
                ver_pid.reset();
            }

            if(abs(10 - ver_input.feedback) < ver_gain_pid.tolerance) //--- hit limit switch
            {
                if(robot_cnt_ms >= 2500)
                {
                    robot_state = 0;
                    robot_cnt_ms = 0;
                }
                else
                {
                    robot_cnt_ms++;
                }
            }
            break;
        
        case 99: //--- manual only
        {
            static float rot_scaled = 0;
            static float hor_scaled = 0;
            static uint8_t grab_state = 0;
            static uint8_t SUCK_state = 0;
            static uint8_t SUCK_TIME = 0;

            if(lim.lim_2 == 0)
            {
                total_hor_enc = 0;
                hor_pid.reset();
            }

            if(controller.touchpad)
            {
                rot_input.setpoint = total_rot_enc = 0;
                rot_pid.reset();
                rot_scaled = 0;
            }

            switch (SUCK_state)
            {
                case 0:
                    if(controller.r1)
                    {
                        SUCK_state++;
                    }
                    break;
                case 1:
                    relay.relay_state = 1;
                    if(SUCK_TIME >= 250)
                    {
                        SUCK_state++;
                        SUCK_TIME = 0;
                    }
                    else
                    {
                        SUCK_TIME++;
                    }
                    break;
                case 2:
                    if(controller.r1)
                    {
                        SUCK_state++;
                    }
                    break;
                case 3:
                    relay.relay_state = 0;
                    if(SUCK_TIME >= 250)
                    {
                        SUCK_state = 0;
                        SUCK_TIME = 0;
                    }
                    else
                    {
                        SUCK_TIME++;
                    }
                    break;

            }
            

            if(controller.left && rot_scaled < 300)
            {
                rot_scaled += 0.2;
            }
            else if(controller.right && rot_scaled > -300)
            {
                rot_scaled -= 0.2;
            }

            if(controller.up && hor_scaled < MAX_HORIZONTAL_PULSE)
            {
                hor_scaled += 1;
            }
            else if(controller.down && hor_scaled > 10)
            {
                hor_scaled -= 1;
            }

            rot_input.setpoint = (int16_t)rot_scaled;
            hor_input.setpoint = (int16_t)hor_scaled;

            switch(grab_state)
            {
                case 0:
                    if(controller.l1)
                    {
                        grab_state++;
                    }
                    break;

                case 1:

                    if(ver_input.setpoint >= 15)
                    {
                        ver_input.setpoint -= 5; 
                    }

                    if(lim.lim_3 == 0)
                    {
                        total_ver_enc = 0;
                        ver_pid.reset();
                    }

                    if(abs(10 - ver_input.feedback) < ver_gain_pid.tolerance) //--- hit limit switch
                    {
                        if(robot_cnt_ms >= 250)
                        {
                            grab_state++;
                            robot_cnt_ms = 0;
                        }
                        else
                        {
                            robot_cnt_ms++;
                        } 
                    }
                    break;
                
                case 2:
                    if(controller.l1)
                    {
                        grab_state++;
                    }
                    break;

                case 3:
                    if(ver_input.setpoint <= MAX_VERTICAL_PULSE - 5)
                    {
                        ver_input.setpoint += 5; 
                    }

                    if(abs(MAX_VERTICAL_PULSE - ver_input.feedback) < ver_gain_pid.tolerance)
                    {
                        if(robot_cnt_ms >= 250)
                        {
                            grab_state = 0;
                            robot_cnt_ms = 0;
                        }
                        else
                        {
                            robot_cnt_ms++;
                        } 
                    }
                    break;
            }

            if(controller.share)
            {
                manual_arm = false;
                rot_scaled = 0;
                hor_scaled = 0;
                grab_state = 0;
                SUCK_state = 0;
                rot_pid.reset();
                hor_pid.reset();
                rot_pid.reset();

                robot_state = 0;
            }

            break;
        }


        default:
            break;
    }
    pub_relay.publish(relay);
}

void timPID1ms(const ros::TimerEvent &event)
{
    if(!robot_system.start)
    {
        last_rot_time = ros::Time::now();
        last_hor_time = ros::Time::now();
        last_ver_time = ros::Time::now();
        last_enc_time = ros::Time::now();
        return;
    }

    ros::Time current_time = ros::Time::now();

    //--- Get encoder feedbacks every 10ms using actual time
    if((current_time - last_enc_time).toSec() * 1000.0 >= 10.0)
    {
        total_rot_enc += feedback.enc_1;
        total_hor_enc += feedback.enc_2;
        total_ver_enc += feedback.enc_3;
        last_enc_time = current_time;
    }
    
    //--- reset encoder readings if hit limit switch
    if(lim.lim_2 == 0)
    {
        total_hor_enc = 0;
    }
    if(lim.lim_3 == 0)
    {
        total_ver_enc = 0;
    }

    //--- choose pid types
    switch(rot_type)
    {
        case enc_pid:
            rot_input.feedback = total_rot_enc;
            memcpy(&rot_gain_pid, &rot_enc_gain, sizeof(PID_Gains_t));
            break;
        case cam_pid:
            rot_input.feedback = camera.closestTrashX;
            memcpy(&rot_gain_pid, &rot_cam_gain, sizeof(PID_Gains_t));
            break;
    }

    switch(hor_type)
    {
        case enc_pid:
            hor_input.feedback = total_hor_enc;
            memcpy(&hor_gain_pid, &hor_enc_gain, sizeof(PID_Gains_t));
            break;
        case cam_pid:
            hor_input.feedback = camera.closestTrashY;
            memcpy(&hor_gain_pid, &hor_cam_gain, sizeof(PID_Gains_t));
            break;
    }
    
    ver_input.feedback = total_ver_enc;
    memcpy(&ver_gain_pid, &ver_enc_gain, sizeof(PID_Gains_t));

    //--- execute pid based on actual time intervals
    if((current_time - last_rot_time).toSec() * 1000.0 >= 10.0)
    {
        rot_input.dt = calculate_dt(current_time, last_rot_time);
        rotationPID(&rot_gain_pid, &rot_input);
        last_rot_time = current_time;
    }
    
    if((current_time - last_hor_time).toSec() * 1000.0 >= 10.0)
    {
        hor_input.dt = calculate_dt(current_time, last_hor_time);
        horizontalPID(&hor_gain_pid, &hor_input);
        last_hor_time = current_time;
    }
   
    if((current_time - last_ver_time).toSec() * 1000.0 >= 10.0)
    {
        ver_input.dt = calculate_dt(current_time, last_ver_time);
        verticalPID(&ver_gain_pid, &ver_input);
        last_ver_time = current_time;
    }

    ROS_INFO("R-H-V dt: %.5f %.5f, %.5f", rot_input.dt, hor_input.dt, ver_input.dt);

    debugPID();
    
    pub_motor.publish(motor);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_node");
    ros::NodeHandle nh;

    ROS_INFO("ARM NODE STARTED!!!");

    ros::AsyncSpinner spinner(0);

    pub_rotation = nh.advertise<robot_msgs::pid>("/pid/debug/arm/rotation", 10);
    pub_horizontal = nh.advertise<robot_msgs::pid>("/pid/debug/arm/horizontal", 10);
    pub_vertical = nh.advertise<robot_msgs::pid>("/pid/debug/arm/vertical", 10);
    pub_system = nh.advertise<robot_msgs::robot_system>("/system/robot_system", 10);
    pub_relay = nh.advertise<robot_msgs::motor>("/actuator/relay", 10);
    pub_motor = nh.advertise<robot_msgs::motor>("/actuator/motor/arm", 10);

    sub_encoder = nh.subscribe("/sensor/encoder", 10, subEncoderCallback);
    sub_button = nh.subscribe("/input/button", 10, subButtonCallback);
    sub_limit = nh.subscribe("/sensor/limit_switch", 10, subLimitCallback);
    sub_camera = nh.subscribe("/sensor/camera", 10, subCameraCallback);
    sub_controller = nh.subscribe("/input/controller", 10, subControllerCallback);
    sub_ultra = nh.subscribe("/sensor/ultrasonic", 10, subUltrasonicCallback);
    sub_base = nh.subscribe("/base/ready_state", 10, subBaseReadyCallback);

    tim_system = nh.createTimer(ros::Duration(0.001), timSystemCallback1ms);
    tim_pid = nh.createTimer(ros::Duration(0.001), timPID1ms);

    spinner.start();

    ros::waitForShutdown();

    spinner.stop();

    // ros::spin();

    return 0;
}
