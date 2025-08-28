#include "ros/ros.h"
#include "robot_msgs/motor.h"
#include "robot_msgs/encoder.h"
#include "robot_msgs/limit_switch.h"
#include "robot_msgs/camera.h"
#include "robot_control/PID.hpp"

#define MAX_ROTATION_PULSE      1840
#define MAX_HORIZONTAL_PULSE    2115
#define MAX_VERTICAL_PULSE      2465

typedef struct 
{
    float setpoint;
    float feedback;
    float max_output;
    float dt;
} PID_Params;


ros::Publisher pub_motor;
ros::Subscriber sub_limit_sw;
ros::Subscriber sub_encoder;
ros::Subscriber sub_camera;
ros::Timer timer_1ms;
robot_msgs::motor motor;
robot_msgs::limit_switch lim;
robot_msgs::encoder encoder;
robot_msgs::camera camera;

int16_t encoder_time_ms = 0;
int16_t total_encoder[3] = {0};

PID arm_rotation(0, 0, 0);
PID arm_horizontal(0, 0, 0);
PID arm_vertical(0, 0, 0);

PID_Params param_rotation = {0};
PID_Params param_horizontal = {0};
PID_Params param_vertical = {0};

uint8_t robot_state = 0;
int16_t robot_time_ms = 0;

int set = 0;
int st = 0;

void timer1msCallback(const ros::TimerEvent &event)
{
    if(lim.lim_2 == 0)
    {
        total_encoder[1] = 0;
    }
    if(lim.lim_3 == 0)
    {
        total_encoder[2] = 0;
    }

    if(encoder_time_ms >= 9)
    {
        total_encoder[0] += encoder.enc_1;
        total_encoder[1] += encoder.enc_2;
        total_encoder[2] += encoder.enc_3;
        param_vertical.feedback = total_encoder[2];
        encoder_time_ms = 0;
    }
    encoder_time_ms++;

    // //--- vertical (260, max - 100)
    // //--- horizontal (25, max - 25)
    // if(robot_time_ms >= 2999)
    // {
    //     switch(st)
    //     {
    //         case 0:
    //             param_horizontal.setpoint = MAX_HORIZONTAL_PULSE - 25;
    //             st++;
    //             break;
    //         case 1:
    //             param_horizontal.setpoint = 25;
    //             st = 0;
    //             break;
    //     }

    //     robot_time_ms = 0;
    // }
    // robot_time_ms++;

    switch(robot_state)
    {
        case 0:
            arm_rotation.setGains(1, 0, 0);
            arm_horizontal.setGains(1, 0, 0);
            arm_vertical.setGains(1, 0, 0);

            param_rotation.dt = 10;
            param_horizontal.dt = 10;
            param_vertical.dt = 10;
            
            robot_state++;
            
            break;
        case 1:
            param_vertical.setpoint = MAX_VERTICAL_PULSE - 100;
            param_vertical.feedback = total_encoder[2];
            param_vertical.max_output = 900;

            if(abs((int)arm_vertical.getError()) <= 50)
            {
                if(robot_time_ms >= 500)
                {
                    robot_state++;
                    robot_time_ms = 0;
                }
                robot_time_ms++;
            }

            break;

        case 2:
            param_horizontal.setpoint = MAX_HORIZONTAL_PULSE/2;
            param_horizontal.feedback = total_encoder[1];
            param_horizontal.max_output = 500;
            
            if(abs((int)arm_horizontal.getError()) <= 50)
            {
                if(robot_time_ms >= 500)
                {
                    robot_state++;
                    robot_time_ms = 0;
                }
                robot_time_ms++;
            }

            break;
        case 3:
            arm_rotation.setGains(2, 0, 0);
            arm_horizontal.setGains(30, 0, 0);
            
            robot_state++;
            break;

        case 4:
            if(camera.trashDetected)
            {
                param_rotation.setpoint = 0;
                param_rotation.feedback = camera.closestTrashX;
                param_rotation.max_output = 150;

                param_horizontal.setpoint = 0;
                param_horizontal.feedback = camera.closestTrashY;
                param_horizontal.max_output = 200;
            }
            else
            {
                param_rotation.max_output = 0;
                param_horizontal.max_output = 0;
            }
            break;

        default:
            break;
    }

    motor.motor_1 = (int16_t)arm_rotation.update(param_rotation.setpoint, param_rotation.feedback, 
                                        param_rotation.max_output, param_rotation.dt);
    motor.motor_2 = (int16_t)arm_horizontal.update(param_horizontal.setpoint, param_horizontal.feedback, 
                                        param_horizontal.max_output, param_horizontal.dt);
    motor.motor_3 = (int16_t)arm_vertical.update(param_vertical.setpoint, param_vertical.feedback, 
                                        param_vertical.max_output, param_vertical.dt);

    if(total_encoder[1] >= 1600)  
    {
        motor.motor_2 = (motor.motor_2 < 0) ? motor.motor_2 : 0;  //--- Negative only
    }
    else if(total_encoder[1] <= 500)
    {
        motor.motor_2 = (motor.motor_2 > 0) ? motor.motor_2 : 0;  //--- Positive only
    }

        ROS_INFO_STREAM( "\n" <<
        "Setpoint: " << param_rotation.setpoint << "\n"
        "Feedback: " << param_rotation.feedback << "\n"
        "Error: " << arm_rotation.getError() << "\n"
        "Output: " << arm_rotation.getOutput() << "\n"
        "Total encoder: " << total_encoder[0];
    );
                                        
    // ROS_INFO_STREAM( "\n" <<
    //     "Setpoint: " << param_horizontal.setpoint << "\n"
    //     "Feedback: " << param_horizontal.feedback << "\n"
    //     "Error: " << arm_horizontal.getError() << "\n"
    //     "Output: " << arm_horizontal.getOutput() << "\n"
    //     "Total encoder: " << total_encoder[1];
    // );

    //     ROS_INFO_STREAM( "\n" <<
    //     "Setpoint: " << param_vertical.setpoint << "\n"
    //     "Feedback: " << param_vertical.feedback << "\n"
    //     "Error: " << arm_vertical.getError() << "\n"
    //     "Output: " << arm_vertical.getOutput()
    // );

    pub_motor.publish(motor);
}

void subLimitCallback(const robot_msgs::limit_switchConstPtr msg)
{
    lim = *msg;
}

void subEncoderCallback(const robot_msgs::encoderConstPtr msg)
{
    encoder.enc_1 = msg->enc_1;
    encoder.enc_2 = msg->enc_2;
    encoder.enc_3 = msg->enc_3;
}

void subCameraCallback(const robot_msgs::cameraConstPtr msg)
{
    camera = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_test_node");
    
    ros::NodeHandle nh;

    pub_motor = nh.advertise<robot_msgs::motor>("/actuator/motor", 10);

    sub_limit_sw = nh.subscribe("/sensor/limit_switch", 10, subLimitCallback);
    sub_encoder = nh.subscribe("/sensor/encoder", 10, subEncoderCallback);
    sub_camera = nh.subscribe("/sensor/camera", 10, subCameraCallback);

    timer_1ms = nh.createTimer(ros::Duration(0.001), timer1msCallback);

    ROS_INFO("Arm Test Node Started!");

    ros::spin();
    
    return 0;
}