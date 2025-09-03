#include "ros/ros.h"
#include "robot_msgs/controller.h"
#include "robot_msgs/motor.h"

ros::Publisher pub_motor, pub_relay;
ros::Subscriber sub_controller;
ros::Timer tim_1kHz;

robot_msgs::controller controller;
robot_msgs::motor motor;
robot_msgs::motor relay;

uint8_t relay_state = 0;

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

void subControllerCallback(const robot_msgs::controllerConstPtr &msg)
{
    controller = *msg;
}

void relay_toggle()
{
    static uint8_t toggle_state = 0;
    static uint16_t cnt_ms = 0;

    switch(toggle_state)
    {
        case 0:
            if(controller.crs)
            {
                ROS_INFO("Relay: %d", relay_state);
                relay_state = !relay_state;
                toggle_state++;
            }
            break;
        case 1:
            if(cnt_ms >= 250)
            {
                toggle_state = 0;
                cnt_ms = 0;
            }
            cnt_ms++;
            break;
    }

    relay.relay_state = relay_state;
}

void timer1msCallback(const ros::TimerEvent &event)
{
    if(controller.left)
    {
        motor.motor_1 = 2;
        ROS_INFO("R1 PRESSED");
    }
    else if (controller.right)
    {
        motor.motor_1 = -2;
    }
    else
    {
        motor.motor_1 = 0;
    }

    if(controller.up)
    {
        ROS_INFO("UP PRESSED");
        motor.motor_2 = 5;
    }
    else if(controller.down)
    {
        motor.motor_2 = -5;
    }
    else
    {
        motor.motor_2 = 0;
    }

    if(controller.l2 > 12)
    {
        motor.motor_3 = 5;
    }
    else if(controller.r2 > 12)
    {
        motor.motor_3 = -5;
    }
    else
    {
        motor.motor_3 = 0;
    }  

    relay_toggle();

    pub_motor.publish(motor);
    pub_relay.publish(relay);
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_arm_node");
    ros::NodeHandle nh;

    ROS_INFO("Manual Arm Node Started!");

    pub_motor = nh.advertise<robot_msgs::motor>("/actuator/motor/arm", 10);
    pub_relay = nh.advertise<robot_msgs::motor>("/actuator/relay", 10);
    sub_controller = nh.subscribe<robot_msgs::controller>("/input/controller", 10, subControllerCallback);
    tim_1kHz = nh.createTimer(ros::Duration(0.001), timer1msCallback);

    ros::spin();

    return 0;
}
