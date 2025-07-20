#include "ros/ros.h"
#include "robot_msgs/encoder.h"
#include "robot_msgs/ultrasonic.h"
#include <robot_control/PID.hpp>

ros::Publisher pub_encoder, pub_ultrasonic;
ros::Timer timer_1ms;
robot_msgs::encoder encoder;
robot_msgs::ultrasonic ultrasonic;
PID MyPID(1, 0, 0);

void timer1msCallback(const ros::TimerEvent &event)
{
    encoder.enc_a += 1;
    encoder.enc_b += 2;
    encoder.enc_c += 3;
    encoder.enc_x += 4;
    encoder.enc_y += 5;

    pub_encoder.publish(encoder);

    ultrasonic.ultrasonic_a += 10;
    ultrasonic.ultrasonic_b += 20;
    ultrasonic.ultrasonic_c += 30;
    ultrasonic.ultrasonic_d += 40;
    
    pub_ultrasonic.publish(ultrasonic);

    ROS_INFO_STREAM(MyPID.update(10, 5, 999, 1000) << "\n");
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_pub_node");
    
    ros::NodeHandle nh;

    pub_encoder = nh.advertise<robot_msgs::encoder>("/sensor/encoder", 10);
    pub_ultrasonic = nh.advertise<robot_msgs::ultrasonic>("/sensor/ultrasonic", 10);

    timer_1ms = nh.createTimer(ros::Duration(0.001), timer1msCallback);

    ROS_INFO("Publisher Node Started!");

    ros::spin();
    
    return 0;
}