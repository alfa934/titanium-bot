#include <ros/ros.h>
#include <robot_msgs/odom.h>
#include <robot_msgs/encoder.h>
#include <robot_msgs/yaw.h>
#include <cmath>

#define ENC_PULSE_TO_CM 0.001;

ros::Subscriber sub_encoder, sub_yaw;
ros::Publisher pub_odom;
ros::Timer timer_odom;

robot_msgs::encoder encoder;
robot_msgs::yaw yaw;

robot_msgs::odom global_odom;

void encoderCallback(const robot_msgs::encoderConstPtr &msg)
{
    encoder = *msg;
}

void yawCallback(const robot_msgs::yawConstPtr &msg)
{
    yaw = *msg;
}

void odomCallback(const ros::TimerEvent &event)
{
    global_odom.x += (encoder.enc_x * cos(yaw.radian) - encoder.enc_y * sin(yaw.radian)) * ENC_PULSE_TO_CM;
    global_odom.y += (encoder.enc_x * sin(yaw.radian) + encoder.enc_y * cos(yaw.radian)) * ENC_PULSE_TO_CM;
    global_odom.w =  yaw.degree;

    pub_odom.publish(global_odom);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_node");
    ros::NodeHandle nh;

    sub_encoder = nh.subscribe("/sensor/encoder", 10, encoderCallback);
    sub_yaw = nh.subscribe("/sensor/yaw", 10, yawCallback);
    pub_odom = nh.advertise<robot_msgs::odom>("/base/odometry", 10);
    timer_odom = nh.createTimer(ros::Duration(0.001), odomCallback); 

    ros::spin();

    return 0;
}
