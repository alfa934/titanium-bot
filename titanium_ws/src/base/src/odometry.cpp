#include <ros/ros.h>
#include <robot_msgs/odom.h>
#include <robot_msgs/encoder.h>
#include <robot_msgs/yaw.h>
#include <std_srvs/Empty.h>
#include <cmath>

#define ENC_PULSE_TO_CM 0.001;

ros::Subscriber sub_encoder, sub_yaw;
ros::Publisher pub_odom;
ros::Timer timer_odom;
ros::ServiceServer srv_reset;

robot_msgs::encoder encoder;
robot_msgs::yaw yaw;

robot_msgs::odom global_odom;

bool resetOdomCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    global_odom.x = 0;
    global_odom.y = 0;
    global_odom.w = 0;
    return true;
}

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

    // ROS_INFO_STREAM(global_odom << "\n");

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
    srv_reset = nh.advertiseService("base/odometry/reset", resetOdomCallback);

    ros::spin();

    return 0;
}
