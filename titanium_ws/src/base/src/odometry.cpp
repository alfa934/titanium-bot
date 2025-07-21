#include <ros/ros.h>
#include <robot_msgs/odom.h>
#include <robot_msgs/encoder.h>
#include <robot_msgs/yaw.h>
#include <robot_msgs/SetOdom.h>
#include <cmath>

#define ENC_PULSE_TO_CM 0.001;

ros::Subscriber sub_encoder, sub_yaw;
ros::Publisher pub_odom;
ros::Timer timer_odom;
ros::ServiceServer srv_odom;

robot_msgs::encoder encoder;
robot_msgs::yaw yaw;

robot_msgs::odom global_odom;

bool setOdomCallback(   robot_msgs::SetOdom::Request &req, 
                        robot_msgs::SetOdom::Response &res)
{
    global_odom.x = req.x;
    global_odom.y = req.y;
    global_odom.w = req.w;
    
    res.success = true;

    ROS_INFO("Odometry set to: x=%.2f, y=%.2f, w=%.2f", 
             req.x, req.y, req.w);

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
    srv_odom = nh.advertiseService("base/odometry/set", setOdomCallback);

    ros::spin();

    return 0;
}
