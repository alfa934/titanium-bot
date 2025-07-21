#include <ros/ros.h>
#include <robot_control/MotionProfile.hpp>
#include <std_msgs/Float64.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_profile_visualiser");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    double max_accel, max_vel, distance, dt;
    pnh.param("max_acceleration", max_accel, 1.0);
    pnh.param("max_velocity", max_vel, 2.0);
    pnh.param("distance", distance, 5.0);
    pnh.param("time_step", dt, 0.01);
    
    ros::Publisher pos_pub = nh.advertise<std_msgs::Float64>("position", 1);
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float64>("velocity", 1);
    ros::Publisher acc_pub = nh.advertise<std_msgs::Float64>("acceleration", 1);
    
    MotionProfile profile(max_accel, max_vel, distance);
    double total_time = profile.getTotalTime();
    
    ROS_INFO("Motion Profile Visualization");
    ROS_INFO("Accel: %.2f m/s², Vel: %.2f m/s, Dist: %.2f m, Time: %.2f s", 
             max_accel, max_vel, distance, total_time);
    
    ros::Rate rate(1.0/dt);

    ros::Duration(3).sleep(); //-- just wait until rqt_plot connects (trust me)
    
    for(double t = 0; t <= total_time && ros::ok(); t += dt)
    {
        auto state = profile.calculate(t);
        
        std_msgs::Float64 pos_msg, vel_msg, acc_msg;
        pos_msg.data = state.position;
        vel_msg.data = state.velocity;
        acc_msg.data = state.acceleration;
        
        pos_pub.publish(pos_msg);
        vel_pub.publish(vel_msg);
        acc_pub.publish(acc_msg);
        
        rate.sleep();
    }
    
    ROS_INFO("Profile completed");
    ros::Duration(1.0).sleep();
    
    return 0;
}