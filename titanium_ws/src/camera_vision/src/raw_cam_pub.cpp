#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "raw_cam_pub");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    int width, height;
    private_nh.param("width", width, 640);
    private_nh.param("height", height, 480);

    ROS_INFO("Starting [raw_cam_pub] at %dx%d", width, height);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera_pub/raw_image", 1);

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        cv::Mat frame;
        cap >> frame;

        if (frame.empty())
        {
            ROS_WARN_THROTTLE(1.0, "Empty frame received");
            continue;
        }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();

        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "camera";
        
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    
    cap.release();
    
    return 0;
}