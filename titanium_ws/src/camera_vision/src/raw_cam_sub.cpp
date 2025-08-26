#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

bool show_display = false;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        cv::Mat final_frame;

        //-- Add processing here

        if (show_display)
        {
            cv::imshow("Camera Stream", final_frame);
            int key = cv::waitKey(1);
            if (key == 27) // ESC key
            { 
                ros::shutdown();
            }
        }

    } 
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Image conversion failed: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "raw_cam_sub");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    private_nh.param("display", show_display, false);

    ROS_INFO("Starting [raw_cam_sub], display=%s", show_display == true ? "true" : "false");

    
    if (show_display)
    {
        cv::namedWindow("Camera Stream");
        ROS_INFO("Display enabled - Press ESC in window to exit");
    } 
    else
    {
        ROS_INFO("Display disabled - Running in headless mode");
    }

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/raw_image", 1, imageCallback);

    ros::spin();

    if (show_display)
    {
        cv::destroyAllWindows();
    }

    return 0;
}