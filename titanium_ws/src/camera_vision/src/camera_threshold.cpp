#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


int low_H = 0, low_S = 0, low_V = 0;
int high_H = 180, high_S = 255, high_V = 255;
int brightness = 0;    // Range: -100 to 100
int contrast = 100;    // Range: 0-200 (100 = no change)
int blur_amount = 0;   // Range: 0-20 (kernel size)
bool show_original = true;

const std::string control_window = "Controls";
const std::string output_window = "Final Output";

void on_low_H(int, void*) { low_H = std::min(high_H-1, low_H); }
void on_high_H(int, void*) { high_H = std::max(high_H, low_H+1); }
void on_low_S(int, void*) { low_S = std::min(high_S-1, low_S); }
void on_high_S(int, void*) { high_S = std::max(high_S, low_S+1); }
void on_low_V(int, void*) { low_V = std::min(high_V-1, low_V); }
void on_high_V(int, void*) { high_V = std::max(high_V, low_V+1); }


void apply_brightness_contrast(cv::Mat& image)
{
    double alpha = contrast / 100.0;
    int beta = brightness;
    image.convertTo(image, -1, alpha, beta);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        
        cv::Mat processed = cv_ptr->image.clone();
        
        apply_brightness_contrast(processed);
        
        if (blur_amount > 0)
        {
            cv::blur(processed, processed, cv::Size(blur_amount, blur_amount));
        }
        
        cv::Mat hsv;
        cv::cvtColor(processed, hsv, cv::COLOR_BGR2HSV);
        
        cv::Mat mask;
        cv::inRange(hsv, 
                   cv::Scalar(low_H, low_S, low_V), 
                   cv::Scalar(high_H, high_S, high_V), 
                   mask);
        
        cv::Mat final_output;
        processed.copyTo(final_output, mask);
        
        if (show_original)
        {
            cv::imshow("Original", cv_ptr->image);
        }
        else
        {
            cv::destroyWindow("Original");
        }
        
        cv::imshow(output_window, final_output);
        
        int key = cv::waitKey(1);
        if (key == 27) // ESC key
        { 
            ros::shutdown();
        }
        
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "camera_threshold_node");
    ros::NodeHandle nh;
    
    cv::namedWindow(control_window);
    
    cv::createTrackbar("Low H", control_window, &low_H, 180, on_low_H);
    cv::createTrackbar("High H", control_window, &high_H, 180, on_high_H);
    cv::createTrackbar("Low S", control_window, &low_S, 255, on_low_S);
    cv::createTrackbar("High S", control_window, &high_S, 255, on_high_S);
    cv::createTrackbar("Low V", control_window, &low_V, 255, on_low_V);
    cv::createTrackbar("High V", control_window, &high_V, 255, on_high_V);
    
    cv::createTrackbar("Brightness", control_window, &brightness, 200, NULL);
    cv::setTrackbarMin("Brightness", control_window, -100);
    cv::createTrackbar("Contrast %", control_window, &contrast, 200, NULL);
    cv::createTrackbar("Blur Amount", control_window, &blur_amount, 20, NULL);
    
    cv::createTrackbar("Show Original", control_window, (int*)&show_original, 1, NULL);
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/raw_image", 1, imageCallback);
    
    ROS_INFO("Camera threshold is running...");
    
    ros::spin();
    
    cv::destroyAllWindows();

    return 0;
}