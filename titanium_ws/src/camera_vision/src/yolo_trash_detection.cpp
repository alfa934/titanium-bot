#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <cmath>
#include "robot_msgs/camera.h"

bool show_display = true;

#define CAMERA_CENTER_X 320
#define CAMERA_CENTER_Y 240

ros::Publisher pub_trash;
robot_msgs::camera camera_msg;

// Store the latest detection results
bool trash_detected = false;
int16_t bbox[4] = {0};
int16_t centroid[2] = {0};
int16_t closestTrashX = 0;
int16_t closestTrashY = 0;

void cameraCallback(const robot_msgs::camera::ConstPtr& msg)
{
    // Store the latest detection results
    trash_detected = msg->trashDetected;
    closestTrashX = msg->closestTrashX;
    closestTrashY = msg->closestTrashY;
    
    // Copy bbox and centroid data
    for (int i = 0; i < 4; i++) {
        bbox[i] = msg->bbox[i];
    }
    for (int i = 0; i < 2; i++) {
        centroid[i] = msg->centroid[i];
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;
        
        // Use raw frame directly (no undistortion)
        cv::Mat result_image = frame.clone();
        
        //--- X in middle (camera center)
        cv::Point center(CAMERA_CENTER_X, CAMERA_CENTER_Y);
        
        int x_length = 10;
        cv::line(result_image, 
                cv::Point(center.x - x_length/2, center.y - x_length/2),
                cv::Point(center.x + x_length/2, center.y + x_length/2),
                cv::Scalar(0, 255, 255), 2);
        cv::line(result_image, 
                cv::Point(center.x - x_length/2, center.y + x_length/2),
                cv::Point(center.x + x_length/2, center.y - x_length/2),
                cv::Scalar(0, 255, 255), 2);
        
        if(trash_detected)
        {
            // Draw bounding box
            cv::Rect trash_bbox(bbox[0], bbox[1], bbox[2] - bbox[0], bbox[3] - bbox[1]);
            cv::rectangle(result_image, trash_bbox, cv::Scalar(0, 255, 0), 2);
            
            // Draw centroid
            cv::Point trash_centroid(centroid[0], centroid[1]);
            cv::circle(result_image, trash_centroid, 5, cv::Scalar(0, 0, 255), -1);
            
            // Draw line from camera center to centroid
            cv::line(result_image, center, trash_centroid, cv::Scalar(255, 0, 255), 2);
            
            // Calculate distances
            double x_distance = trash_centroid.x - center.x;
            double y_distance = center.y - trash_centroid.y;
            double euclidean_distance = std::sqrt(std::pow(x_distance, 2) + std::pow(y_distance, 2));
            
            // Display information
            std::string center_text = cv::format("Camera Center: (%d,%d)", center.x, center.y);
            cv::putText(result_image, center_text, cv::Point(20, result_image.rows - 100),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            
            std::string euclidean_text = cv::format("Euclidean: %.1f px", euclidean_distance);
            std::string x_text = cv::format("X-distance: %.1f px", x_distance);
            std::string y_text = cv::format("Y-distance: %.1f px", y_distance);   
            cv::putText(result_image, euclidean_text, cv::Point(20, result_image.rows - 80),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
            cv::putText(result_image, x_text, cv::Point(20, result_image.rows - 60),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
            cv::putText(result_image, y_text, cv::Point(20, result_image.rows - 40),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            
            std::string x_dir = (x_distance > 0) ? "RIGHT" : "LEFT";
            std::string y_dir = (y_distance > 0) ? "UP" : "DOWN";
            
            std::string direction_text = cv::format("Direction: %s, %s", x_dir.c_str(), y_dir.c_str());
            cv::putText(result_image, direction_text, cv::Point(20, result_image.rows - 20),
                       cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 2);
        }
        else
        {
            cv::putText(result_image, "No trash detected!", cv::Point(20, 40),
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
        }

        if(show_display)
        {
            cv::imshow("YOLOv8 Detection Result", result_image);
            int key = cv::waitKey(1);
            if (key == 27) //--- ESC key
            { 
                ros::shutdown();
            }
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolov8_visualizer_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    private_nh.param<bool>("display", show_display, true);

    camera_msg.cameraCenterX = CAMERA_CENTER_X;
    camera_msg.cameraCenterY = CAMERA_CENTER_Y;
    
    ROS_INFO("YOLOv8 Visualizer node started");
    ROS_INFO("Camera center coordinates: (%d, %d)", CAMERA_CENTER_X, CAMERA_CENTER_Y);

    if(show_display)
    {
        ROS_INFO("Display enabled - Press ESC in window to exit");
        cv::namedWindow("YOLOv8 Detection Result");
    }
    else
    {
        ROS_INFO("Display disabled - Running in headless mode");
    }
    
    // Subscribe to camera messages
    ros::Subscriber camera_sub = nh.subscribe("/sensor/camera", 1, cameraCallback);
    
    // Subscribe to image topic
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/camera/raw_image", 1, imageCallback);

    ros::spin();
    
    cv::destroyAllWindows();

    return 0;
}