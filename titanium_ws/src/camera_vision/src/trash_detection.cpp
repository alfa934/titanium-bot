#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <fstream>
#include <cmath>
#include "robot_msgs/camera.h"


/* 
 * To-Do :
 * - Make display window optional
 */

bool show_display = false;

const int low_H = 90, low_S = 150, low_V = 60;
const int high_H = 180, high_S = 255, high_V = 255;

#define CAMERA_CENTER_X 320
#define CAMERA_CENTER_Y 190

cv::Mat cameraMatrix, distCoeffs;
cv::Mat newCameraMatrix;
cv::Rect validPixROI;

ros::Publisher pub_trash;
robot_msgs::camera camera;

bool loadCalibrationData(const std::string& filename)
{
    try
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            ROS_ERROR("Failed to open calibration file: %s", filename.c_str());
            return false;
        }
        
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        
        if (cameraMatrix.empty() || distCoeffs.empty())
        {
            ROS_ERROR("Failed to load calibration parameters from file");
            return false;
        }
        
        ROS_INFO("Camera calibration data loaded successfully");
        
        std::cout << "Camera matrix: " << std::endl << cameraMatrix << std::endl;
        std::cout << "Distortion coefficients: " << std::endl << distCoeffs << std::endl;
        
        fs.release();
        return true;
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error loading calibration data: %s", e.what());
        return false;
    }
}

double calculateDistance(const cv::Point& p1, const cv::Point& p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;
        
        //--- Undistort
        cv::Mat undistorted_frame;
        if (!cameraMatrix.empty() && !distCoeffs.empty())
        {  
            static cv::Mat map1, map2;
            static bool maps_initialized = false;
            if (!maps_initialized)
            {
                cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(),
                                          cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, frame.size(), 1.0, frame.size(), 0),
                                          frame.size(), CV_16SC2, map1, map2);
                maps_initialized = true;
            }
            cv::remap(frame, undistorted_frame, map1, map2, cv::INTER_LINEAR);
            
        }
        else
        {
            undistorted_frame = frame.clone();
        }
        
        cv::Mat frame_HSV, frame_threshold;
        cv::cvtColor(undistorted_frame, frame_HSV, cv::COLOR_BGR2HSV);
        cv::inRange(frame_HSV, 
                   cv::Scalar(low_H, low_S, low_V), 
                   cv::Scalar(high_H, high_S, high_V), 
                   frame_threshold);
        
        cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        cv::morphologyEx(frame_threshold, frame_threshold, cv::MORPH_CLOSE, morph_kernel);
        cv::morphologyEx(frame_threshold, frame_threshold, cv::MORPH_OPEN, morph_kernel);

        //--- Set the blob detector
        cv::SimpleBlobDetector::Params params;
        params.filterByColor = false;
        params.filterByArea = true;
        params.minArea = 50;
        params.maxArea = 50000;
        params.filterByCircularity = true;
        params.minCircularity = 0.5;
        params.filterByConvexity = true;
        params.minConvexity = 0.7;
        params.filterByInertia = true;
        params.minInertiaRatio = 0.5;

        cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
        std::vector<cv::KeyPoint> keypoints;
        detector->detect(frame_threshold, keypoints);


        cv::Mat result_image = undistorted_frame.clone();
        
        //--- X in middle
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
        

        //--- Bounding boxes
        double min_distance = std::numeric_limits<double>::max();
        cv::Point bbox_center;
        cv::Rect closest_bbox;
        double x_distance = 0;
        double y_distance = 0;
        
        if(keypoints.empty())
        {
            cv::putText(result_image, "No trash detected!", cv::Point(20, 40),
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
            
            camera.trashDetected = false;
            camera.closestTrashX = 0;
            camera.closestTrashY = 0;
        }
        else
        {
            camera.trashDetected = true;

            for (const auto& kp : keypoints)
            {
                float radius = kp.size/2;
                cv::Rect bbox(kp.pt.x - radius, kp.pt.y - radius, radius*2, radius*2);
                cv::rectangle(result_image, bbox, cv::Scalar(0,255,0), 2);
                
                cv::Point current_center(bbox.x + bbox.width/2, bbox.y + bbox.height/2);
                double current_distance = calculateDistance(center, current_center);
                
                //--- closest bounding box
                if (current_distance < min_distance)
                {
                    min_distance = current_distance;
                    bbox_center = current_center;
                    closest_bbox = bbox;
                    x_distance = bbox_center.x - center.x;
                    y_distance = center.y - bbox_center.y;

                    camera.closestTrashX = (int16_t)x_distance;
                    camera.closestTrashY = -(int16_t)y_distance;
                }
                
                std::string coord_text = cv::format("(%d,%d)", (int)kp.pt.x, (int)kp.pt.y);
                int baseline = 0;
                cv::Size text_size = cv::getTextSize(coord_text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                
                cv::rectangle(result_image, 
                            cv::Point(bbox.x, bbox.y - text_size.height - 5),
                            cv::Point(bbox.x + text_size.width, bbox.y - 5),
                            cv::Scalar(0,0,0), cv::FILLED);
                
                cv::putText(result_image, coord_text, 
                          cv::Point(bbox.x, bbox.y - 7),
                          cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,255), 1);
                
                cv::circle(result_image, kp.pt, 3, cv::Scalar(0,0,255), -1);
            }
            
            //--- line to center bbox
            if (min_distance < std::numeric_limits<double>::max())
            {
                cv::line(result_image, center, bbox_center, cv::Scalar(255, 0, 255), 2);
                
                cv::circle(result_image, bbox_center, 5, cv::Scalar(255, 255, 0), -1);
                
                std::string center_text = cv::format("Camera Center: (%d,%d)", center.x, center.y);
                cv::putText(result_image, center_text, cv::Point(20, result_image.rows - 100),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                
                std::string euclidean_text = cv::format("Euclidean: %.1f px", min_distance);
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
        }

        pub_trash.publish(camera);

        if(show_display)
        {
            cv::imshow("Detection Result", result_image);
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
    ros::init(argc, argv, "trash_detector_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    private_nh.param<bool>("display", show_display, false);

    camera.cameraCenterX = CAMERA_CENTER_X;
    camera.cameraCenterY = CAMERA_CENTER_Y;
    pub_trash = nh.advertise<robot_msgs::camera>("/sensor/camera", 10);
    
    ROS_INFO("Trash Detector node started");
    ROS_INFO("Camera center coordinates: (%d, %d)", CAMERA_CENTER_X, CAMERA_CENTER_Y);
    ROS_INFO("Detecting objects in HSV range: H[%d-%d], S[%d-%d], V[%d-%d]", 
             low_H, high_H, low_S, high_S, low_V, high_V);

    if(show_display)
    {
        ROS_INFO("Display enabled - Press ESC in window to exit");
        std::string calibration_file;
        private_nh.param<std::string>("calibration_file", calibration_file, "/home/roobics/titanium-bot/titanium_ws/src/camera_vision/src/calibration.yaml");
        
        if (!loadCalibrationData(calibration_file))
        {
            ROS_WARN("Using uncalibrated camera mode. Distortion correction will not be applied.");
        }
        
        cv::namedWindow("Detection Result");
    }
    else
    {
        ROS_INFO("Display disabled - Running in headless mode");
    }
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/raw_image", 1, imageCallback);

    ros::spin();
    
    cv::destroyAllWindows();

    return 0;
}