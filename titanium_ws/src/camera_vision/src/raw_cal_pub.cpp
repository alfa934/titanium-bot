#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <fstream>

// Function to load calibration data
bool loadCalibrationData(const std::string& filename, 
                        cv::Mat& cameraMatrix, 
                        cv::Mat& distCoeffs,
                        cv::Mat& map1, 
                        cv::Mat& map2,
                        cv::Size imageSize)
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
        
        // Precompute the undistortion maps
        cv::Mat newCameraMatrix = cv::getOptimalNewCameraMatrix(
            cameraMatrix, distCoeffs, imageSize, 1.0, imageSize, 0);
        cv::initUndistortRectifyMap(
            cameraMatrix, distCoeffs, cv::Mat(),
            newCameraMatrix, imageSize,
            CV_16SC2, map1, map2);
        
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "raw_cam_pub");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    int width, height;
    private_nh.param("width", width, 640);
    private_nh.param("height", height, 480);
    
    std::string calibration_file;
    private_nh.param<std::string>("calibration_file", calibration_file, "/home/roobics/titanium-bot/titanium_ws/src/camera_vision/src/calibration.yaml");

    ROS_INFO("Starting [raw_cam_pub] at %dx%d", width, height);

    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/camera/raw_image", 1);

    cv::VideoCapture cap(0);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open camera!");
        return -1;
    }

    // Calibration variables
    cv::Mat cameraMatrix, distCoeffs, map1, map2;
    bool use_calibration = false;
    
    if (!calibration_file.empty())
    {
        use_calibration = loadCalibrationData(
            calibration_file, cameraMatrix, distCoeffs, 
            map1, map2, cv::Size(width, height));
        
        if (use_calibration)
        {
            ROS_INFO("Using calibration data for image undistortion");
        }
        else
        {
            ROS_WARN("Failed to load calibration data, publishing raw images");
        }
    }
    else
    {
        ROS_INFO("No calibration file specified, publishing raw images");
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

        // Apply undistortion if calibration data is available
        if (use_calibration)
        {
            cv::Mat undistorted_frame;
            cv::remap(frame, undistorted_frame, map1, map2, cv::INTER_LINEAR);
            frame = undistorted_frame;
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