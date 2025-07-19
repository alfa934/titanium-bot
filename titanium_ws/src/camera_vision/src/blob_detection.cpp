#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

//--- Adjust HSV
const int low_H = 25, low_S = 40, low_V = 125;
const int high_H = 60, high_S = 255, high_V = 255;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat frame = cv_ptr->image;
        
        cv::imshow("Original", frame);
        
        cv::Mat frame_HSV, frame_threshold;
        cv::cvtColor(frame, frame_HSV, cv::COLOR_BGR2HSV);
        cv::inRange(frame_HSV, 
                   cv::Scalar(low_H, low_S, low_V), 
                   cv::Scalar(high_H, high_S, high_V), 
                   frame_threshold);
        
        cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5,5));
        cv::morphologyEx(frame_threshold, frame_threshold, cv::MORPH_CLOSE, morph_kernel);
        cv::morphologyEx(frame_threshold, frame_threshold, cv::MORPH_OPEN, morph_kernel);

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

        cv::Mat result_image = frame.clone();
        
        if(keypoints.empty())
        {
            cv::putText(result_image, "No blobs detected!", cv::Point(20, 40),
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255), 2);
        }
        else
        {
            for (const auto& kp : keypoints)
            {
                float radius = kp.size/2;
                cv::Rect bbox(kp.pt.x - radius, kp.pt.y - radius, radius*2, radius*2);
                cv::rectangle(result_image, bbox, cv::Scalar(0,255,0), 2);
                
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
        }

        cv::imshow("Detection Result", result_image);
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
    ros::init(argc, argv, "blob_detector_node");
    ros::NodeHandle nh;
    
    cv::namedWindow("Original");
    cv::namedWindow("Detection Result");
    
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/raw_image", 1, imageCallback);

    ROS_INFO("Blob detector node started");
    ROS_INFO("Detecting objects in HSV range: H[%d-%d], S[%d-%d], V[%d-%d]", 
             low_H, high_H, low_S, high_S, low_V, high_V);

    ros::spin();
    
    cv::destroyAllWindows();

    return 0;
}