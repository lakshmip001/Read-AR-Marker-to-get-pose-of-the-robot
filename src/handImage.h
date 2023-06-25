#ifndef HANDIMAGE_H
#define HANDIMAGE_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "navigation.h"
#include <sstream>
#include <memory>
#include "math.h"
#include <opencv2/features2d.hpp>
#include <algorithm>

class HandImage{
    public: 
        //check if it aruco is available
        HandImage(bool aruco);
        //used reading an image from /camera/rgb/camera_info & converted and published for navigation
        void imageReader(std::string sub_topicName, std::string pub_topicName, Navigation* navigate);
        void handImageCallback(const sensor_msgs::ImageConstPtr& msg);
        void getCameraParameters(const sensor_msgs::CameraInfoConstPtr& info);
        void detectMultipleMarkers();
        void detectFeatures();
        //declaration of variables to store the information and publish
        std::string subscribeTopicName;
        std::string publishTopicName;        
        _Float64 camMatrix[9];
        _Float64 distCoefficients[5];
        bool arucoMarker;
        image_transport::Subscriber sub;
        image_transport::Publisher pub;
        Navigation* navPtr;
        ros::NodeHandle node;
        ros::Subscriber camera_info_sub;
        cv::Mat cameraMatrix;
        cv::Mat distCoeffs;
        cv::Ptr<cv::aruco::Dictionary> dictionary;
        cv_bridge::CvImagePtr m_cv_ptr;
        cv::Mat m_imageCopy;
        std::vector<cv::Vec3d> rvecs;
        std::vector<cv::Vec3d> tvecs;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::Ptr<cv::ORB> orb;
        cv::Ptr<cv::BFMatcher> bf;
        cv::Mat originalImage;
        cv::Mat inputImage;
        cv::Mat outputImage;
        std::vector<cv::KeyPoint> kp1;
        std::vector<cv::KeyPoint> kp2;
        cv::Mat des1;
        cv::Mat des2;
        std::vector<cv::DMatch> matches;
        
};

#endif
