#include "handImage.h"

static const std::string OPENCV_WINDOW = "image window";
//classes to initialize
HandImage::HandImage(bool aruco):
//aruco markers dictionary is defined
dictionary(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)), //
arucoMarker(aruco) 
{
  if (not arucoMarker){
      originalImage = cv::imread("image.png", cv::IMREAD_GRAYSCALE); 
     // if aruco is not detected, image being loaded 
      orb = cv::ORB::create(); 
     //created orbit instance
      orb->detect(originalImage, kp1); 
     // detect orbit
      orb->compute(originalImage, kp1, des1);
    //computed orbit
  }
}

void HandImage::imageReader(std::string sub_topicName, std::string pub_topicName, Navigation* navigate)
{
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  //ros::NodeHandle node;
  image_transport::ImageTransport it_(*node);
  //image_transport::ImageTransport it_(node);
  subscribeTopicName = sub_topicName;
  //instance for subscribed topic
  publishTopicName = pub_topicName;
  //instance for publishing topic
  navPtr = navigate;
  //initializing navigation pointer to hold array of points
  camera_info_sub = node->subscribe("/camera/rgb/camera_info", 1, &HandImage::getCameraParameters, this);

  sub = it_.subscribe(subscribeTopicName, 1, &HandImage::handImageCallback, this);
  pub = it_.advertise(publishTopicName, 10);

  ROS_INFO("Subscribed to /camera/rgb/camera_info & published");

  ros::spin();
}

void HandImage::handImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    m_cv_ptr = (arucoMarker) ? cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8) : cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  if (arucoMarker){
    this->detectMultipleMarkers();
  } else {
    this->detectFeatures();
  }
  

  pub.publish(m_cv_ptr->toImageMsg());
}

void HandImage::getCameraParameters(const sensor_msgs::CameraInfoConstPtr& info){
  //Data manipulation to get camera matrix from cam info 
  std::copy(info->K.begin(), info->K.end(), this->camMatrix);
  std::copy(info->D.begin(), info->D.end(), this->distCoefficients);

  this->cameraMatrix = cv::Mat(3,3, CV_64F, camMatrix);
  this->distCoeffs = cv::Mat(1,5, CV_64F, distCoefficients);
}

void HandImage::detectMultipleMarkers(){
    int k =0;
    cv::aruco::detectMarkers(m_cv_ptr->image, dictionary, corners, ids);
    // if aruco_marker is detected
    if (ids.size() > 0) {
          
        //ROS_INFO("atleast one marker is detected");
	ROS_DEBUG("find %lu aruco markers", ids.size());
        cv::aruco::drawDetectedMarkers(m_cv_ptr->image, corners, ids);
        //camera matrix is obtained from camera_info topic in getCameraParameters
        cv::aruco::estimatePoseSingleMarkers(corners, 0.05, this->cameraMatrix, this->distCoeffs, rvecs, tvecs);
           // axis for aruco_marker
        for(int i=0; i<ids.size(); i++){
          cv::aruco::drawAxis(m_cv_ptr->image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);

          navPtr->navTwist.linear_x = tvecs[i][0];
          navPtr->navTwist.linear_y = tvecs[i][1];
          navPtr->navTwist.angular_z = rvecs[i][0];
        }
    } 
       //while(k<1)
        //{
        for(int j=0; j <rvecs.size(); j++)
        std::cout << "pose of the robot reprensenting rotation"<<" " << "rvecs" << rvecs.at(j) << "\n";
        for(int j=0; j <tvecs.size(); j++)
        std::cout << "pose of the robot reprensenting translation"<<" " << "tvecs" << tvecs.at(j) << "\n";
         // k = k+1;
        //}
        //std::cout << "out of while loop" << k <<"\n"; 
}


void HandImage::detectFeatures(){
  

  orb->detect(m_cv_ptr->image, kp2);
  orb->compute(m_cv_ptr->image, kp2, des2);

  bf = cv::BFMatcher::create(cv::NORM_HAMMING, true);
  bf->match(des1, des2, matches);
  std::sort(matches.begin(), matches.end());
  matches = std::vector<cv::DMatch>(matches.begin(), matches.begin() + 3);
  cv::drawMatches(originalImage, kp1, m_cv_ptr->image, kp2, matches, outputImage);
}


