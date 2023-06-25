#include "navigation.h"
#include "handImage.h"
#include <thread>
//detect aruco
bool use_aruco = true;

int main(int argc, char **argv)
{
  //ros initialization & node handle 
  ros::init(argc, argv, "navigation");
  //ros::NodeHandle node;
  ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
  //Instance for classes
  
  //Navi
  Navigation* navigation = new Navigation();
  //Navigation navigation = new Navigation();
  HandImage* handimage = new HandImage(use_aruco);
  //thread handling 
  //ros::Publisher navigation_pub = n.advertise<geometry_msgs::Twist> (&Navigation::publishTwist, navigation, "/cmd_vel");
  std::thread navthread(&Navigation::publishTwist, navigation, "/cmd_vel");
//calling functions from HandImage
  std::thread imagethread(&HandImage::imageReader, handimage, "/camera/rgb/image_raw", "/img_transport/converted_img", navigation);

  //function to return when the thread execution has completed.
  imagethread.join();
  navthread.join();
 //delete is an operator that is used to destroy array and non-array(pointer)objects
  delete navigation;
  delete handimage;

 return 0;
}
