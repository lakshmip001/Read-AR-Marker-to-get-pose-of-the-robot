#include "navigation.h"

Navigation::Navigation():
    navTwist({0.0, 0.0, 0.0}),
    state('X'),
    alpha(1.57), //minimum of PI angle rotation
    min_distance(0.003),
    min_angle(0.003)
    {

    }

void Navigation::publishTwist(std::string i_topicName)
    {
        topicName = i_topicName;
        ros::NodeHandlePtr node = boost::make_shared<ros::NodeHandle>();
        //ros::NodeHandle node;
        ros::Publisher navigation_pub = node->advertise<geometry_msgs::Twist>(topicName, 10);
        ros::Rate loop_rate(10);


        while (ros::ok())
        {
            //distance: represents the distance to move by the robot
               distance = sqrt( pow(navTwist.linear_x, 2) + pow(navTwist.linear_y, 2) );
               angle = acos(navTwist.linear_x / distance);
               d_theta = angle - alpha;

               if (distance > min_distance and state == 'X'){
               state = 'A';
            }

           switch (state)  {
             case 'A': 
             if (abs(d_theta) > min_angle){
             twist.angular.z = d_theta * -2.0;
             } else {
            twist.angular.z = 0.0;
            state = (distance > 2.0 * min_distance) ? 'B' : 'H';
             }
             break;

             case 'B': 
             alpha = 1.57;
             if (distance > min_distance){
             float pull = (distance > 0.006) ? 4.0 : 2.0;
             twist.linear.x = -1.0 * distance * pull;
             twist.angular.z = d_theta * -0.5;
             } else {
             alpha = 2.35;
             min_angle = 0.0005;
             //the command will be to turn at 0.75 rad/s
             twist.linear.x = 0.0;
             state = 'A';
            }
           break;
          default:
          if (distance > 0.01 and state == 'H'){
            alpha = 1.57;
            min_angle = 0.003;
            state = 'A';
           }
           twist.linear.x = 0.0;
           twist.angular.z = 0.0;
          }   
            navigation_pub.publish(twist);
            loop_rate.sleep();
            ros::spinOnce();
        }
    }

