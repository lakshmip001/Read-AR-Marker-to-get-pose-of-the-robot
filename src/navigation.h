#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "tf/tf.h"

class Navigation{
    public: //functions
        Navigation();
        void publishTwist(std::string i_topicName);
        

        // a function to detect marker and do pose estimate and then set the twist parameters

    public: //variables
        std::string topicName;
        geometry_msgs::Twist twist;

        struct navigationTwist {
            float linear_x;
            float linear_y;
            float angular_z;
        } navTwist;

        float angle;
        float min_angle;
        float distance;
        float min_distance;
        float d_theta;
        float alpha;

        char state;
};

#endif
