// lib ros 
#include <ros/ros.h>

// lib msgs 
#include "pouco2000_ros/Controller.h"
#include "geometry_msgs/Twist.h"

// lib cpp 
#include <boost/function.hpp>


float convertPercentToLinearX(const float value){
  return value/50;
}

void callback(const pouco2000_ros::Controller::ConstPtr& msg, ros::Publisher& pub){
    if(!msg->potentiometers_circle.data.empty()){
        geometry_msgs::Twist twist_msg;
        float value = msg->potentiometers_circle.data.at(0)/50;
        twist_msg.linear.x = convertPercentToLinearX(value);
        pub.publish(twist_msg);
    }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "twist_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("output",100);

  auto callback_twist = boost::bind(&callback,_1,boost::ref(pub));
  ros::Subscriber sub = nh.subscribe<pouco2000_ros::Controller>("input",1000,callback_twist);
  
  ros::spin();

  return 0;
}