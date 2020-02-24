#include <ros/ros.h>  
//lib msgs 
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
// lib pouco2000_introspection 
#include <pouco2000_ros_tools/pouco2000_extractor.hpp>


void callback(const pouco2000_ros_msgs::Controller::ConstPtr& msg, ExtractorButton& extractor){
    bool value;

    if(extractor.extract(msg,value)){
      ROS_INFO("[extract] callback called: %d",value);
    }

    if(extractor.extract_only_change(msg,value)){
      ROS_INFO("[extract_only_change] new value: %d",value);
    }

    if(extractor.is_push(msg)){
      ROS_INFO("[is_push] button is pushed: %d",value);
    }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "extractor_node");
  ros::NodeHandle n; 

  ExtractorButton extractor(0);
  auto callback_extractor = boost::bind(callback,_1,boost::ref(extractor));
  ros::Subscriber sub = n.subscribe<pouco2000_ros_msgs::Controller>("controller",1000,callback_extractor);
  
  ros::spin();

  return 0;
}