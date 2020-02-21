//lib msgs 
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
// lib pouco2000_introspection 
#include <pouco2000_ros/pouco2000_introspection.hpp>


/*void callback(const pouco2000_ros::Controller::ConstPtr& msg, Extractor<bool,pouco2000_ros::Buttons>& extractor){
    bool value;
    if(extractor.extract_only_change(msg,value)){
        ROS_INFO("new value: %d",value);
    }
    ROS_WARN("same value");
}*/

int main(int argc, char **argv){
  ros::init(argc, argv, "extractor_node");
  ros::NodeHandle n;

  /*Extractor<bool,pouco2000_ros::Buttons> extractor(extract::field_buttons,0);
  auto callback_extractor = boost::bind(callback,_1,boost::ref(extractor));
  ros::Subscriber sub = n.subscribe<pouco2000_ros::Controller>("/ns_release/controller",1000,callback_extractor);
  */
  
  ros::spin();

  return 0;
}