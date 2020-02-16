//lib msgs 
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
// lib pouco2000_introspection 
#include <pouco2000_ros/pouco2000_introspection.hpp>


int main(int argc, char **argv){
  ros::init(argc, argv, "filter_node");
  ros::NodeHandle n;

  FilterPublisher<std_msgs::Bool,pouco2000_ros::Buttons> filter0(n,"controller","filter_button",extract::field_buttons,2);
  FilterPublisher<std_msgs::Float32,pouco2000_ros::Potentiometers> filter1(n,"controller","filter_pot_circle",extract::field_potentiometers_circle,2);
  FilterPublisher<std_msgs::Float32,pouco2000_ros::Potentiometers> filter2(n,"controller","filter_pot_slider",extract::field_potentiometers_slider,2);
  FilterPublisher<std_msgs::Bool,pouco2000_ros::SwitchsMode> filter3(n,"controller","filter_switchs_onoff",extract::field_switchs_modes,2);
  FilterPublisher<std_msgs::UInt8,pouco2000_ros::SwitchsOnOff> filter4(n,"controller","filter_switchs_modes",extract::field_switchs_onoff,2);

  ros::spin();

  return 0;
}