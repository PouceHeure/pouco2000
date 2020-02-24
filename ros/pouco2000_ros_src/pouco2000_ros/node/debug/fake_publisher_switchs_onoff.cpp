#include "pouco2000_ros/pouco2000_debug.hpp"
#include "pouco2000_ros_msgs/SwitchsOnOff.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "debug_fake_publisher_switchs_onoff_node");

  ros::NodeHandle n;
  auto gen = [](){
    return rnd::gen<float>(0,10) > 5; 
  }; 

  FakePublisher<pouco2000_ros_msgs::SwitchsOnOff> fake_publisher(n,"switchs_onoff");
  fake_publisher.run<bool>(5,gen);

  return 0;
}