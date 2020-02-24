#include "pouco2000_ros/pouco2000_debug.hpp"
#include "pouco2000_ros_msgs/Buttons.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "debug_fake_publisher_buttons_node");

  ros::NodeHandle n;
  auto gen = [](){
    return rnd::gen<int>(0,10)> 5;
  };

  FakePublisher<pouco2000_ros_msgs::Buttons> fake_publisher(n,"buttons");
  fake_publisher.run<bool>(10,gen);
  
  return 0;
}