#include "pouco2000_ros/pouco2000_debug.hpp"
#include "pouco2000_ros/Potentiometers.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "debug_fake_publisher_potentiometers_slider_node");

  ros::NodeHandle n;
  auto gen = [](){
    return rnd::gen<float>(0,10); 
  }; 

  FakePublisher<pouco2000_ros::Potentiometers> fake_publisher(n,"potentiometers_slider");
  fake_publisher.run<float>(5,gen);

  return 0;
}