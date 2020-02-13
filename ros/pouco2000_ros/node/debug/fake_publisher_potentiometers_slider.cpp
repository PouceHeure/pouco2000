#include "ros/ros.h"

#include "pouco2000_ros/pouco2000_debug.hpp"
#include "pouco2000_ros/Potentiometers.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "debug_fake_publisher_potentiometers_slider_node");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<pouco2000_ros::Potentiometers>("potentiometers_slider", 1000);
  
  int rate = load_param_rate(KEY_PARAM_RATE,DEFAULT_RATE);
  ros::Rate loop_rate(rate);

  rnd::init();

  int count = 0;
  while (ros::ok())
  {
    pouco2000_ros::Potentiometers msg; 
    msg.data.push_back(rnd::gen<float>(0,1024));
    msg.data.push_back(rnd::gen<float>(0,1024));
    msg.data.push_back(rnd::gen<float>(0,1024));

    pub.publish(msg);
    loop_rate.sleep();
    ++count;
  }

  return 0;
}