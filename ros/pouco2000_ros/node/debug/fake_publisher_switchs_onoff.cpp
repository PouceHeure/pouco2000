#include "ros/ros.h"

#include "pouco2000_ros/pouco2000_debug.hpp"
#include "pouco2000_ros/SwitchsOnOff.h"


int main(int argc, char **argv){
  ros::init(argc, argv, "debug_fake_publisher_switchs_onoff_node");

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<pouco2000_ros::SwitchsOnOff>("switchs_onoff", 1000);
  
  int rate = load_param_rate(KEY_PARAM_RATE,DEFAULT_RATE);
  ros::Rate loop_rate(rate);

  rnd::init();

  int count = 0;
  while (ros::ok())
  {
    pouco2000_ros::SwitchsOnOff msg; 
    msg.data.push_back(rnd::gen<int>(0,10) > 5);
    msg.data.push_back(rnd::gen<int>(0,10) > 5);
    msg.data.push_back(rnd::gen<int>(0,10) > 2);

    pub.publish(msg);
    loop_rate.sleep();
    ++count;
  }

  return 0;
}