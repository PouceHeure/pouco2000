#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <pouco2000_ros/pouco2000_introspection.hpp>


int main(int argc, char **argv){
  ros::init(argc, argv, "filter_node");
  ros::NodeHandle n;

  FilterPublisher<std_msgs::Float32,pouco2000_ros::Potentiometers> filter(n,"controller","filter",Field::potentiometers_circle,2);
  filter.run();

  return 0;
}