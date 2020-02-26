#include <ros/ros.h>  
// lib msgs 
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
// lib pouco2000_introspection 
#include <pouco2000_ros_tools/pouco2000_extractor.hpp>


/**
 * @brief Example class showing how use extract information from controller msg
 * 
 */
class DemoSimple{
  private: 
    // sub
    ros::Subscriber sub;

    void callback(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
      if(msg->buttons.data.size() > 0){
          bool value_button = msg->buttons.data.at(0);
          ROS_INFO("value button 0 : %d",value_button);
      }
      if(msg->switchs_on_off.data.size() > 0){
          bool value_switch_on_off = msg->switchs_on_off.data.at(0);
          ROS_INFO("value switchs_on_off 0 : %d",value_switch_on_off);
      }
      if(msg->switchs_mode.data.size() > 0){
          int value_switch_mode = msg->switchs_mode.data.at(0);
          ROS_INFO("value switch_mode 0 : %d",value_switch_mode);
      }
      if(msg->potentiometers_circle.data.size() > 0){
          float value_pot_circle = msg->potentiometers_circle.data.at(0);
          ROS_INFO("value pot_circle 0 : %f",value_pot_circle);
      }
      if(msg->potentiometers_slider.data.size() > 0){
          float value_pot_slider = msg->potentiometers_slider.data.at(0);
          ROS_INFO("value pot_slider 0 : %f",value_pot_slider);
      }
    }

  public: 
    DemoSimple(ros::NodeHandle& nh,const std::string &topic){
      // instanciate subscriber to controller  
      sub = nh.subscribe(topic,1000,&DemoSimple::callback,this);
    }

};

int main(int argc, char **argv){
  ros::init(argc, argv, "demo_simple_node");
  ros::NodeHandle nh; 
 
  DemoSimple demo_simple(nh,"controller");

  ros::spin();

  return 0;
}