#include <ros/ros.h>  
// lib msgs 
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
// lib pouco2000_introspection 
#include <pouco2000_ros_tools/pouco2000_extractor.hpp>


/**
 * @brief Example class showing how use extract information from controller msg
 * In using pouco2000_extractor lib 
 * 
 */
class DemoExtractor{
  private: 
    // extractors  
    HandleExtractors* handle_extractor;
    // sub
    ros::Subscriber sub;

    void callback(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
      if(handle_extractor->get_button(0)->is_push(msg)){
        ROS_INFO("button 0 pushed");
      }

      if(handle_extractor->get_button(3)->is_push(msg)){
        ROS_INFO("button 3 pushed");
      }

      if(handle_extractor->get_switchs_modes(3)->is_mode(msg,0)){
        ROS_INFO("switch_modes 3 is on mode 0");
      }

      if(handle_extractor->get_switchs_onoff(3)->is_on(msg)){
        ROS_INFO("switch_on_off 3 is on");
      }

      float value_circle; 
      if(handle_extractor->get_potentiometers_circle(3)->extract_only_change(msg,value_circle)){
        ROS_INFO("potentiometers_circle 3 value: %f",value_circle);
      }

      float value_slider; 
      if(handle_extractor->get_potentiometers_slider(3)->extract_only_change(msg,value_slider)){
        ROS_INFO("potentiometers_slider 3 value: %f",value_slider);
      }

    }

  public: 
    DemoExtractor(ros::NodeHandle& nh,const std::string &topic){
      // instanciate extractors 
      handle_extractor = new HandleExtractors();
      // instanciate subscriber to controller  
      sub = nh.subscribe(topic,1000,&DemoExtractor::callback,this);
    }
};


int main(int argc, char **argv){
  ros::init(argc, argv, "demo_handle_extractor_node");
  ros::NodeHandle nh; 

  DemoExtractor demo_extractor(nh,"controller");
  ros::spin();

  return 0;
}