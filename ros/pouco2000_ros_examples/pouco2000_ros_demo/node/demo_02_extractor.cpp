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
    ExtractorButton* ext_button_0;
    ExtractorSwitchOnOff* ext_switch_on_off_0;
    ExtractorSwitchMode* ext_switch_mode_0;
    ExtractorPotentiometerCircle* ext_pot_circle_0;
    ExtractorPotentiometerSlider* ext_pot_slider_0;
    // sub
    ros::Subscriber sub;

    void callback(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
      if(ext_button_0->is_push(msg)){
        ROS_INFO("button 0 pushed");
      }
      if(ext_switch_on_off_0->is_on(msg)){
        ROS_INFO("switch_onoff 0 is on");
      }
      if(ext_switch_mode_0->is_mode(msg,0)){
        ROS_INFO("swith_mode 0 is on mode 0");
      }
      float value_circle;
      if(ext_pot_circle_0->extract_only_change(msg,value_circle)){
        ROS_INFO("pot_circle 0 has value: %f",value_circle);
      }
      float value_slider; 
      if(ext_pot_slider_0->extract_only_change(msg,value_slider)){
        ROS_INFO("pot_slider 0 has value: %f",value_slider);
      }
    }

  public: 
    DemoExtractor(ros::NodeHandle& nh,const std::string &topic){
      // instanciate extractors 
      ext_button_0 = new ExtractorButton(0);
      ext_switch_on_off_0 = new ExtractorSwitchOnOff(0);
      ext_switch_mode_0 = new ExtractorSwitchMode(0);
      ext_pot_circle_0 = new ExtractorPotentiometerCircle(0);
      ext_pot_slider_0 = new ExtractorPotentiometerSlider(0);
      // instanciate subscriber to controller  
      sub = nh.subscribe(topic,1000,&DemoExtractor::callback,this);
    }

};



int main(int argc, char **argv){
  ros::init(argc, argv, "demo_extractor_node");
  ros::NodeHandle nh; 
 
  DemoExtractor demo_extractor(nh,"controller");

  ros::spin();

  return 0;
}