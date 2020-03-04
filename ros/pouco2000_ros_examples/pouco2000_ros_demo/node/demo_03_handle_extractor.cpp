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
    HandleExtractor* handle_extractor;
    // sub
    ros::Subscriber sub;

    void callback(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
      if(handle_extractor->get_button(0)->is_push(msg)){
        ROS_INFO("button 0 pushed");
      }

      if(handle_extractor->get_button(3)->is_push(msg)){
        ROS_INFO("button 3 pushed");
      }

      if(handle_extractor->get_switchs_onoff(3)->is_on(msg)){
        ROS_INFO("switchs 3 is on");
      }
    }

  public: 
    DemoExtractor(ros::NodeHandle& nh,const std::string &topic){
      // instanciate extractors 
      handle_extractor = new HandleExtractor();
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