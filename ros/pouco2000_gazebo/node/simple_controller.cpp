// lib ros 
#include <ros/ros.h>

// lib msgs 
#include "pouco2000_ros/Controller.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

// lib cpp 
#include <boost/function.hpp>

#include <pouco2000_ros/pouco2000_extractor.hpp>


class HardwareToController{
    private: 

    ros::Publisher pub_hlf;
    ros::Publisher pub_hlb;
    ros::Publisher pub_beacon;
    ros::Publisher pub_cmd_vel;
    ros::Publisher pub_fork;
    ros::Subscriber sub_controller;

    bool hlf_is_on = false; 
    bool hlb_is_on = false; 
    bool beacon_is_on = false; 
    bool fork_is_on = false; 

    ExtractorButton* hlf_extractor;
    ExtractorButton* hlb_extractor;
    ExtractorButton* beacon_extractor;
    ExtractorButton* fork_extractor;

    ExtractorPotentiometersCircle* linear_extractor;
    ExtractorPotentiometersCircle* rot_extractor;


    void callback(const pouco2000_ros::Controller::ConstPtr& msg){
        std_msgs ::Bool bool_msg;
        if(hlf_extractor->is_push(msg)){
            hlf_is_on = !hlf_is_on;
            bool_msg.data = hlf_is_on;
            pub_hlf.publish(bool_msg);
        }
        if(hlb_extractor->is_push(msg)){  
            hlb_is_on = !hlb_is_on;
            bool_msg.data = hlb_is_on;
            pub_hlb.publish(bool_msg);
        }
        if(beacon_extractor->is_push(msg)){ 
            beacon_is_on = !beacon_is_on;
            bool_msg.data = beacon_is_on;
            pub_beacon.publish(bool_msg);
        }
        if(fork_extractor->is_push(msg)){  
            fork_is_on = !fork_is_on;
            bool_msg.data = fork_is_on;
            pub_fork.publish(bool_msg);
        }
        
        float linear_value;
        bool linear_ok = linear_extractor->extract(msg,linear_value);
        
        float rot_value;
        bool rot_ok = rot_extractor->extract(msg,rot_value);
        
        if(linear_ok && rot_ok){
            geometry_msgs::Twist twist_msg;
            twist_msg.linear.x = linear_value/20;
            twist_msg.angular.z = rot_value/20;
            pub_cmd_vel.publish(twist_msg);
        }
    }

    public: 
    HardwareToController(ros::NodeHandle& nh, 
                         const std::string& topic_controller,
                         const std::string& topic_hlf,
                         const std::string& topic_hlb,
                         const std::string& topic_beacon,
                         const std::string& topic_fork,
                         const std::string& topic_cmd_vel){
    
    this->pub_hlf = nh.advertise<std_msgs::Bool>(topic_hlf,10);
    this->pub_hlb = nh.advertise<std_msgs::Bool>(topic_hlb,10);
    this->pub_beacon = nh.advertise<std_msgs::Bool>(topic_beacon,10);
    this->pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(topic_cmd_vel,10);
    this->pub_fork = nh.advertise<std_msgs::Bool>(topic_fork,10);
    this->sub_controller = nh.subscribe<pouco2000_ros::Controller>(topic_controller,1000,&HardwareToController::callback,this);
    
    hlf_extractor = new ExtractorButton(0);
    hlb_extractor = new ExtractorButton(1);
    beacon_extractor = new ExtractorButton(2);
    fork_extractor = new ExtractorButton(3);

    linear_extractor = new ExtractorPotentiometersCircle(0);
    rot_extractor = new ExtractorPotentiometersCircle(1);

    }


};

int main(int argc, char **argv){
  ros::init(argc, argv, "simple_controller_node");
  ros::NodeHandle nh;

  HardwareToController h_to_controller(nh,
                      "/ns_release/controller",
                      "/controller/headlight_front",
                      "/controller/headlight_back",
                      "/controller/fork",
                      "/controller/beacon",
                      "/controller/cmdvel"
                      );
  ros::spin();

  return 0;
}