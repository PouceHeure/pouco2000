// lib ros 
#include <ros/ros.h>

// lib msgs 
#include "pouco2000_ros/Controller.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

// lib cpp 
#include <boost/function.hpp>


float convertPercentToLinearX(const float value){
  return value/50;
}

void callback(const pouco2000_ros::Controller::ConstPtr& msg, ros::Publisher& pub_v_robot, ros::Publisher& pub_v_beacon, ros::Publisher& pub_t_beacon, bool& is_on){
    if(!msg->potentiometers_circle.data.empty()){
        geometry_msgs::Twist twist_msg;
        float value = msg->potentiometers_circle.data.at(0)/50;
        twist_msg.linear.x = convertPercentToLinearX(value);
        pub_v_robot.publish(twist_msg);
    }

    if(!msg->switchs_on_off.data.empty()){
        bool local_is_on = msg->switchs_on_off.data.at(0);
        if(local_is_on){
            is_on = !is_on;
            std_msgs::Float64 msg_t;
            std_msgs::Float64 msg_v;
            if(is_on){
                msg_t.data = 10;
                msg_v.data = 10;
            }else{
                msg_t.data = 0;
                msg_v.data = 0;
            }
            pub_t_beacon.publish(msg_t);
            pub_v_beacon.publish(msg_v);
        }
    }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "simple_controller_node");
  ros::NodeHandle nh;

  bool is_on = false;

  ros::Publisher pub_v_robot = nh.advertise<geometry_msgs::Twist>("/pouco2000Robot_diff_drive_controller/cmd_vel",100);
  ros::Publisher pub_v_beacon = nh.advertise<std_msgs::Float64>("/pouco2000Robot_beacon_rot_controller/command",100);
  ros::Publisher pub_t_beacon = nh.advertise<std_msgs::Float64>("pouco2000Robot_beacon_trans_controller/command",100);

  auto callback_twist = boost::bind(&callback,_1,boost::ref(pub_v_robot),boost::ref(pub_v_beacon),boost::ref(pub_t_beacon),boost::ref(is_on));
  ros::Subscriber sub = nh.subscribe<pouco2000_ros::Controller>("/ns_release/controller",1000,callback_twist);
  
  ros::spin();

  return 0;
}