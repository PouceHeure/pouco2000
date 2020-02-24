#include <ros/ros.h>

#include <pouco2000_ros_msgs/Controller.h>

#include <pouco2000_ros_tools/pouco2000_extractor.hpp>
#include <ros_popup_img/PopUp.h>

#include <boost/function.hpp>


ros_popup_img::PopUp generate_msg(bool state){
    ros_popup_img::PopUp msg;
    msg.title = "Motor Wheels"; 
    if(state){
        msg.level = 2;
        msg.subtitle = "state: on";
    }else{
        msg.level = 3;
        msg.subtitle = "state: off";
    }
    return msg;
}

void callback(const pouco2000_ros_msgs::Controller::ConstPtr& msg,ros::Publisher& pub, ExtractorSwitchOnOff& extractor){
    bool state = extractor.is_on(msg);
    ros_popup_img::PopUp msg_popup = generate_msg(state);
    pub.publish(msg_popup); 
}


int main(int argc, char **argv){
  ros::init(argc, argv, "popup_node");
  ros::NodeHandle n;

  ExtractorSwitchOnOff extractor(0);
  ros::Publisher pub = n.advertise<ros_popup_img::PopUp>("state_img_switch",10);
  auto callback_bind = boost::bind(callback,_1,boost::ref(pub),boost::ref(extractor)); 
  ros::Subscriber sub = n.subscribe<pouco2000_ros_msgs::Controller>("controller",10,callback_bind);

  ros::spin();

  return 0;
}