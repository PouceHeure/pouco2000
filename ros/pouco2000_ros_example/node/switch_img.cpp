#include <ros/ros.h>

#include <pouco2000_ros/Controller.h>
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

void callback(const pouco2000_ros::Controller::ConstPtr& msg,ros::Publisher& pub){
    if(msg->switchs_on_off.data.size() > 0){
        bool state = msg->switchs_on_off.data.at(0);
        ros_popup_img::PopUp msg_popup = generate_msg(state);
        pub.publish(msg_popup); 
    }
}


int main(int argc, char **argv){
  ros::init(argc, argv, "switch_img");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<ros_popup_img::PopUp>("switch_state_img",10);
  auto callback_bind = boost::bind(callback,_1,boost::ref(pub)); 
  ros::Subscriber sub = n.subscribe<pouco2000_ros::Controller>("controller",10,callback_bind);

  ros::spin();

  return 0;
}