#include <pouco2000_ros/pouco2000_introspection.hpp>

pouco2000_ros_msgs::Buttons extract::field_buttons(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    return msg->buttons;
}

pouco2000_ros_msgs::SwitchsOnOff extract::field_switchs_onoff(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    return msg->switchs_on_off;
}

pouco2000_ros_msgs::SwitchsMode extract::field_switchs_modes(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    return msg->switchs_mode;
}

pouco2000_ros_msgs::Potentiometers extract::field_potentiometers_circle(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    return msg->potentiometers_circle;
}

pouco2000_ros_msgs::Potentiometers extract::field_potentiometers_slider(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    return msg->potentiometers_slider;
}


