#include <pouco2000_ros/pouco2000_introspection.hpp>

pouco2000_ros::Buttons extract::buttons(const pouco2000_ros::Controller::ConstPtr& msg){
    return msg->buttons;
}

pouco2000_ros::SwitchsOnOff extract::switchs_onoff(const pouco2000_ros::Controller::ConstPtr& msg){
    return msg->switchs_on_off;
}

pouco2000_ros::SwitchsMode extract::switchs_modes(const pouco2000_ros::Controller::ConstPtr& msg){
    return msg->switchs_mode;
}

pouco2000_ros::Potentiometers extract::potentiometers_circle(const pouco2000_ros::Controller::ConstPtr& msg){
    return msg->potentiometers_circle;
}

pouco2000_ros::Potentiometers extract::potentiometers_slider(const pouco2000_ros::Controller::ConstPtr& msg){
    return msg->potentiometers_slider;
}
