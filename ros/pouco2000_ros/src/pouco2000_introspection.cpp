#include <pouco2000_ros/pouco2000_introspection.hpp>

template<>
pouco2000_ros::Buttons extract_field(const pouco2000_ros::Controller::ConstPtr& msg,Field v){
    switch(v){ 
        case buttons : 
            return msg->buttons;
    }
    return default_msg<pouco2000_ros::Buttons>();
}   

template<>
pouco2000_ros::Potentiometers extract_field(const pouco2000_ros::Controller::ConstPtr& msg,Field v){
    switch(v){
        case potentiometers_circle:
            return msg->potentiometers_circle;
        case potentiometers_slider:
            return msg->potentiometers_slider;
    }
    return default_msg<pouco2000_ros::Potentiometers>();
}

template<>
pouco2000_ros::SwitchsMode extract_field(const pouco2000_ros::Controller::ConstPtr& msg,Field v){
    switch(v){
        case switchs_modes:
            return msg->switchs_mode;
    }
    return default_msg<pouco2000_ros::SwitchsMode>();
}

template<>
pouco2000_ros::SwitchsOnOff extract_field(const pouco2000_ros::Controller::ConstPtr& msg,Field v){
    switch(v){
        case switchs_on_off:
            return msg->switchs_on_off;
    }
    return default_msg<pouco2000_ros::SwitchsOnOff>();
}