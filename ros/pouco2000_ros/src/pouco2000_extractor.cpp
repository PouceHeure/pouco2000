#include<pouco2000_ros/pouco2000_extractor.hpp>

/* button */

ExtractorButton::ExtractorButton(int index):Extractor<bool,pouco2000_ros::Buttons>(index){

}

bool ExtractorButton::is_push(const pouco2000_ros::Controller::ConstPtr& msg){
    bool state;
    return extract_only_change(msg,state) && state;
}

pouco2000_ros::Buttons ExtractorButton::extract_field(const pouco2000_ros::Controller::ConstPtr& msg){
    return msg->buttons;
}

/* switch_on_off */

ExtractorSwitchsOnOff::ExtractorSwitchsOnOff(int index):Extractor<bool,pouco2000_ros::SwitchsOnOff>(index){

}

bool ExtractorSwitchsOnOff::is_on(const pouco2000_ros::Controller::ConstPtr& msg){
    bool state;
    return extract_only_change(msg,state) && state;
}

pouco2000_ros::SwitchsOnOff ExtractorSwitchsOnOff::extract_field(const pouco2000_ros::Controller::ConstPtr& msg){
    return msg->switchs_on_off;
}

/* switch_modes */

ExtractorSwitchsMode::ExtractorSwitchsMode(int index):Extractor<int,pouco2000_ros::SwitchsMode>(index){

}

bool ExtractorSwitchsMode::is_mode(const pouco2000_ros::Controller::ConstPtr& msg, const int& mode){
    int current_mode;
    return extract_only_change(msg,current_mode) && current_mode == mode;
}

pouco2000_ros::SwitchsMode ExtractorSwitchsMode::extract_field(const pouco2000_ros::Controller::ConstPtr& msg){
    return msg->switchs_mode;
}


/* potentiometers_circle */

ExtractorPotentiometersCircle::ExtractorPotentiometersCircle(int index):Extractor<float,pouco2000_ros::Potentiometers>(index){

}

pouco2000_ros::Potentiometers ExtractorPotentiometersCircle::extract_field(const pouco2000_ros::Controller::ConstPtr& msg){
    return msg->potentiometers_circle;
}

/* potentiomers_slider */

ExtractorPotentiometersSlider::ExtractorPotentiometersSlider(int index):Extractor<float,pouco2000_ros::Potentiometers>(index){

}

pouco2000_ros::Potentiometers ExtractorPotentiometersSlider::extract_field(const pouco2000_ros::Controller::ConstPtr& msg){
    return msg->potentiometers_circle;
}