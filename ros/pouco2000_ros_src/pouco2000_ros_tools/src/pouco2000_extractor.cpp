#include<pouco2000_ros_tools/pouco2000_extractor.hpp>

/* button */

ExtractorButton::ExtractorButton(int index):Extractor<bool,pouco2000_ros_msgs::Buttons>(index){

}

bool ExtractorButton::is_push(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    bool state;
    return extract_only_change(msg,state) && state;
}

pouco2000_ros_msgs::Buttons ExtractorButton::extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    return msg->buttons;
}

/* switch_on_off */

ExtractorSwitchOnOff::ExtractorSwitchOnOff(int index):Extractor<bool,pouco2000_ros_msgs::SwitchsOnOff>(index){

}

bool ExtractorSwitchOnOff::is_on(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    bool state;
    return extract_only_change(msg,state) && state;
}

pouco2000_ros_msgs::SwitchsOnOff ExtractorSwitchOnOff::extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    return msg->switchs_on_off;
}

/* switch_modes */

ExtractorSwitchMode::ExtractorSwitchMode(int index):Extractor<int,pouco2000_ros_msgs::SwitchsMode>(index){
    //avoid ignoring extration when mode researched is 0 
    previous_data = -1;
}

bool ExtractorSwitchMode::is_mode(const pouco2000_ros_msgs::Controller::ConstPtr& msg, const int& mode){
    int current_mode;
    return extract_only_change(msg,current_mode) && (current_mode == mode);
}

pouco2000_ros_msgs::SwitchsMode ExtractorSwitchMode::extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    return msg->switchs_mode;
}


/* potentiometers_circle */

ExtractorPotentiometerCircle::ExtractorPotentiometerCircle(int index):Extractor<float,pouco2000_ros_msgs::Potentiometers>(index){

}

pouco2000_ros_msgs::Potentiometers ExtractorPotentiometerCircle::extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    return msg->potentiometers_circle;
}

/* potentiomers_slider */

ExtractorPotentiometerSlider::ExtractorPotentiometerSlider(int index):Extractor<float,pouco2000_ros_msgs::Potentiometers>(index){

}

pouco2000_ros_msgs::Potentiometers ExtractorPotentiometerSlider::extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    return msg->potentiometers_slider;
}



HandleExtractors::HandleExtractors(){

}

ExtractorButton* HandleExtractors::get_button(int index){
    init_element<ExtractorButton>(index,map_extractors_button);
    return map_extractors_button[index];
}

ExtractorSwitchOnOff* HandleExtractors::get_switchs_onoff(int index){
    init_element<ExtractorSwitchOnOff>(index,map_extractors_switchs_onoff);
    return map_extractors_switchs_onoff[index];
}

ExtractorSwitchMode* HandleExtractors::get_switchs_modes(int index){
    init_element<ExtractorSwitchMode>(index,map_extractors_switchs_modes);
    return map_extractors_switchs_modes[index];
}

ExtractorPotentiometerCircle* HandleExtractors::get_potentiometers_circle(int index){
    init_element<ExtractorPotentiometerCircle>(index,map_extractors_potentiometers_circle);
    return map_extractors_potentiometers_circle[index];
}

ExtractorPotentiometerSlider* HandleExtractors::get_potentiometers_slider(int index){
    init_element<ExtractorPotentiometerSlider>(index,map_extractors_potentiometers_slider);
    return map_extractors_potentiometers_slider[index];
}
