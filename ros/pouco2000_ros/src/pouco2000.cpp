#include <pouco2000_ros/pouco2000.hpp>

/* Convertisser */ 

ConvertisserPotentiometerToPercent::ConvertisserPotentiometerToPercent(const float& _min,const float& _max){
    this->min = _min; 
    this->max = _max; 
}

float ConvertisserPotentiometerToPercent::convert(const float& value){
    return (value - this->min)/(this->max - this->min)*100;
}

pouco2000_ros::Potentiometers convert_potentiometers(ConvertisserPotentiometerToPercent c, pouco2000_ros::Potentiometers& msg){
    pouco2000_ros::Potentiometers new_msg;
    std::vector<float>::iterator it;
    for(it = msg.data.begin();it != msg.data.end();++it){
        new_msg.data.push_back(c.convert(*it));
    }
    return new_msg;
}

/* Controller */

Controller::Controller(){

}

void Controller::set_node_handle(ros::NodeHandle &nh){
    this->nh = nh;
}

void Controller::set_send_mode(SendMode send_mode){
    this->send_mode = send_mode;
}

void Controller::set_sleep_rate(int rate){
    this->sleep_rate = rate; 
}

using type_msg_buttons = pouco2000_ros::Buttons;

void Controller::set_sub_buttons(std::string topic,boost::function<void(const type_msg_buttons::ConstPtr&,pouco2000_ros::Controller& current_msg)> buttons_callback){
    auto wrapper_callback = this->wrapper(buttons_callback);
    this->sub_buttons = this->nh.subscribe<type_msg_buttons>(topic,Controller::QUEUE_SIZE_SUB,wrapper_callback);
}

using type_msg_switch_onoff = pouco2000_ros::SwitchsOnOff;

void Controller::set_sub_switchs_onoff(std::string topic,boost::function<void(const type_msg_switch_onoff::ConstPtr&,pouco2000_ros::Controller& current_msg)> buttons_callback){
    auto wrapper_callback = this->wrapper(buttons_callback);
    this->sub_switchs_onoff = this->nh.subscribe<type_msg_switch_onoff>(topic,Controller::QUEUE_SIZE_SUB,wrapper_callback);
}

using type_msg_switch_modes = pouco2000_ros::SwitchsMode;

void Controller::set_sub_switchs_modes(std::string topic,boost::function<void(const type_msg_switch_modes::ConstPtr&,pouco2000_ros::Controller& current_msg)> buttons_callback){
    auto wrapper_callback = this->wrapper(buttons_callback);
    this->sub_switchs_modes = this->nh.subscribe<type_msg_switch_modes>(topic,Controller::QUEUE_SIZE_SUB,wrapper_callback);
}

using type_msg_potentiometers = pouco2000_ros::Potentiometers;

void Controller::set_sub_potentiometers_circle(std::string topic, boost::function<void(const type_msg_potentiometers::ConstPtr&,pouco2000_ros::Controller& current_msg)> buttons_callback){
    auto wrapper_callback = this->wrapper(buttons_callback);
    this->sub_potentiometers_circle = this->nh.subscribe<type_msg_potentiometers>(topic,Controller::QUEUE_SIZE_SUB,wrapper_callback);
}

void Controller::set_sub_potentiometers_slider(std::string topic,boost::function<void(const type_msg_potentiometers::ConstPtr&,pouco2000_ros::Controller& current_msg)> buttons_callback){
    auto wrapper_callback = this->wrapper(buttons_callback);
    this->sub_potentiometers_slider = this->nh.subscribe<type_msg_potentiometers>(topic,Controller::QUEUE_SIZE_SUB,wrapper_callback);
}

void Controller::set_pub_controller(std::string topic){
    this->pub_controller = nh.advertise<pouco2000_ros::Controller>(topic,Controller::QUEUE_SIZE_SUB);
}

void Controller::callback_controller(){
    switch(this->send_mode){
        case update:
            this->publish();
            break;
        case freq: 
            break;
        default:
            ROS_ERROR("unknown mode");
    }
}

void Controller::publish(){
    this->current_msg.header.seq = this->seq_msg++;
    this->current_msg.header.stamp = ros::Time::now();
    this->pub_controller.publish(this->current_msg);
}

void Controller::run(){
    ros::Rate r(this->sleep_rate);
    while(ros::ok()){
        switch(this->send_mode){
            case freq:  
                this->publish();
        }
        ros::spinOnce();
        r.sleep();
    }
}

void callback::buttons(const type_msg_buttons::ConstPtr& msg,pouco2000_ros::Controller& msg_controller){
    msg_controller.buttons.data = msg->data;
}

void callback::switchs_onoff(const type_msg_switch_onoff::ConstPtr& msg,pouco2000_ros::Controller& msg_controller){
    msg_controller.switchs_on_off.data = msg->data;
}

void callback::switchs_modes(const type_msg_switch_modes::ConstPtr& msg,pouco2000_ros::Controller& msg_controller){
    msg_controller.switchs_mode.data = msg->data;
}

void callback::potentiometers_circle(const type_msg_potentiometers::ConstPtr& msg,pouco2000_ros::Controller& msg_controller,ConvertisserPotentiometerToPercent& c){
    type_msg_potentiometers new_msg;
    new_msg.data = msg->data; 
    msg_controller.potentiometers_circle = convert_potentiometers(c,new_msg);
}

void callback::potentiometers_slider(const type_msg_potentiometers::ConstPtr& msg,pouco2000_ros::Controller& msg_controller,ConvertisserPotentiometerToPercent& c){
    type_msg_potentiometers new_msg;
    new_msg.data = msg->data; 
    msg_controller.potentiometers_slider = convert_potentiometers(c,new_msg);
}