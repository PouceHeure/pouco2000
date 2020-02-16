#include "ros/ros.h"
#include "pouco2000_ros/pouco2000.hpp"

// keys param 
#define KEY_PARAM_POTENTIOMETERS_CIRCLE "potentiometers/circle/"
#define KEY_PARAM_POTENTIOMETERS_SLIDER "potentiometers/slider/"
#define KEY_PARAM_RATE "~rate"

/**
 * @brief load parameter converter from rosparam 
 * 
 * @param key the key where is define the param 
 * @param d_min default min value 
 * @param d_max default max value 
 * @param min result min value 
 * @param max result max value 
 */
void load_parameter_converter(const std::string key,
                              const int d_min, const int d_max, 
                              int& min, int& max){
  XmlRpc::XmlRpcValue parameters;
  if(ros::param::get(key,parameters)){
    min = parameters["min"];
    max = parameters["max"];
  }else{
    min = d_min;
    max = d_max;
  }
}

/**
 * @brief load parameter rate from rosparam 
 * if a rate is given, the controller switch to the freq send mode
 * by default the node publishs once it receives a message 
 * 
 * @param c 
 * @param key 
 */
void load_parameter_rate(Controller& c,std::string key){
  int rate;
  if(ros::param::get(key,rate)){
    c.set_sleep_rate(rate);
    c.set_send_mode(SendMode::freq);
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle n;

  // init convertissers
  int circle_min; int circle_max;
  int slider_min; int slider_max;
  load_parameter_converter(KEY_PARAM_POTENTIOMETERS_CIRCLE,0,1024,circle_min,circle_max);
  load_parameter_converter(KEY_PARAM_POTENTIOMETERS_SLIDER,0,1024,slider_min,slider_max);
  ConvertisserPotentiometerToPercent convert_potetiomerters_circle(circle_min,circle_max);
  ConvertisserPotentiometerToPercent convert_potetiomerters_slider(slider_min,slider_max);

  // init controller  
  Controller controller;
  // essential sets 
  load_parameter_rate(controller,KEY_PARAM_RATE);
  controller.set_node_handle(n);
  controller.set_pub_controller("controller");
  controller.set_sub_buttons("buttons",callback::buttons); 
  // buttons 
  // switchs
  controller.set_sub_switchs_onoff("switchs_onoff",callback::switchs_onoff);
  controller.set_sub_switchs_modes("switchs_modes",callback::switchs_modes);
  // potentiometers
  auto callback_attach_convert_circle = boost::bind(&callback::potentiometers_circle,_1,_2,boost::ref(convert_potetiomerters_circle));
  controller.set_sub_potentiometers_circle("potentiometers_circle",callback_attach_convert_circle);
  auto callback_attach_convert_slider = boost::bind(&callback::potentiometers_slider,_1,_2,boost::ref(convert_potetiomerters_slider));
  controller.set_sub_potentiometers_slider("potentiometers_slider",callback_attach_convert_slider);
  
  // run controller, starting the communication with ros    
  controller.run();

  return 0;
}