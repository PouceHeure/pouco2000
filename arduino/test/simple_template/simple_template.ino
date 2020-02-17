#include <Arduino.h>

#define ESP_SERIAL


#include <pouco2000.h>

std::vector<int> switchs_connexions = {5,4,3};

HandleSwitchsOnOff handle_switchs("switchs_onoff",switchs_connexions,3,true);


//ros variables  
ros::NodeHandle nh;

void setup() {
  // setup ros  
  nh.initNode();
  handle_switchs.setup(nh);
}

void loop() {
   handle_switchs.update();
  
  //connection with ros master 
  nh.spinOnce();
  delay(2);
}
