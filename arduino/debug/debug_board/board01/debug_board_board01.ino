#include <pouco2000.h>

#define DELAY 50

int switchs_onoff_pins[] = {2,3,4,5,6};
int switchs_mode_pins[] = {7,8,9,10,11};
int potentiometer_circle_pins[] = {A7,A6,A5,A4,A3};

//ros variables  
ros::NodeHandle nh;

HandleSwitchsOnOff handle_switchs_onoff(TOPIC_SWITCHS_ONOFF,
                          switchs_onoff_pins,
                          sizeof(switchs_onoff_pins)/sizeof(int),
                          INPUT);

HandleSwitchsMode handle_switchs_mode(TOPIC_SWITCHS_MODES,
                          switchs_mode_pins,
                          sizeof(switchs_mode_pins)/sizeof(int));

HandlePotentiometers handle_potentiometers_circle(TOPIC_POTENTIOMETERS_CIRCLE,
                          potentiometer_circle_pins,
                          sizeof(potentiometer_circle_pins)/sizeof(int),
                          INPUT);

void setup(){
  // setup ros 
  nh.initNode();
  // setup handle
  handle_switchs_onoff.setup(nh);
  handle_switchs_mode.setup(nh);
  handle_potentiometers_circle.setup(nh);
}

void loop(){
  // update handles
  handle_switchs_onoff.update();
  handle_switchs_mode.update();
  handle_potentiometers_circle.update();
  
  nh.spinOnce();
  delay(DELAY);
}




