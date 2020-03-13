#include <pouco2000.h>

#define DELAY 50

int switchs_onoff_pin_connections[] = {12,10,8,6,4};
int switchs_mode_pin_connections[] = {11,9,7,5,3};
int potentiometer_circle_pin_connections[] = {A1};

//ros variables  
ros::NodeHandle nh;

HandleSwitchsOnOff handle_switchs_onoff("switchs_onoff",
                                  switchs_onoff_pin_connections,
                                  sizeof(switchs_onoff_pin_connections)/sizeof(int));

HandleSwitchsMode handle_switchs_mode("switchs_modes",
                                  switchs_mode_pin_connections,
                                  sizeof(switchs_mode_pin_connections)/sizeof(int));

HandlePotentiometers handle_potentiometers_circle("potentiometers_circle",
                                                  potentiometer_circle_pin_connections,
                                                  sizeof(potentiometer_circle_pin_connections)/sizeof(int),
                                                  false);

void setup() {
  // setup ros 
  nh.initNode();

  handle_switchs_onoff.setup(nh);
  handle_switchs_mode.setup(nh);
  handle_potentiometers_circle.setup(nh);
}

void loop() {
  handle_switchs_onoff.update();
  handle_switchs_mode.update();
  handle_potentiometers_circle.update();
  
  nh.spinOnce();
  delay(DELAY);
}




