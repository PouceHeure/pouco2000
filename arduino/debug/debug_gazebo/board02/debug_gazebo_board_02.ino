#include <pouco2000.h>

#define DELAY 50

int switchs_onoff_pin_connections[] = {2};
int switchs_mode_pin_connections[] = {4};

//ros variables  
ros::NodeHandle nh;

HandleSwitchsOnOff handle_switchs_onoff("switchs_onoff",
                                  switchs_onoff_pin_connections,
                                  sizeof(switchs_onoff_pin_connections)/sizeof(int));

HandleSwitchsMode handle_switchs_mode("switchs_modes",
                                  switchs_mode_pin_connections,
                                  sizeof(switchs_mode_pin_connections)/sizeof(int));



void setup() {
  // setup ros 
  nh.initNode();

  handle_switchs_onoff.setup(nh);
  handle_switchs_mode.setup(nh);
}

void loop() {
  handle_switchs_onoff.update();
  handle_switchs_mode.update();
  
  nh.spinOnce();
  delay(DELAY);
}
