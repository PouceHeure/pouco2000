#include <pouco2000.h>

#define DELAY 50
// create pins array 
int switchs_pin_connections[] = {3,2,5,6};

// nodehandle  
ros::NodeHandle nh;

HandleSwitchsMode handle_switchs_mode(TOPIC_SWITCHS_MODES,
                                  switchs_pin_connections,
                                  sizeof(switchs_pin_connections)/sizeof(int));

void setup() {
  // setup ros 
  nh.initNode();
  
  // setup handle (attach publisher and set pinMode)
  handle_switchs_mode.setup(nh);
}

void loop() {
  // update handle 
  handle_switchs_mode.update();
  
  nh.spinOnce();
  delay(DELAY);
}
