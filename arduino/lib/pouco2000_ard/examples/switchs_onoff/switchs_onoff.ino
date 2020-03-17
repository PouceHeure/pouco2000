#include <pouco2000.h>

#define DELAY 50
// create pins array 
int switchs_pin_connections[] = {3,1,5,6};

// nodehandle  
ros::NodeHandle nh;

HandleSwitchsOnOff handle_switchs(TOPIC_SWITCHS_ONOFF,
                                  switchs_pin_connections,
                                  sizeof(switchs_pin_connections)/sizeof(int));

void setup() {
  // setup ros 
  nh.initNode();
  
  // setup handle (attach publisher and set pinMode)
  handle_switchs.setup(nh);
}

void loop() {
  // update handle 
  handle_switchs.update();
  
  nh.spinOnce();
  delay(DELAY);
}
