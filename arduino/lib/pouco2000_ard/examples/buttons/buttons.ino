#include <pouco2000.h>

#define DELAY 50
// create pins array 
int buttons_pin_connections[] = {2,5,6};

// nodehandle  
ros::NodeHandle nh;

HandleButtons handle_buttons(TOPIC_BUTTONS,
                      buttons_pin_connections,
                      sizeof(buttons_pin_connections)/sizeof(int));

void setup() {
  // setup ros 
  nh.initNode();
  
  // setup handle (attach publisher and set pinMode)
  handle_buttons.setup(nh);
}

void loop() {
  // update handle 
  handle_buttons.update();
  
  nh.spinOnce();
  delay(DELAY);
}
