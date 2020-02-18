#include <pouco2000.h>

#define DELAY 50
// create pins array 
int potentiometers_pin_connections[] = {A3,A5};

// nodehandle  
ros::NodeHandle nh;

HandlePotentiometers handle_potentiometers("potentiometers_slider",
                                  potentiometers_pin_connections,
                                  sizeof(potentiometers_pin_connections)/sizeof(int),
                                  false); // isn't a digital pin 

void setup() {
  // setup ros 
  nh.initNode();
  
  // setup handle (attach publisher and set pinMode)
  handle_potentiometers.setup(nh);
}

void loop() {
  // update handle 
  handle_potentiometers.update();
  
  nh.spinOnce();
  delay(DELAY);
}
