#include <pouco2000.h>

#define DELAY 50

int buttons_pin_connections[] = {9,7,4,3};
int potentiometer_circle_pin_connections[] = {A5,A7};

//ros variables  
ros::NodeHandle nh;

HandleButtons handle_buttons("buttons",
                              buttons_pin_connections,
                              sizeof(buttons_pin_connections)/sizeof(int),
                              true);

HandlePotentiometers handle_potentiometers("potentiometers_circle",
                                            potentiometer_circle_pin_connections,
                                            sizeof(potentiometer_circle_pin_connections)/sizeof(int),
                                            false);

void setup() {
  // setup ros 
  nh.initNode();

  handle_buttons.setup(nh);
  handle_potentiometers.setup(nh);
}

void loop() {
  handle_buttons.update();
  handle_potentiometers.update();
  
  nh.spinOnce();

  delay(DELAY);
}
