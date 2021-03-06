#include <pouco2000.h>

#define DELAY 50

int switchs_pin_connections[] = {3,1,5,6};
int buttons_pin_connections[] = {2,7,8};
int potentiometer_circle_pin_connections[] = {A3,A5};

//ros variables  
ros::NodeHandle nh;

HandleSwitchsOnOff handle_switchs("switchs_onoff",
                                  switchs_pin_connections,
                                  sizeof(switchs_pin_connections)/sizeof(int));

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

  handle_switchs.setup(nh);
  handle_buttons.setup(nh);
  handle_potentiometers.setup(nh);
}

void loop() {
  handle_switchs.update();
  handle_buttons.update();
  handle_potentiometers.update();
  
  nh.spinOnce();

  delay(DELAY);
}
