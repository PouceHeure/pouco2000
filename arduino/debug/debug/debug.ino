#include <pouco2000.h>

#define DELAY 50

int switchs_pins[] = {3,1,5,6};
int buttons_pins[] = {2,7,8};
int potentiometer_circle_pins[] = {A3,A5};

//ros variables  
ros::NodeHandle nh;

HandleSwitchsOnOff handle_switchs(TOPIC_SWITCHS_ONOFF,
                          switchs_pins,
                          sizeof(switchs_pins)/sizeof(int));

HandleButtons handle_buttons(TOPIC_BUTTON,
                     buttons_pins,
                     sizeof(buttons_pins)/sizeof(int),
                     true);

HandlePotentiometers handle_potentiometers(TOPIC_POTENTIOMETERS_CIRCLE,
                            potentiometer_circle_pins,
                            sizeof(potentiometer_circle_pins)/sizeof(int),
                            false);

void setup() {
  // setup ros 
  nh.initNode();
  // setup handles 
  handle_switchs.setup(nh);
  handle_buttons.setup(nh);
  handle_potentiometers.setup(nh);
}

void loop() {
  // update handles 
  handle_switchs.update();
  handle_buttons.update();
  handle_potentiometers.update();
  
  nh.spinOnce();

  delay(DELAY);
}