#include <pouco2000.h>

#define DELAY 50

int buttons_pins[] = {2,3,4,5,
                      6,7,8,9};

int potentiometer_slider_pins[] = {A7,A6,A4};

//ros variables  
ros::NodeHandle nh;

HandleButtons handle_buttons(TOPIC_BUTTONS,
                     buttons_pins,
                     sizeof(buttons_pins)/sizeof(int));

HandlePotentiometers handle_potentiometers_slider(TOPIC_POTENTIOMETERS_SLIDER,
                            potentiometer_slider_pins,
                            sizeof(potentiometer_slider_pins)/sizeof(int),
                            INPUT);

void setup() {
  // setup ros 
  nh.initNode();
  // setup handles 
  handle_buttons.setup(nh);
  handle_potentiometers_slider.setup(nh);
}

void loop() {
  // update handles
  handle_buttons.update();
  handle_potentiometers_slider.update();
  
  nh.spinOnce();

  delay(DELAY);
}
