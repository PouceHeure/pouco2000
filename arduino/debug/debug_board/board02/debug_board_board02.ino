#include <pouco2000.h>

#define DELAY 50

int buttons_pin_connections[] = {8,7,6,5};
int potentiometer_slider_pin_connections[] = {A0};

//ros variables  
ros::NodeHandle nh;

HandleButtons handle_buttons("buttons",
                              buttons_pin_connections,
                              sizeof(buttons_pin_connections)/sizeof(int),
                              true);


HandlePotentiometers handle_potentiometers_slider("potentiometers_slider",
                                            potentiometer_slider_pin_connections,
                                            sizeof(potentiometer_slider_pin_connections)/sizeof(int),
                                            false);



void setup() {
  // setup ros 
  nh.initNode();

  handle_buttons.setup(nh);
  handle_potentiometers_slider.setup(nh);
}

void loop() {
  handle_buttons.update();
  handle_potentiometers_slider.update();
  
  nh.spinOnce();

  delay(DELAY);
}
