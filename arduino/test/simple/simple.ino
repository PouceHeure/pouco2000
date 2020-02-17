#include <Arduino.h>

#define ESP_SERIAL
#include <ros.h>
#include <pouco2000_ros/SwitchsOnOff.h> 
#include <pouco2000_ros/Buttons.h> 
#include <pouco2000_ros/Potentiometers.h> 

#define THRESHOLD_ANALOG 500

struct Switch {
  int pin; 
  bool is_on;    
};

struct Button {
  int pin;
  bool is_pushed;
};

struct Potentiometer{
  int pin; 
  int value;
};

/* begin edit region */

const int number_switchs = 1; 
bool data_switchs[number_switchs];
Switch switch_01 = {5,false};
Switch switchs[number_switchs] = {
    switch_01
};

bool trig_new_msg_switch = false;

const int number_buttons = 1;
bool data_buttons[number_buttons];
Button button_01 = {4,false};
Button buttons[number_buttons] = {
    button_01
};

bool trig_new_msg_button = false;

const int number_potentiometers = 1;
float data_potentiometers[number_potentiometers];
Potentiometer potentiometer_01 = {A0,false};
Potentiometer potentiometers[number_potentiometers] = {
    potentiometer_01
};

bool trig_new_msg_potentiometer = false;


/* end edit region */

//tmp variables, avoiding allocate and delete continually
int current_value;
bool current_is_on;

//ros variables  
ros::NodeHandle nh;
pouco2000_ros::SwitchsOnOff msg_switchs;
pouco2000_ros::Buttons msg_buttons;
pouco2000_ros::Potentiometers msg_potentiometers;

ros::Publisher pub_switchs("switchs_onoff",&msg_switchs);
ros::Publisher pub_buttons("buttons",&msg_buttons);
ros::Publisher pub_potentiometers("potentiometers_slider",&msg_potentiometers);


void setup() {
  // setup ros  
  nh.initNode();
  nh.advertise(pub_switchs);
  nh.advertise(pub_buttons);
  nh.advertise(pub_potentiometers);
  // setup hardware 
  for(int i=0;i<number_switchs;i++){
    pinMode(switchs[i].pin,INPUT);
  }

  for(int i=0;i<number_buttons;i++){
    pinMode(buttons[i].pin,INPUT);
  }

  for(int i=0;i<number_potentiometers;i++){
    pinMode(potentiometers[i].pin,INPUT);
  }

}

void loop() {
  //extract sensors values    
  for(int i=0;i<number_switchs;i++){
    current_is_on = digitalRead(switchs[i].pin);
    data_switchs[i] = current_is_on;
    if(current_is_on != switchs[i].is_on){
      trig_new_msg_switch = true;
      switchs[i].is_on = current_is_on;
    }
  }

  //extract sensors values    
  for(int i=0;i<number_potentiometers;i++){
    int value = analogRead(potentiometers[i].pin);
    data_potentiometers[i] = value;
    if(value != potentiometers[i].value){
      trig_new_msg_potentiometer = true;
      potentiometers[i].value = value;
    }
  }

  //extract sensors values    
  for(int i=0;i<number_buttons;i++){
    current_is_on = digitalRead(buttons[i].pin);
    data_buttons[i] = current_is_on;
    if(current_is_on != buttons[i].is_pushed){
      trig_new_msg_button = true;
      buttons[i].is_pushed = current_is_on;
    }
  }

  //send only if a value has been updated 
  if(trig_new_msg_switch){ 
    msg_switchs.data = data_switchs;
    msg_switchs.data_length = number_switchs;
    pub_switchs.publish(&msg_switchs);
    trig_new_msg_switch = false;
  }

  if(trig_new_msg_button){ 
    msg_buttons.data = data_buttons;
    msg_buttons.data_length = number_buttons;
    pub_buttons.publish(&msg_buttons);
    trig_new_msg_button = false;
  }

  if(trig_new_msg_potentiometer){ 
    msg_potentiometers.data = data_potentiometers;
    msg_potentiometers.data_length = number_potentiometers;
    pub_potentiometers.publish(&msg_potentiometers);
    trig_new_msg_potentiometer = false;
  }
  
  //connection with ros master 
  nh.spinOnce();
  delay(2);
}
