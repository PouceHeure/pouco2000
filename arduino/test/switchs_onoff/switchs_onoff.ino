#include <Arduino.h>

#define ESP_SERIAL
#include <ros.h>
#include <pouco2000_ros/SwitchsOnOff.h> 

#define THRESHOLD_ANALOG 500

struct Switch {
  int pin; 
  bool is_on;    
};

/* begin edit region */

const int number_switchs = 4; 

Switch switch_01 = {A0,false};
Switch switch_02 = {2,false};
Switch switch_03 = {2,false};
Switch switch_04 = {2,false};

Switch switchs[number_switchs] = {
    switch_01,
    switch_02,
    switch_03,
    switch_04
};

bool trig_new_msg = false;
bool data[number_switchs];

/* end edit region */

//tmp variables, avoiding allocate and delete continually
int current_value;
bool current_is_on;

//ros variables  
ros::NodeHandle nh;
pouco2000_ros::SwitchsOnOff msg;
ros::Publisher pub("switchs_onoff",&msg);


void setup() {
  // setup ros 
  nh.initNode();
  nh.advertise(pub);
  // setup hardware 
  for(int i=0;i<number_switchs;i++){
    pinMode(switchs[i].pin,INPUT);
  }

}

void loop() {
  //extract sensors values    
  for(int i=0;i<number_switchs;i++){
    current_value = analogRead(switchs[i].pin);
    current_is_on = (current_value > THRESHOLD_ANALOG);
    data[i] = current_is_on;
    if(current_is_on != switchs[i].is_on){
      trig_new_msg = true;
      switchs[i].is_on = current_is_on;
    }
  }

  //send only if a value has been updated 
  if(trig_new_msg){ 
    msg.data = data;
    msg.data_length = number_switchs;
    pub.publish(&msg);
    trig_new_msg = false;
  }
  
  //connection with ros master 
  nh.spinOnce();
  delay(2);
}
