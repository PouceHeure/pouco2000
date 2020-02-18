#include <Arduino.h>
#include <ros.h>

#include <pouco2000_ros/SwitchsOnOff.h>
#include <pouco2000_ros/Buttons.h>
#include <pouco2000_ros/Potentiometers.h>

#define DEBUG_SERIAL 


struct Switch {
  int pin; 
  bool value = false;
};

struct Button {
  int pin;
  bool value = false;
};

struct Potentiometer{
  int pin; 
  int value = 0;
};

template<typename T_field, typename T_data, typename T_msg>
class Handle{
  private: 
    ros::Publisher* pub;
    T_field* elements;
    T_data* data;
    T_msg msg;

    int n_connections;
    bool is_digital;
  
  public:
    Handle(char* topic);
    Handle(char* topic,int* connections,int n_connections, bool is_digital);
    void setup(ros::NodeHandle& nh);
    void update();
};

template<typename T_field, typename T_data, typename T_msg>
Handle<T_field,T_data,T_msg>::Handle(char* topic,int* connections,int n_connections,bool _is_digital){

  this->elements = static_cast< T_field* >(malloc(n_connections * sizeof(T_field)));
  this->data = static_cast< T_data* >(malloc(n_connections * sizeof(T_data)));
  this->n_connections = n_connections;
  this->is_digital = _is_digital;
 
  for(int i=0;i<this->n_connections;i++){
    T_field* new_element = static_cast< T_field* >(malloc(1 * sizeof(T_field)));
    new_element->pin = connections[i];
    this->elements[i] = *new_element; 
  }

  this->pub = new ros::Publisher(topic,&msg);
}

template<typename T_field, typename T_data, typename T_msg>
void Handle<T_field,T_data,T_msg>::setup(ros::NodeHandle& nh){
  nh.loginfo("advertise pub");
  nh.advertise(*(this->pub));
  for(int i=0;i<this->n_connections;i++){
    pinMode(elements[i].pin,INPUT);
  }
  msg.data_length = this->n_connections;
}

template<typename T_field, typename T_data, typename T_msg>
void Handle<T_field,T_data,T_msg>::update(){
  T_data tmp_var;
  
  for(int i=0;i<this->n_connections;i++){
    if(is_digital){
      tmp_var = digitalRead(elements[i].pin);
    }else{
      tmp_var = analogRead(elements[i].pin);
    }
    this->data[i] = tmp_var;
    if(tmp_var != elements[i].value){
      elements[i].value = tmp_var;
    }
  }

  msg.data = this->data;
  pub->publish(&msg);
}


typedef Handle<Switch,pouco2000_ros::SwitchsOnOff::_data_type,pouco2000_ros::SwitchsOnOff> HandleSwitchsOnOff;
typedef Handle<Button,pouco2000_ros::Buttons::_data_type,pouco2000_ros::Buttons> HandleButtons;
typedef Handle<Potentiometer,pouco2000_ros::Potentiometers::_data_type,pouco2000_ros::Potentiometers> HandlePotentiometers;