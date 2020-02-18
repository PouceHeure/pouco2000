#include<Arduino.h>
//ros lib 
#include <ros.h>
//msg lib 
#include <pouco2000_ros/SwitchsMode.h>
#include <pouco2000_ros/SwitchsOnOff.h>
#include <pouco2000_ros/Buttons.h>
#include <pouco2000_ros/Potentiometers.h>



/* struct fields */

struct Switch {
  int pin; 
  bool value = false;
};

struct SwitchMode {
  int pin; 
  int value = false;
};

struct Button {
  int pin;
  bool value = false;
};

struct Potentiometer{
  int pin; 
  int value = 0;
};

/* handle */ 

/**
 * @brief Handle a set of element from a same field. 
 * 
 * @tparam T_field struct of the field 
 * @tparam T_data type of data used by the field 
 * @tparam T_msg type of msg sent 
 */
template<typename T_field, typename T_data, typename T_msg>
class Handle{
  private:
    // publisher used to sent data to ROS 
    ros::Publisher* pub;
    T_field* elements;
    T_data* data;
    T_msg msg;
    // number of elements  
    int n_connections;
    // define if the field use digital port or analog port 
    bool is_digital;

  public:
    /**
     * @brief Construct a new Handle object
     * The construction set by default is_digital to true. 
     * @param topic topic where the message will be published 
     * @param connections array of connections
     * @param n_connections number of connections 
     */
    Handle(char* topic,int* connections,int n_connections);

    /**
     * @brief Construct a new Handle object
     * 
     * @param topic topic where the message will be published
     * @param connections array of connections
     * @param n_connections number of connections 
     * @param is_digital if the field use digital or analog port 
     */
    Handle(char* topic,int* connections,int n_connections, bool is_digital);

    /**
     * @brief setup the current handle, declare the publisher to the NodeHandle and 
     * set the pinMode of each pin to INPUT  
     * @param nh current nodehandle 
     */
    void setup(ros::NodeHandle& nh);

    /**
     * @brief update msg used by the handle in checking state of pin 
     * 
     */
    void update();
};

template<typename T_field, typename T_data, typename T_msg>
Handle<T_field,T_data,T_msg>::Handle(char* topic,int* connections,int n_connections,bool _is_digital){

  this->elements = static_cast< T_field* >(malloc(n_connections * sizeof(T_field)));
  this->data = static_cast< T_data* >(malloc(n_connections * sizeof(T_data)));
  this->n_connections = n_connections;
  this->is_digital = _is_digital;
  
  // create a new element for each pin
  for(int i=0;i<this->n_connections;i++){
    T_field* new_element = static_cast< T_field* >(malloc(1 * sizeof(T_field)));
    new_element->pin = connections[i];
    this->elements[i] = *new_element; 
  }

  this->pub = new ros::Publisher(topic,&msg);
}

template<typename T_field, typename T_data, typename T_msg>
Handle<T_field,T_data,T_msg>::Handle(char* topic,int* connections,int n_connections):Handle(topic, connections,n_connections, true){

}

template<typename T_field, typename T_data, typename T_msg>
void Handle<T_field,T_data,T_msg>::setup(ros::NodeHandle& nh){
  nh.advertise(*(this->pub));
  for(int i=0;i<this->n_connections;i++){
    pinMode(elements[i].pin,INPUT);
  }
  msg.data_length = this->n_connections;
}

template<typename T_field, typename T_data, typename T_msg>
void Handle<T_field,T_data,T_msg>::update(){
  // read current state of each pin 
  for(int i=0;i<this->n_connections;i++){
    if(is_digital){
      this->data[i] = digitalRead(elements[i].pin);
    }else{
      this->data[i] = analogRead(elements[i].pin);
    }
    elements[i].value = this->data[i];
  }
  msg.data = this->data;
  pub->publish(&msg);
}

/* typedef */

/**
 * @brief Handle used by Switchs On Off type
 * 
 */
typedef Handle<Switch,pouco2000_ros::SwitchsOnOff::_data_type,pouco2000_ros::SwitchsOnOff> HandleSwitchsOnOff;

/**
 * @brief Handle used by Switchs Mode type
 * 
 */
typedef Handle<SwitchMode,pouco2000_ros::SwitchsMode::_data_type,pouco2000_ros::SwitchsMode> HandleSwitchsMode;

/**
 * @brief Handle used by Buttons 
 * 
 */
typedef Handle<Button,pouco2000_ros::Buttons::_data_type,pouco2000_ros::Buttons> HandleButtons;

/**
 * @brief Handle used by Potentiometers (circle or slider)
 * 
 */
typedef Handle<Potentiometer,pouco2000_ros::Potentiometers::_data_type,pouco2000_ros::Potentiometers> HandlePotentiometers;