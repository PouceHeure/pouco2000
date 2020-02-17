#include <vector>
#include <Arduino.h>
#include <ros.h>

#include <pouco2000_ros/SwitchsOnOff.h>



struct Switch {
  int pin; 
  bool value = false;
};

struct Button {
  int pin;
  bool value;
};

struct Potentiometer{
  int pin; 
  int value;
};


template<typename T_field, typename T_data, typename T_msg>
class Handle{
  private: 
    ros::Publisher* pub;
    std::vector<T_field> elements;
    std::vector<T_data> data;
    T_msg msg;

    bool is_digital;
  
  public:
    Handle(char* topic);
    Handle(char* topic,int connections[],int n_connections, bool is_digital);
    void add(const T_field element);
    void setup(ros::NodeHandle& nh);
    void update();
};

template<typename T_field, typename T_data, typename T_msg>
Handle<T_field,T_data,T_msg>::Handle(char* topic,int connections[],int n_connections,bool _is_digital){
  this->is_digital = _is_digital;
  this->pub = new ros::Publisher(topic,&msg);
  int n = sizeof(connections)/sizeof(int);
  for(int i=0;i<n;i++){
    T_field* new_element = new T_field();
    new_element->pin = connections[i];
    //new_element->value = false;
    this->elements.push_back(*new_element); 
    //this->add(*new_element);
  }
}

template<typename T_field, typename T_data, typename T_msg>
void Handle<T_field,T_data,T_msg>::add(const T_field element){
  this->elements.push_back(element);
}

template<typename T_field, typename T_data, typename T_msg>
void Handle<T_field,T_data,T_msg>::setup(ros::NodeHandle& nh){
  nh.advertise(*(this->pub));
  for(int i=0;i<elements.size();i++){
    pinMode(elements[i].pin,INPUT);
  }
  data.reserve(elements.size());
  msg.data_length = elements.size();
}

template<typename T_field, typename T_data, typename T_msg>
void Handle<T_field,T_data,T_msg>::update(){
  T_data tmp_var;
  T_data local_data[elements.size()] ;
  for(int i=0;i<elements.size();i++){
    if(is_digital){
      tmp_var = digitalRead(elements[i].pin);
    }else{
      tmp_var = digitalRead(elements[i].pin);
    }
    local_data[i] = tmp_var;
    if(tmp_var != elements[i].value){
      //trig_new_msg_switch = true;
      elements[i].value = tmp_var;
    }
  }

  msg.data = local_data;
  pub->publish(&msg);
}


typedef Handle<Switch,bool,pouco2000_ros::SwitchsOnOff> HandleSwitchsOnOff;