#pragma once 
//lib cpp
#include <boost/function.hpp>
//lib ros 
#include <ros/ros.h>
//msgs  
#include <pouco2000_ros/Controller.h>
#include <pouco2000_ros/Buttons.h>
#include <pouco2000_ros/SwitchsOnOff.h>
#include <pouco2000_ros/SwitchsMode.h>
#include <pouco2000_ros/Potentiometers.h>

/**
 * @brief 
 * 
 */
enum Field{
    buttons,
    switchs_on_off,
    switchs_modes,
    potentiometers_circle,
    potentiometers_slider
};

template<typename T>
T default_msg();

template<typename T>
T default_msg(){
    T value;
    return value;
}

template<typename T> 
T extract_field(const pouco2000_ros::Controller::ConstPtr& msg,Field v);

template<typename T> 
T extract_field(const pouco2000_ros::Controller::ConstPtr& msg,Field v){
    ROS_ERROR("failed extraction");
    return default_msg<T>();
}

template<> 
pouco2000_ros::Buttons extract_field(const pouco2000_ros::Controller::ConstPtr& msg,Field v);

template<> 
pouco2000_ros::Potentiometers extract_field(const pouco2000_ros::Controller::ConstPtr& msg,Field v);

template<> 
pouco2000_ros::SwitchsMode extract_field(const pouco2000_ros::Controller::ConstPtr& msg,Field v);

template<> 
pouco2000_ros::SwitchsOnOff extract_field(const pouco2000_ros::Controller::ConstPtr& msg,Field v);


/**
 * @brief Class filtering data from controller msg 
 * This class can be used for plotting some data from controller msg 
 * @tparam T_msg_send type of msg send once filtering done 
 * @tparam T_msg_field type of field, where data is located, for example: pouco2000_ros::Buttons
 */
template<typename T_msg_send,typename T_msg_field>
class FilterPublisher{
    private: 
        static const int QUEUE_SIZE_PUBLISHER = 10;
        static const int QUEUE_SIZE_SUBSCRIBER = 100;
        ros::Publisher pub;
        ros::Subscriber sub;
        void callback(const pouco2000_ros::Controller::ConstPtr& msg, 
                      ros::Publisher& pub,
                      Field field,
                      int position);
    public:
        /**
         * @brief Construct a new Filter Publisher object
         * 
         * @param n the current handlenode 
         * @param topic_sub topic name where controller msg is sent 
         * @param topic_pub topic name where filter will publish 
         * @param field field of element filter, for example buttons 
         * @param position position in the array of this field
         */
        FilterPublisher(ros::NodeHandle& n,std::string topic_sub, std::string topic_pub,Field field,int position);
        /**
         * @brief Run the current FilterPublisher, in starting the communication with ros master 
         */
        void run();
};

template<typename T_msg_send,typename T_msg_field>
FilterPublisher<T_msg_send,T_msg_field>::FilterPublisher(ros::NodeHandle& n,std::string topic_sub, std::string topic_pub,Field field,int position){
    this->pub = n.advertise<T_msg_send>(topic_pub,FilterPublisher::QUEUE_SIZE_PUBLISHER);
    auto callback_bind = boost::bind(&FilterPublisher<T_msg_send,T_msg_field>::callback,this,_1,boost::ref(pub),field,position);
    this->sub = n.subscribe<pouco2000_ros::Controller>(topic_sub,FilterPublisher::QUEUE_SIZE_SUBSCRIBER,callback_bind);
}

template<typename T_msg_send,typename T_msg_field>
void FilterPublisher<T_msg_send,T_msg_field>::callback(const pouco2000_ros::Controller::ConstPtr& msg, 
                       ros::Publisher& pub,
                       Field field,
                       int position){

    T_msg_field filter_msg = extract_field<T_msg_field>(msg,field); 
    if(filter_msg.data.size() > position){
        T_msg_send msg_send;
        msg_send.data = filter_msg.data.at(position);
        pub.publish(msg_send);
    }else{
        ROS_ERROR("out of range");
    }
}

template<typename T_msg_send,typename T_msg_field>
void FilterPublisher<T_msg_send,T_msg_field>::run(){
    ros::spin();
}



