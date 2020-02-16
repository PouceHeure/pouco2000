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
 * @brief Functions allowing to extract field from controller msg 
 * If a new field is create inside the controller msg, a new extract method need to be created  
 */
namespace extract {
    pouco2000_ros::Buttons field_buttons(const pouco2000_ros::Controller::ConstPtr& msg);
    pouco2000_ros::SwitchsOnOff field_switchs_onoff(const pouco2000_ros::Controller::ConstPtr& msg);
    pouco2000_ros::SwitchsMode field_switchs_modes(const pouco2000_ros::Controller::ConstPtr& msg);
    pouco2000_ros::Potentiometers field_potentiometers_circle(const pouco2000_ros::Controller::ConstPtr& msg);
    pouco2000_ros::Potentiometers field_potentiometers_slider(const pouco2000_ros::Controller::ConstPtr& msg);
};

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
                      boost::function< T_msg_field (const pouco2000_ros::Controller::ConstPtr& msg)> extracter,
                      int position);
    public:
        /**
         * @brief Construct a new Filter Publisher object
         * 
         * @param n the current handlenode 
         * @param topic_sub topic name where controller msg is sent 
         * @param topic_pub topic name where filter will publish 
         * @param extracter function allowing to extract the field of a controller msg 
         * @param position position in the array of this field
         */
        FilterPublisher(ros::NodeHandle& n,
                        std::string topic_sub, 
                        std::string topic_pub,
                        boost::function< T_msg_field (const pouco2000_ros::Controller::ConstPtr& msg)> extracter,
                        int position);      
};

template<typename T_msg_send,typename T_msg_field>
FilterPublisher<T_msg_send,T_msg_field>::FilterPublisher(ros::NodeHandle& n,
                                                         std::string topic_sub, 
                                                         std::string topic_pub,
                                                         boost::function< T_msg_field (const pouco2000_ros::Controller::ConstPtr& msg)> extracter,
                                                         int position){
    this->pub = n.advertise<T_msg_send>(topic_pub,FilterPublisher::QUEUE_SIZE_PUBLISHER);
    auto callback_bind = boost::bind(&FilterPublisher<T_msg_send,T_msg_field>::callback,this,_1,boost::ref(pub),extracter,position);
    this->sub = n.subscribe<pouco2000_ros::Controller>(topic_sub,FilterPublisher::QUEUE_SIZE_SUBSCRIBER,callback_bind);
}

template<typename T_msg_send,typename T_msg_field>
void FilterPublisher<T_msg_send,T_msg_field>::callback(const pouco2000_ros::Controller::ConstPtr& msg, 
                       ros::Publisher& pub,
                       boost::function< T_msg_field (const pouco2000_ros::Controller::ConstPtr& msg)> extracter,
                       int position){

    T_msg_field filter_msg = extracter(msg); 
    if(filter_msg.data.size() > position){
        T_msg_send msg_send;
        msg_send.data = filter_msg.data.at(position);
        pub.publish(msg_send);
    }else{
        ROS_WARN("[topic:%s] [index:%d] error: out of range",this->pub.getTopic().c_str(),position);
    }
}