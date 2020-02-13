#pragma once 
//lib c++
#include <iostream>
#include <string.h>
#include <vector>
#include <boost/function.hpp>
#include <boost/signals2.hpp>
//lib ros 
#include <ros/ros.h>
//msgs  
#include <pouco2000_ros/Controller.h>
#include <pouco2000_ros/Buttons.h>
#include <pouco2000_ros/SwitchsOnOff.h>
#include <pouco2000_ros/SwitchsMode.h>
#include <pouco2000_ros/Potentiometers.h>
//utest  
#include <gtest/gtest_prod.h>

/*
########## Convertisser ########## 
*/

/**
 * @brief Converter of data
 * 
 * @tparam T_from 
 * @tparam T_to 
 */
template<typename T_from,typename T_to>
class Convertisser{
    public:
        virtual T_to convert(const T_from& value)=0;
};


class ConvertisserPotentiometerToPercent: public Convertisser<float,float>{
    private:
        float min;
        float max; 

    public: 
        ConvertisserPotentiometerToPercent(const float& _min,const float& _max);
        virtual float convert(const float& value);
};

pouco2000_ros::Potentiometers convert_potentiometers(ConvertisserPotentiometerToPercent c,pouco2000_ros::Potentiometers& msg);

/*
########## Controller ########## 
*/

/**
 * @brief Define the mode of sending.  
 * freq: msg is sent to x ms
 * update: wait a new update and send the msg  
 */
enum SendMode {
    freq = 0,
    update
};

/**
 * @brief Regroups callback subscribers used by the Controller class  
 */
namespace callback{
    void buttons(const pouco2000_ros::Buttons::ConstPtr&,pouco2000_ros::Controller&);
    void switchs_onoff(const pouco2000_ros::SwitchsOnOff::ConstPtr&,pouco2000_ros::Controller&);
    void switchs_modes(const pouco2000_ros::SwitchsMode::ConstPtr&,pouco2000_ros::Controller&);
    void potentiometers_circle(const pouco2000_ros::Potentiometers::ConstPtr&,pouco2000_ros::Controller&,ConvertisserPotentiometerToPercent&);
    void potentiometers_slider(const pouco2000_ros::Potentiometers::ConstPtr&,pouco2000_ros::Controller&,ConvertisserPotentiometerToPercent&);
};

class Controller{
    
    private:
 
        long seq_msg = 0;
        int sleep_rate = FREQ_PUBLICATION;
        SendMode send_mode = SendMode::update;
        pouco2000_ros::Controller current_msg;

        // nodehande 
        ros::NodeHandle nh;
        // publisher 
        ros::Publisher pub_controller;
        // subscribers
        ros::Subscriber sub_buttons;
        ros::Subscriber sub_switchs_onoff;
        ros::Subscriber sub_switchs_modes;
        ros::Subscriber sub_potentiometers_circle;
        ros::Subscriber sub_potentiometers_slider;

        /**
         * @brief create a new method, wrapping the callback given with the callback_controller()
         * The wrapper attachs current msg reference to the callback, creating a lambda function 
         * @tparam T_msg type of message of the topic subscribed 
         * @return boost::function<void(const T_msg&)> wrapper callback 
         */
        template<typename T_msg>
        boost::function<void(const T_msg&)> wrapper(boost::function<void(const T_msg,pouco2000_ros::Controller&)>);

        /**
         * @brief publish current_msg and fill informations (seq and stamp)
         */
        void publish();

        // give an access to private contents 
        FRIEND_TEST(TestPouco2000Lib, testWrapper);

    public: 

        const static int QUEUE_SIZE_SUB = 10;
        const static int FREQ_PUBLICATION = 10;  

        Controller();

        // setters 
        void set_node_handle(ros::NodeHandle& nh);
        void set_send_mode(SendMode send_mode);
        void set_sleep_rate(int rate);
        void set_pub_controller(std::string topic);
        void set_sub_buttons(std::string topic,boost::function<void(const pouco2000_ros::Buttons::ConstPtr&,pouco2000_ros::Controller& current_msg)>);
        void set_sub_switchs_onoff(std::string topic, boost::function<void(const pouco2000_ros::SwitchsOnOff::ConstPtr&,pouco2000_ros::Controller& current_msg)>);
        void set_sub_switchs_modes(std::string topic, boost::function<void(const pouco2000_ros::SwitchsMode::ConstPtr&,pouco2000_ros::Controller& current_msg)>);
        void set_sub_potentiometers_circle(std::string topic, boost::function<void(const pouco2000_ros::Potentiometers::ConstPtr&,pouco2000_ros::Controller& current_msg)>);
        void set_sub_potentiometers_slider(std::string topic, boost::function<void(const pouco2000_ros::Potentiometers::ConstPtr&,pouco2000_ros::Controller& current_msg)>);

        /**
         * @brief Regroups common instructions between all callback, like publish instruction. 
         */
        void callback_controller();

        /**
         * @brief start communication with ros 
         */
        void run();  
        
};

template<typename T_msg>
boost::function<void(const T_msg&)> Controller::wrapper(boost::function<void(const T_msg, pouco2000_ros::Controller& current_msg)> sub_callback){
    // attach current_msg reference 
    boost::function<void(const T_msg&)> sub_callback_attach_msg = boost::bind(sub_callback,_1,boost::ref(current_msg));
    // wrap callback_sub and callback_controller
    boost::function<void(const T_msg&)> wrapper_callbacks = [this,sub_callback_attach_msg](const T_msg& msg){
        sub_callback_attach_msg(msg);
        callback_controller();
    };
    return  wrapper_callbacks;
}



