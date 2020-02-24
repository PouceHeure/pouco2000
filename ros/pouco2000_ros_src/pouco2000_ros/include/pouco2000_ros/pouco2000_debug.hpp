#pragma once 
//lib cpp
#include <ctime>   
#include <cstdlib>
#include <boost/bind.hpp>
//lib ros 
#include <ros/ros.h>


// information allowing to load param about rate sleep 
#define DEFAULT_RATE 10
#define KEY_PARAM_RATE "~rate"

/**
 * @brief random namespace, including init(seed) and gen(template) methods 
 */
namespace rnd{

    void init();

    template<typename T>
    T gen(int begin, int end); 

    template<typename T>
    T gen(int begin, int end){
        ROS_ERROR("unknow method, need specialization");
        return NULL;
    }

    template<>
    int gen(int begin, int end);

    template<>
    float gen(int begin, int end);
}

int load_param_rate(const std::string key,const int& default_value);


/**
 * @brief Initalise easily a fake publisher, class used on debug mode. 
 * Avoiding to develop continually with the hardware part
 * 
 * @tparam T_msg 
 */
template<typename T_msg>
class FakePublisher {
    private: 
        static const int QUEUE_SIZE_PUBLISHER = 1000;
        int rate;
        ros::Publisher pub;
    public: 
        FakePublisher(ros::NodeHandle& nh, std::string topic);

        template<typename T_data>
        void run(int size_data,boost::function<T_data(void)> gen);
};

template<typename T_msg>
FakePublisher<T_msg>::FakePublisher(ros::NodeHandle& nh, std::string topic){
    this->pub = nh.advertise<T_msg>(topic, FakePublisher::QUEUE_SIZE_PUBLISHER);
    this->rate = load_param_rate(KEY_PARAM_RATE,DEFAULT_RATE);
};

template<typename T_msg>
template<typename T_data>
void FakePublisher<T_msg>::run(int size_data,boost::function<T_data(void)> gen){
    ros::Rate r(this->rate);
    while (ros::ok())
    {   
        try{
            T_msg msg; 
            for(int i=0;i<size_data;i++){
                 msg.data.push_back(gen());
            }
            this->pub.publish(msg);
        }catch(...){
            ROS_ERROR("can't generate new msg");
        }
        r.sleep();
        ros::spinOnce();
    }
}

