#pragma once 
//lib cpp
#include <ctime>   
#include <cstdlib>
//lib ros 
#include <ros/ros.h>

// information allowing to load param about rate sleep 
#define DEFAULT_RATE 10
#define KEY_PARAM_RATE "~rate"

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
