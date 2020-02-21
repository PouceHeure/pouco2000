#pragma once 

#include<pouco2000_ros/Controller.h>


template<typename T_data, typename T_msg_field>
class Extractor{
    private: 
        int index;
        T_data previous_data;
        
    protected:
        virtual T_msg_field extract_field(const pouco2000_ros::Controller::ConstPtr& msg)=0;

    public: 
        Extractor(int _index);
        bool extract_only_change(const pouco2000_ros::Controller::ConstPtr& msg, T_data& result);
        bool extract(const pouco2000_ros::Controller::ConstPtr& msg, T_data& result);
};

template<typename T_data, typename T_msg_field>
Extractor<T_data,T_msg_field>::Extractor(int _index):index(_index){

} 

template<typename T_data, typename T_msg_field>
bool Extractor<T_data,T_msg_field>::extract(const pouco2000_ros::Controller::ConstPtr& msg, T_data& result){
    T_msg_field data_field = extract_field(msg);
    if(data_field.data.size() > index){
        T_data data = data_field.data.at(index); 
        result = data;
        return true;
    }
    return false;
}


template<typename T_data, typename T_msg_field>
bool Extractor<T_data,T_msg_field>::extract_only_change(const pouco2000_ros::Controller::ConstPtr& msg, T_data& result){
    T_data data_extract;
    if(extract(msg,data_extract)){
        if(data_extract != previous_data){
            previous_data = data_extract;
            result = data_extract;
            return true;
        }
    }
    return false;
}


class ExtractorButton: public Extractor<bool,pouco2000_ros::Buttons>{
    protected:
        pouco2000_ros::Buttons extract_field(const pouco2000_ros::Controller::ConstPtr& msg);
    
    public:
        ExtractorButton(int index);
        bool is_push(const pouco2000_ros::Controller::ConstPtr& msg);
};


class ExtractorSwitchsOnOff: public Extractor<bool,pouco2000_ros::SwitchsOnOff>{
    protected:
        pouco2000_ros::SwitchsOnOff extract_field(const pouco2000_ros::Controller::ConstPtr& msg);
    
    public:
        ExtractorSwitchsOnOff(int index);
        bool is_on(const pouco2000_ros::Controller::ConstPtr& msg);
};

class ExtractorSwitchsMode: public Extractor<int,pouco2000_ros::SwitchsMode>{
    protected:
        pouco2000_ros::SwitchsMode extract_field(const pouco2000_ros::Controller::ConstPtr& msg);
    
    public:
        ExtractorSwitchsMode(int index);
        bool is_mode(const pouco2000_ros::Controller::ConstPtr& msg,const int& mode);
};

class ExtractorPotentiometersCircle: public Extractor<float,pouco2000_ros::Potentiometers>{
    protected:
        pouco2000_ros::Potentiometers extract_field(const pouco2000_ros::Controller::ConstPtr& msg);
    
    public:
        ExtractorPotentiometersCircle(int index);
};

class ExtractorPotentiometersSlider: public Extractor<float,pouco2000_ros::Potentiometers>{
    protected:
        pouco2000_ros::Potentiometers extract_field(const pouco2000_ros::Controller::ConstPtr& msg);
    
    public:
        ExtractorPotentiometersSlider(int index);
};
