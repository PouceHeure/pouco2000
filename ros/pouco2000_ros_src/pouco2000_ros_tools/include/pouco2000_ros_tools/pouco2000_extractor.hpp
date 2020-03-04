#pragma once 

#include<pouco2000_ros_msgs/Controller.h>

/**
 * @brief extract data from msg controller 
 * 
 * @tparam T_data type of data extract 
 * @tparam T_msg_field type of field where the data will be extracted 
 */
template<typename T_data, typename T_msg_field>
class Extractor{
    private: 
        int index;
        
    protected:
        T_data previous_data;
        /**
         * @brief extract the data of the field from the controller msg  
         * 
         * @param msg controller msg 
         * @return T_msg_field 
         */
        virtual T_msg_field extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg)=0;

    public: 
        Extractor(int _index);
        /**
         * @brief extract the value from msg only if the data has been updated 
         * 
         * @param msg controller msg 
         * @param result result of the extraction 
         * @return true trig an update  
         * @return false update hasn't been done  
         */
        bool extract_only_change(const pouco2000_ros_msgs::Controller::ConstPtr& msg, T_data& result);

        /**
         * @brief extract the value if it's possible to do
         * 
         * @param msg controller msg 
         * @param result result of the extraction 
         * @return true extraction is possible 
         * @return false extraction isn't possible 
         */
        bool extract(const pouco2000_ros_msgs::Controller::ConstPtr& msg, T_data& result);
};

template<typename T_data, typename T_msg_field>
Extractor<T_data,T_msg_field>::Extractor(int _index):index(_index){

} 

template<typename T_data, typename T_msg_field>
bool Extractor<T_data,T_msg_field>::extract(const pouco2000_ros_msgs::Controller::ConstPtr& msg, T_data& result){
    T_msg_field data_field = extract_field(msg);
    // check data is available 
    if(data_field.data.size() > index){
        T_data data = data_field.data.at(index); 
        result = data;
        return true;
    }
    return false;
}

template<typename T_data, typename T_msg_field>
bool Extractor<T_data,T_msg_field>::extract_only_change(const pouco2000_ros_msgs::Controller::ConstPtr& msg, T_data& result){
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

/**
 * @brief Extractor for buttons field 
 * 
 */
class ExtractorButton: public Extractor<bool,pouco2000_ros_msgs::Buttons>{
    protected:
        pouco2000_ros_msgs::Buttons extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg);
    
    public:
        ExtractorButton(int index);

        /**
         * @brief check if the button has been pushed or not 
         * 
         * @param msg controller msg 
         * @return true the button has been pushed
         * @return false the button hasn't been pushed 
         */
        bool is_push(const pouco2000_ros_msgs::Controller::ConstPtr& msg);
}; 

/**
 * @brief Extractor for switchs_on_off field
 * 
 */
class ExtractorSwitchOnOff: public Extractor<bool,pouco2000_ros_msgs::SwitchsOnOff>{
    protected:
        pouco2000_ros_msgs::SwitchsOnOff extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg);
    
    public:
        ExtractorSwitchOnOff(int index);

        /**
         * @brief check is the switch is on position "on" 
         * 
         * @param msg controller msg 
         * @return true switch on "on"
         * @return false switch on "off"
         */
        bool is_on(const pouco2000_ros_msgs::Controller::ConstPtr& msg);
};


/**
 * @brief Extractor for switchs_mode field 
 * 
 */
class ExtractorSwitchMode: public Extractor<int,pouco2000_ros_msgs::SwitchsMode>{
    protected:
        pouco2000_ros_msgs::SwitchsMode extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg);
    
    public:
        ExtractorSwitchMode(int index);

        /**
         * @brief check if the switch is on the mode given 
         * 
         * @param msg controller msg 
         * @param mode compare mode 
         * @return true the switch is on the mode 
         * @return false the switch isn't on the mode 
         */
        bool is_mode(const pouco2000_ros_msgs::Controller::ConstPtr& msg,const int& mode);
};



/**
 * @brief Extractor for potentiometers_circle field 
 * 
 */
class ExtractorPotentiometerCircle: public Extractor<float,pouco2000_ros_msgs::Potentiometers>{
    protected:
        pouco2000_ros_msgs::Potentiometers extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg);
    
    public:
        ExtractorPotentiometerCircle(int index);
};


/**
 * @brief Extracor for potentiometers_slider field
 * 
 */
class ExtractorPotentiometerSlider: public Extractor<float,pouco2000_ros_msgs::Potentiometers>{
    protected:
        pouco2000_ros_msgs::Potentiometers extract_field(const pouco2000_ros_msgs::Controller::ConstPtr& msg);
    
    public:
        ExtractorPotentiometerSlider(int index);
};


/**
 * @brief Handle all extractors on a same objects 
 * The allocations of each extractor depend if the extractor need to be called avoiding large useless allocation
 * 
 */
class HandleExtractors{
    private: 
        // extractors map variables 
        std::map<int,ExtractorButton*> map_extractors_button;
        std::map<int,ExtractorSwitchOnOff*> map_extractors_switchs_onoff;
        std::map<int,ExtractorSwitchMode*> map_extractors_switchs_modes;
        std::map<int,ExtractorPotentiometerCircle*> map_extractors_potentiometers_circle;
        std::map<int,ExtractorPotentiometerSlider*> map_extractors_potentiometers_slider;

        template<typename T_extractor>
        void init_element(int index, std::map<int,T_extractor*>& map);

    public: 
        HandleExtractors();

        // getters for each field extractors
        ExtractorButton* get_button(int index);
        ExtractorSwitchOnOff* get_switchs_onoff(int index);
        ExtractorSwitchMode* get_switchs_modes(int index);
        ExtractorPotentiometerCircle* get_potentiometers_circle(int index);
        ExtractorPotentiometerSlider* get_potentiometers_slider(int index);
};

template<typename T_extractor>
void HandleExtractors::init_element(int index, std::map<int,T_extractor*>& map){
    if(map.find(index) == map.end()){
        std::pair<int,T_extractor*> new_element = std::make_pair(index,new T_extractor(index));
        map.insert(new_element);
    }
}


