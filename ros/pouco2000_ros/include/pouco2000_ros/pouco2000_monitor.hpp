#pragma once 
//cpp
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>
//ros 
#include <ros/ros.h>
//msgs 
#include <pouco2000_ros/Controller.h>

#define NB_COLS_SWITCHS 5
#define NB_COLS_POTS 2 

namespace color {
    enum FG {
        BLACK = 30,
        RED,             
        GREEN,           
        YELLOW,          
        BLUE,            
        MAGENTA,         
        CYAN,            
        WHITE 
    };
    /**
     * @brief Generate a string of the content in applying a font color 
     * 
     * @param content 
     * @param color 
     * @return std::string 
     */
    std::string write(std::string content, color::FG color);
};

namespace screen {
    /**
     * @brief Print the current value 
     * 
     * @param value 
     */
    void print(std::string value);

    /**
     * @brief Clear the terminal 
     * 
     */
    void clear();
}


namespace math {
    std::string round_digits(float value, int digits);
}

/**
 * @brief Mother class defining the abstract method draw.
 * 
 */
class View {
    public: 
        View();
        /**
         * @brief Abstract method defined by each child 
         * 
         * @return std::string draw generate by the current view 
         */
        virtual std::string draw()=0;
};

/**
 * @brief Draw a label 
 * 
 */
class LabelView: public View {
    protected: 
        std::string label;
    public: 
        LabelView(std::string label);
        void set_label(std::string label);
        std::string draw();
};

/**
 * @brief Draw a title 
 * 
 */
class TitleLabelView: public LabelView{
    private:
        static const int WIDTH_BEGIN = 2;
        static const int NB_NEW_LINES = 1;
        static const char CHAR_LINE = '+';

        char char_line;
        int nb_new_lines;
        int width;
    public: 
        TitleLabelView(std::string label, int width);
        TitleLabelView(std::string label, int width,char char_line);
        std::string draw();
};

/**
 * @brief Draw a progress bar 
 * 
 */
class ProgressBarView: public View {
    private: 
        static const float SCALE;

        static const char CHAR_HIGH_STICK = '|';
        static const char CHAR_LOW_STICK = '_';

        static const color::FG COLOR_0P = color::FG::RED;
        static const color::FG COLOR_25P = color::FG::BLUE;
        static const color::FG COLOR_50P = color::FG::YELLOW;
        static const color::FG COLOR_75P = color::FG::GREEN;
         
        int min;
        int max;
        float value;

        color::FG compute_color(float percent_value); 

    public: 
        ProgressBarView();
        ProgressBarView(int min, int max);
        void set_value(float value);
        std::string draw();
};

/**
 * @brief Draw a switch 
 * 
 * @tparam T 
 */
template<typename T>
class SwitchView: public View {
    private: 
        static const color::FG COLOR_SELECTED = color::FG::GREEN;
        static const color::FG COLOR_UNSELECTED = color::FG::RED;
        std::string valA;
        std::string valB;
        
        T value;
        int define_val_selected();
        
    public: 
        SwitchView(std::string valA, std::string valB);
        void set_value(T value);

        std::string draw();
};  

template<typename T>
SwitchView<T>::SwitchView(std::string valA, std::string valB):View(){
    this-> valA = valA;
    this-> valB = valB;
};

template<typename T>
void SwitchView<T>::set_value(T value){
    this-> value = value;
}

template<typename T>
int SwitchView<T>::define_val_selected(){
    return -1;
}

template<>
int SwitchView<bool>::define_val_selected();

template<>
int SwitchView<int>::define_val_selected();


template<typename T>
std::string SwitchView<T>::draw(){
    int value_selected = define_val_selected();
    std::string result_a;
    std::string result_b;
    if(value_selected == 0){
        result_a = color::write(valA,SwitchView::COLOR_SELECTED);
        result_b = color::write(std::string(valB.length(),'-'),SwitchView::COLOR_UNSELECTED);
    }else{
        result_a = color::write(std::string(valA.length(),'-'),SwitchView::COLOR_SELECTED);
        result_b = color::write(valB,SwitchView::COLOR_UNSELECTED);
    }
    std::string result = "["+ result_a + "|" + result_b + "]";
    return result;
};

/**
 * @brief Manages views
 * 
 */
class Container: public View {
    protected:
        std::vector<View*> views;
    public:
        Container();
        void add_view(View *v);
        virtual std::string draw()=0; 
};

/**
 * @brief Manages vertically views 
 * 
 */
class ContainerVertical: public Container {
    public:
        ContainerVertical();
        std::string draw(); 
};

/**
 * @brief Manages Horizontally views 
 * 
 */
class ContainerHorizontal: public Container {
    public:
        ContainerHorizontal();
        std::string draw(); 
};

/**
 * @brief Manages views in N columns  
 * 
 */
class ContainerNColumns: public Container {
    private: 
        int cols;
    public:
        ContainerNColumns(int cols);
        std::string draw(); 
};


/**
 * @brief Handles reception of controller msg, creates views and updates views.   
 * 
 */
class Monitor: public ContainerVertical {
    private: 
        int width_cols;

        ros::Subscriber sub;

        LabelView* view_seq_value;

        std::vector<SwitchView<bool>*> views_buttons;
        std::vector<SwitchView<bool>*> views_switchs_onoff;
        std::vector<SwitchView<int>*> views_switchs_mode;
        std::vector<ProgressBarView*> views_potentiometers_slider;
        std::vector<ProgressBarView*> views_potentiometers_circle;


        ContainerVertical* create_part(std::string title, Container* cv);
           
        // init and update method 
        // each field = 1 init and 1 update methods 

        void init_buttons(const pouco2000_ros::Controller::ConstPtr& msg);
        void update_buttons(const pouco2000_ros::Controller::ConstPtr& msg);

        void init_potentiometers_slider(const pouco2000_ros::Controller::ConstPtr& msg);
        void update_potentiometers_slider(const pouco2000_ros::Controller::ConstPtr& msg);

        void init_switchs_modes(const pouco2000_ros::Controller::ConstPtr& msg);
        void update_switchs_modes(const pouco2000_ros::Controller::ConstPtr& msg);

        void init_switchs_onoff(const pouco2000_ros::Controller::ConstPtr& msg);
        void update_switchs_onoff(const pouco2000_ros::Controller::ConstPtr& msg);

        void init_potentiometers_circle(const pouco2000_ros::Controller::ConstPtr& msg);
        void update_potentiometers_circle(const pouco2000_ros::Controller::ConstPtr& msg);

        void callback(const pouco2000_ros::Controller::ConstPtr& msg);


    public: 
        Monitor(ros::NodeHandle& nh, std::string topic, int width_cols, std::string title);
   
        
};

