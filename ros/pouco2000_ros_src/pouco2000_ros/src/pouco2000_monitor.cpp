#include <pouco2000_ros/pouco2000_monitor.hpp>

/* color */

std::string color::write(const std::string& content,const color::FG color){
    std::string result = "\033[";
    result += std::to_string(color);
    result += "m";
    result += content;
    result += "\033[0m";
    return result;
}

/* screen */ 

void screen::print(const std::string& value){
    std::cout << value << std::endl;
}

void screen::clear(){
    std::cout << "\x1B[2J\x1B[H";
}

/* math */

std::string math::round_digits(const float& value,const int& digits){
    std::stringstream stream;
    stream << std::fixed << std::setprecision(digits) << value;
    return stream.str();
}

/* String */

ColorString::ColorString(){
    length = 0;
    content = "";
}

ColorString::ColorString(std::string _content):ColorString(){
    add(_content);
}

ColorString::ColorString(std::string _content,const color::FG color):ColorString(){
    add(_content,color);
}

std::string ColorString::getContent(){
    return this->content; 
}

int ColorString::getLength(){
    return length;
}

void ColorString::add(ColorString other_string){
    length += other_string.getLength();
    this->content += other_string.getContent();
}

void ColorString::add(std::string _content){
    length += _content.length();
    this->content += _content;
}

void ColorString::add(std::string _content, const color::FG color){
    length += _content.length();
    this->content += color::write(_content,color);
}
 
/* View */

View::View(){

}

/* LabelView */

LabelView::LabelView(std::string label):View(){
    this->label = label;
}

void LabelView::set_label(std::string label){
    this->label = label;
}

ColorString LabelView::draw(){
    return ColorString(this->label);
}

/* TitleLabelView */

TitleLabelView::TitleLabelView(std::string label, unsigned short& width, 
                                char char_line):width(width),
LabelView(label){
    this->width = width;
    this->char_line = char_line;
}

TitleLabelView::TitleLabelView(std::string label, unsigned short& width):
TitleLabelView(label,width,TitleLabelView::CHAR_LINE){

}

ColorString TitleLabelView::draw(){
    ColorString result("\n"); 
    result.add(std::string(WIDTH_BEGIN,char_line));
    result.add(" " + label + " ");
    result.add(std::string(width - (2 + (label.length() + WIDTH_BEGIN)), char_line));
    result.add("\n");
    return result;
}

/* ProgressBarView */

const float ProgressBarView::SCALE = 0.25f;

ProgressBarView::ProgressBarView(int min, int max):View(){
    this->min = min;
    this->max = max;
}

ProgressBarView::ProgressBarView():ProgressBarView(0,100){

}

void ProgressBarView::set_value(float value){
    this->value = value;
}

color::FG ProgressBarView::compute_color(float percent_value){
    color::FG color_fg; 
    if(percent_value >= 75){
        color_fg = ProgressBarView::COLOR_75P;
    }
    else if(percent_value >= 50){
        color_fg = ProgressBarView::COLOR_50P;
    }
    else if(percent_value >= 25){
        color_fg = ProgressBarView::COLOR_25P;
    }
    else {
        color_fg = ProgressBarView::COLOR_0P;
    }
    return color_fg;
}

ColorString ProgressBarView::draw(){
    float percent = (this->value-min)/(max-min) * 100;
    int high_sticks = percent * ProgressBarView::SCALE;
    int low_sticks = (100 - percent) * ProgressBarView::SCALE;
    color::FG color = compute_color(percent);
    ColorString result("[");
    std::string high_sticks_str = std::string(high_sticks,ProgressBarView::CHAR_HIGH_STICK);
    std::string low_sticks_str = std::string(low_sticks,ProgressBarView::CHAR_LOW_STICK);
    std::string value_str = math::round_digits(percent,2); 
    result.add(high_sticks_str + low_sticks_str + value_str,color);
    result.add("]");
    return result;
}

/* Switch<T> */

template<>
int SwitchView<bool>::define_val_selected(){
    if(this->value){
        return 0;
    }
    return 1;
}

template<>
int SwitchView<int>::define_val_selected(){
    if(this->value == 0){
        return 0;
    }
    return 1;
}

/* Container */

Container::Container():View(){

};

void Container::add_view(View *v){
    views.push_back(v);
}

/* ContainerVertical */

ContainerVertical::ContainerVertical():Container(){

}

ColorString ContainerVertical::draw(){
    ColorString result;
    std::vector<View*>::iterator it;
    int i = 0;
    for(it = views.begin(); it != views.end(); ++it){
        result.add((**it).draw());
        if(i++ < views.size() - 1){
            result.add("\n");
        }
    }
    return result;
}

/* ContainerHorizontal */

ContainerHorizontal::ContainerHorizontal():Container(){

}

ColorString ContainerHorizontal::draw(){
    ColorString result;
    std::vector<View*>::iterator it;
    int i = 0;
    for(it = views.begin(); it != views.end(); ++it){
        result.add((**it).draw());
        if(i++ < views.size() - 1){
            result.add(" ");
        }
    }
    return result;
}

/* ContainerNColumns */
 
const std::string ContainerNColumns::LINE_SEPARATOR = " | ";

ContainerNColumns::ContainerNColumns(unsigned short& cols_or_width, bool is_auto=false):width(cols_or_width),Container(){
    this->is_auto = is_auto;
    this->cols = cols_or_width;
}

ColorString ContainerNColumns::draw(){
    std::vector<ColorString> views_str;
    std::vector<View*>::iterator it;
    int size_max = -1;
    ColorString local_view_str;
    for(it = views.begin(); it != views.end(); ++it){
        local_view_str = (**it).draw();
        views_str.push_back(local_view_str);
        if(size_max == -1 or local_view_str.getLength() > size_max){
            size_max = local_view_str.getLength();
        }
    }

    int nb_cols;
    if(is_auto){
        nb_cols = this->width / (size_max + LINE_SEPARATOR.length());
    }else{
        nb_cols = this->cols;
    }

    ColorString result;
    std::vector<std::string>::iterator it_str;
    int n_lines = views_str.size()/nb_cols + (views_str.size()%nb_cols ? 0 : 0 , 1) ;
  
    for(int i = 0; i<n_lines; i++){
        for (int j=0;j < nb_cols; j++){
            int index = i*nb_cols + j;
            if(index < views_str.size()){
                ColorString local = views_str.at(index);
                result.add(local);
                if(j < nb_cols - 1){
                    result.add(std::string(this->width/nb_cols-(local.getLength()+LINE_SEPARATOR.length()),' '));
                    result.add(LINE_SEPARATOR);
                }
            }
        }
        if(i < n_lines -1){
            result.add("\n");
        }
    }
    return result;
}


bool operator!= (pouco2000_ros_msgs::Controller::ConstPtr const &c1, 
                pouco2000_ros_msgs::Controller::ConstPtr const& c2){
    
    if(c1->buttons.data != c2->buttons.data){
        return true;
    }
    if(c1->switchs_mode.data != c2->switchs_mode.data){
        return true;
    }
    if(c1->switchs_on_off.data != c2->switchs_on_off.data){
        return true;
    }
    if(c1->potentiometers_circle.data != c2->potentiometers_circle.data){
        return true;
    }
    if(c1->potentiometers_slider.data != c2->potentiometers_slider.data){
        return true;
    }

    return false;
}
 
/* Monitor */
 
Monitor::Monitor(ros::NodeHandle& nh, std::string topic, unsigned short& width_cols, std::string title,const bool& auto_refresh)
:ContainerVertical(),width_cols(width_cols),auto_refresh(auto_refresh){  
    this->sub = nh.subscribe<pouco2000_ros_msgs::Controller>(topic, 1000, &Monitor::callback, this);

    TitleLabelView* v_title = new TitleLabelView(title,width_cols,'#');
    this->add_view(v_title); 

    ContainerHorizontal* ch_seq = new ContainerHorizontal();
    LabelView* view_seq_text = new LabelView("seq:");
    view_seq_value = new LabelView("");
    ch_seq->add_view(view_seq_text);
    ch_seq->add_view(view_seq_value);

    pouco2000_ros_msgs::Controller msg_controller;
    msg_previous = *(new pouco2000_ros_msgs::Controller::ConstPtr(new pouco2000_ros_msgs::Controller(msg_controller))); 
    this->add_view(ch_seq);
} 

ContainerVertical* Monitor::create_part(std::string title, Container* cv){
    ContainerVertical* cv_global = new ContainerVertical();
    TitleLabelView* v_title = new TitleLabelView(title,width_cols); 
    (*cv_global).add_view(v_title);
    (*cv_global).add_view(cv);
    return cv_global;
}

void Monitor::init_buttons(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    int number_buttons = msg->buttons.data.size();
    ContainerNColumns* cv = new ContainerNColumns(this->width_cols, true);
    for (int i=0;i < number_buttons; i++){
        ContainerHorizontal* ch = new ContainerHorizontal();
        LabelView* l = new LabelView("BUT"+std::to_string(i));
        SwitchView<bool>* s = new SwitchView<bool>("ON","OFF");
        views_buttons.push_back(s);
        (*ch).add_view(l);
        (*ch).add_view(s);
        (*cv).add_view(ch);
    }
    this->add_view(create_part("Buttons",cv));
}

void Monitor::update_buttons(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    int size = msg->buttons.data.size();
    for(int i=0;i < size;i++){
        bool value = msg->buttons.data.at(i);
        (*views_buttons.at(i)).set_value(value);
    }
} 


void Monitor::init_potentiometers_slider(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    int number_buttons = msg->potentiometers_slider.data.size();
    ContainerNColumns* cv = new ContainerNColumns(this->width_cols, true);
    for (int i=0;i < number_buttons; i++){
        ContainerHorizontal* ch = new ContainerHorizontal();
        LabelView* l = new LabelView("POS"+std::to_string(i));
        ProgressBarView* s = new ProgressBarView();
        views_potentiometers_slider.push_back(s);
        (*ch).add_view(l);
        (*ch).add_view(s);
        (*cv).add_view(ch);
    }
    this->add_view(create_part("Pots Slider",cv));
}


void Monitor::update_potentiometers_slider(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    int size = msg->potentiometers_slider.data.size();
    for(int i=0;i < size;i++){
        float value = msg->potentiometers_slider.data.at(i);
        (*views_potentiometers_slider.at(i)).set_value(value);
    }
} 

void Monitor::init_potentiometers_circle(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    int number_buttons = msg->potentiometers_circle.data.size();
    ContainerNColumns* cv = new ContainerNColumns(this->width_cols, true);
    for (int i=0;i < number_buttons; i++){
        ContainerHorizontal* ch = new ContainerHorizontal();
        LabelView* l = new LabelView("POC"+std::to_string(i));
        ProgressBarView* s = new ProgressBarView();
        views_potentiometers_circle.push_back(s);
        (*ch).add_view(l);
        (*ch).add_view(s);
        (*cv).add_view(ch);
    }
    this->add_view(create_part("Pots Circle",cv));
}

void Monitor::update_potentiometers_circle(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    int size = msg->potentiometers_circle.data.size();
    for(int i=0;i < size;i++){
        float value = msg->potentiometers_circle.data.at(i);
        (*views_potentiometers_circle.at(i)).set_value(value);
    }
} 


void Monitor::init_switchs_onoff(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    int number_buttons = msg->switchs_on_off.data.size();
    ContainerNColumns* cv = new ContainerNColumns(this->width_cols, true);
    for (int i=0;i < number_buttons; i++){
        ContainerHorizontal* ch = new ContainerHorizontal();
        LabelView* l = new LabelView("SOF"+std::to_string(i));
        SwitchView<bool>* s = new SwitchView<bool>("ON","OFF");
        views_switchs_onoff.push_back(s);
        (*ch).add_view(l);
        (*ch).add_view(s);
        (*cv).add_view(ch);
    }
    this->add_view(create_part("Switchs On Off",cv));
}

void Monitor::update_switchs_onoff(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    int size = msg->switchs_on_off.data.size();
    for(int i=0;i < size;i++){
        bool value = msg->switchs_on_off.data.at(i);
        (*views_switchs_onoff.at(i)).set_value(value);
    }
} 

void Monitor::init_switchs_modes(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    int number_buttons = msg->switchs_mode.data.size();
    ContainerNColumns* cv = new ContainerNColumns(this->width_cols, true);
    for (int i=0;i < number_buttons; i++){
        ContainerHorizontal* ch = new ContainerHorizontal();
        LabelView* l = new LabelView("SMO"+std::to_string(i));
        SwitchView<int>* s = new SwitchView<int>("M1","M2");
        views_switchs_mode.push_back(s);
        (*ch).add_view(l);
        (*ch).add_view(s);
        (*cv).add_view(ch);
    }
    this->add_view(create_part("Switchs Modes",cv));
}

void Monitor::update_switchs_modes(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    int size = msg->switchs_mode.data.size();
    for(int i=0;i < size;i++){
        bool value = msg->switchs_mode.data.at(i);
        (*views_switchs_mode.at(i)).set_value(value);
    }
} 

void Monitor::callback(const pouco2000_ros_msgs::Controller::ConstPtr& msg){
    
    if(!msg->buttons.data.empty() && this->views_buttons.empty()){
        init_buttons(msg);
    }
    if(!msg->switchs_mode.data.empty() &&this->views_switchs_mode.empty()){
        init_switchs_modes(msg);
    }
    if(!msg->switchs_on_off.data.empty() &&this->views_switchs_onoff.empty()){
        init_switchs_onoff(msg);
    }
    if(!msg->potentiometers_circle.data.empty() &&this->views_potentiometers_circle.empty()){
        init_potentiometers_circle(msg);
    }
    if(!msg->potentiometers_slider.data.empty() &&this->views_potentiometers_slider.empty()){
        init_potentiometers_slider(msg);
    }

    view_seq_value->set_label(std::to_string(msg->header.seq));
    
    update_buttons(msg);
    update_switchs_modes(msg);
    update_switchs_onoff(msg);
    update_potentiometers_slider(msg);
    update_potentiometers_circle(msg);

    if(auto_refresh || msg != msg_previous){
        screen::clear();
        screen::print(draw().getContent());
    }
    msg_previous = msg;
}