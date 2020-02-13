#include <pouco2000_ros/pouco2000_debug.hpp>

void rnd::init(){
    srand(time(0)); 
}

template<>
int rnd::gen(int begin, int end){
    return rand()%((end - begin) + 1) + begin;
}

template<>
float rnd::gen(int begin, int end) {
    float random = ((float) rand()) / (float) RAND_MAX;
    float diff = end - begin;
    float r = random * diff;
    return begin + r;
}

int load_param_rate(const std::string key,const int& default_value){
    int rate;
    if(ros::param::get(key,rate)){
        return rate;
    }
    return default_value;
}