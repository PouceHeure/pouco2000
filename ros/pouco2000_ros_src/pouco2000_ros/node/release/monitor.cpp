//lib allowing to load current console size 
#include <sys/ioctl.h>
//lib pouco2000_monitor
#include <pouco2000_ros/pouco2000_monitor.hpp>

#define KEY_PARAM_RATE "~rate"
#define KEY_PARAM_AUTO_REFRESH "~auto_refresh"


int main(int argc, char **argv){
    ros::init(argc, argv, "monitor_node");
    ros::NodeHandle n;

    struct winsize size;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);

    int param_value_rate = 10;
    ros::param::get(KEY_PARAM_RATE,param_value_rate);
    ros::Rate r(param_value_rate);

    bool param_value_auto_refresh = true;
    ros::param::get(KEY_PARAM_RATE,param_value_auto_refresh);

    Monitor m(n,"controller",size.ws_col,"Pouco2000 Monitor",param_value_auto_refresh);
    
    while(ros::ok()){
        ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}