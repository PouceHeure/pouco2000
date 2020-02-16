//lib allowing to load current console size 
#include <sys/ioctl.h>
//lib pouco2000_monitor
#include <pouco2000_ros/pouco2000_monitor.hpp>


int main(int argc, char **argv){
    ros::init(argc, argv, "monitor_node");
    ros::NodeHandle n;

    struct winsize size;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);

    Monitor m(n,"controller",size.ws_col,"Pouco2000 Monitor");
    ros::spin();

    return 0;
}