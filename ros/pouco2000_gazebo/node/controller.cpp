// lib ros 
#include <ros/ros.h>

// lib msgs 
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"

// lib cpp 
#include <boost/function.hpp>
#include <string>

#define RATE 10

//TODO: move classes to lib 


/* controller On/Off to float  */

class ControllerOnOffToFloat {
    private: 
        //FIX ME: fix real value, maybe fix it with a rosparam 
        static const float VALUE_MIN;
        static const float VALUE_MAX; 

        float value_min;
        float value_max;

        // ros variables 
        ros::Publisher pub;
        ros::Subscriber sub;
        
        /**
         * @brief send a msg to HeadLightFront 
         * controller in fact the value of the msg received
         * 
         * @param msg msg received  
         */
        void callback(const std_msgs::Bool::ConstPtr& msg){
            std_msgs::Float64 msg_sent;
            if(msg->data){
                msg_sent.data = this->value_max;
            }else {
                msg_sent.data = this->value_min;
            }
            this->pub.publish(msg_sent);
        }

    public:
        ControllerOnOffToFloat(ros::NodeHandle& nh, 
                                const std::string& sub_topic,
                                const std::string& pub_topic,
                                float _value_min,
                                float _value_max):value_min(_value_min),value_max(_value_max){
            this->pub = nh.advertise<std_msgs::Float64>(pub_topic,10);
            this->sub = nh.subscribe<std_msgs::Bool>(sub_topic,1000,&ControllerOnOffToFloat::callback,this);
        }

        ControllerOnOffToFloat(ros::NodeHandle& nh, 
                                const std::string& sub_topic,
                                const std::string& pub_topic):ControllerOnOffToFloat(nh,sub_topic,pub_topic,VALUE_MIN,VALUE_MAX){

        }
};

const float ControllerOnOffToFloat::VALUE_MIN = 0;
const float ControllerOnOffToFloat::VALUE_MAX = 10;



/* controller headlight back */


class ControllerHeadLightBack {
    private: 
        //FIX ME: fix real value, maybe fix it with a rosparam 
        static const float VALUE_MIN;
        static const float VALUE_MAX; 

        float value_min;
        float value_max;

        // ros variables 
        ros::Publisher pub;
        ros::Subscriber sub;

       
        
        /**
         * @brief send a msg to HeadLightFront 
         * controller in fact the value of the msg received
         * 
         * @param msg msg received  
         */
        void callback(const std_msgs::Bool::ConstPtr& msg){
            std_msgs::Float64MultiArray msg_sent;
            msg_sent.layout.dim.push_back(std_msgs::MultiArrayDimension());
            msg_sent.layout.dim[0].size = 2;
            msg_sent.layout.dim[0].stride = 1;
            if(msg->data){
                msg_sent.data.push_back(-1*value_max);
                msg_sent.data.push_back(-1*value_max);
            }else {
                msg_sent.data.push_back(value_min);
                msg_sent.data.push_back(value_min);
            }
            this->pub.publish(msg_sent);
        }

    public:
        ControllerHeadLightBack(ros::NodeHandle& nh, 
                                const std::string& sub_topic,
                                const std::string& pub_topic,
                                float _value_min,
                                float _value_max):value_min(_value_min),value_max(_value_max){
            this->pub = nh.advertise<std_msgs::Float64MultiArray>(pub_topic,10);
            this->sub = nh.subscribe<std_msgs::Bool>(sub_topic,1000,&ControllerHeadLightBack::callback,this);
        }

        ControllerHeadLightBack(ros::NodeHandle& nh, 
                                const std::string& sub_topic,
                                const std::string& pub_topic):ControllerHeadLightBack(nh,sub_topic,pub_topic,VALUE_MIN,VALUE_MAX){

        }
};

const float ControllerHeadLightBack::VALUE_MIN = 0;
const float ControllerHeadLightBack::VALUE_MAX = 10;



/* controller Beacon */

class ControllerBeacon {
    private: 
        //FIX ME: fix real value, maybe fix it with a rosparam 
        static const float TRANS_VALUE_MIN;
        static const float TRANS_VALUE_MAX; 
        static const float ROT_VALUE_MIN;
        static const float ROT_VALUE_MAX; 

        // ros variables 
        ros::Publisher pub_trans;
        ros::Publisher pub_rot;
        ros::Subscriber sub;
        
        /**
         * @brief send a msg to HeadLightFront 
         * controller in fact the value of the msg received
         * 
         * @param msg msg received  
         */
        void callback(const std_msgs::Bool::ConstPtr& msg){
            // generation 
            std_msgs::Float64 msg_trans;
            std_msgs::Float64 msg_rot;
            if(msg->data){
                msg_trans.data = TRANS_VALUE_MAX;
                msg_rot.data = ROT_VALUE_MAX;
            }else {
                msg_trans.data = TRANS_VALUE_MIN;
                msg_rot.data = ROT_VALUE_MIN;
            }
            // publication 
            this->pub_trans.publish(msg_trans);
            this->pub_rot.publish(msg_rot);
        }

    public:
        ControllerBeacon(ros::NodeHandle& nh, 
                                const std::string& sub_topic,
                                const std::string& pub_topic_rot,
                                const std::string& pub_topic_trans){
            this->pub_trans = nh.advertise<std_msgs::Float64>(pub_topic_trans,10);
            this->pub_rot = nh.advertise<std_msgs::Float64>(pub_topic_rot,10);
            this->sub = nh.subscribe<std_msgs::Bool>(sub_topic,1000,&ControllerBeacon::callback,this);
        }
};

const float ControllerBeacon::TRANS_VALUE_MIN = 0;
const float ControllerBeacon::TRANS_VALUE_MAX = 10;
const float ControllerBeacon::ROT_VALUE_MIN = 0;
const float ControllerBeacon::ROT_VALUE_MAX = 10;


/* */



/* controller On/Off to float  */

class ControllerCmdVel {
    private: 
        // ros variables 
        ros::Publisher pub;
        ros::Subscriber sub;
        
        /**
         * @brief send a msg to HeadLightFront 
         * controller in fact the value of the msg received
         * 
         * @param msg msg received  
         */
        void callback(const geometry_msgs::Twist::ConstPtr& msg){
            this->pub.publish(msg);
        }

    public:
        ControllerCmdVel(ros::NodeHandle& nh, 
                                const std::string& sub_topic,
                                const std::string& pub_topic){
            this->pub = nh.advertise<geometry_msgs::Twist>(pub_topic,10);
            this->sub = nh.subscribe<geometry_msgs::Twist>(sub_topic,1000,&ControllerCmdVel::callback,this);
        }
};



int main(int argc, char **argv){
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle nh;

  ControllerOnOffToFloat controller_headlight_front(nh,"controller/headlight_front","/pouco2000Robot_headlights_front_controller/command");
  ControllerHeadLightBack controller_headlight_back(nh,"controller/headlight_back","/pouco2000Robot_headlights_back_controller/command");
  ControllerOnOffToFloat controller_fork(nh,"controller/fork","/pouco2000Robot_forks_controller/command");
  ControllerBeacon controller_beacon(nh,"controller/beacon","/pouco2000Robot_beacon_rot_controller/command","/pouco2000Robot_beacon_trans_controller/command");
  ControllerCmdVel controller_cmdvel(nh,"controller/cmdvel","/pouco2000Robot_diff_drive_controller/cmd_vel");
  

  ros::spin();

  return 0;
}


