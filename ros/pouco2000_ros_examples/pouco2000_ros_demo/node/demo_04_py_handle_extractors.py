#!/usr/bin/python3.6
import rospy 
from pouco2000_ros_msgs.msg import Controller
from pouco2000_ros_tools.pouco2000_extractor_py import HandleExtractors  


class ExampleExtractor: 

    def __init__(self,topic_name):
        rospy.Subscriber(topic_name, Controller, self.callback, queue_size=10)
        self.he = HandleExtractors()

    def callback(self,data):
        if(self.he.get_button(0).is_push(data)):
            rospy.loginfo("button 0 is push")

        if(self.he.get_switch_onoff(0).is_on(data)):
            rospy.loginfo("switch_onoff 0 is on")

        if(self.he.get_switch_modes(0).is_mode(data,0)):
            rospy.loginfo("switch_modes 0 is on mode 0")

        if(self.he.get_potentiometer_circle(0).extract_only_change(data)):
            rospy.loginfo("potentiometer_circle 0 value: %s"%self.he.get_potentiometer_circle(0).get_data())

        if(self.he.get_potentiometer_slider(0).extract_only_change(data)):
            rospy.loginfo("potentiometer_slider 0 value: %s"%self.he.get_potentiometer_slider(0).get_data())
       
if __name__ == '__main__':
    try:
        rospy.init_node('demo_04_py_handle_extractors_node', anonymous=True)
        e = ExampleExtractor("controller")
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
    