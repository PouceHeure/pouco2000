#include "pouco2000_ros_tools/pouco2000_extractor.hpp"
#include "pouco2000_ros_msgs/Controller.h"
// ros 
#include <ros/ros.h>
#include <gtest/gtest.h>


/**
 * @brief Test ExtractorButton  
 */
TEST(TestPouco2000ExtractorLib, test_button)
{   
    pouco2000_ros_msgs::Controller msg_content;
    ExtractorButton extractor(0);  
    //test: data is missing 
    pouco2000_ros_msgs::Controller::ConstPtr msg_empty(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_FALSE(extractor.is_push(msg_empty));
    //test: data added (false)
    msg_content.buttons.data.push_back(false);
    pouco2000_ros_msgs::Controller::ConstPtr msg_fill_false(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_FALSE(extractor.is_push(msg_fill_false));
    //test: data updated (true)
    msg_content.buttons.data.at(0) = true;
    pouco2000_ros_msgs::Controller::ConstPtr msg_fill_true(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_TRUE(extractor.is_push(msg_fill_true));
    //test: data not updated 
    EXPECT_FALSE(extractor.is_push(msg_fill_true));
}

/**
 * @brief Test ExtractorSwitchOnOff  
 */
TEST(TestPouco2000ExtractorLib, test_switch_onoff)
{
    pouco2000_ros_msgs::Controller msg_content;
    ExtractorSwitchOnOff extractor(0);  
    //test: data is missing 
    pouco2000_ros_msgs::Controller::ConstPtr msg_empty(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_FALSE(extractor.is_on(msg_empty));
    //test: data added (false)
    msg_content.switchs_on_off.data.push_back(false);
    pouco2000_ros_msgs::Controller::ConstPtr msg_fill_false(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_FALSE(extractor.is_on(msg_fill_false));
    //test: data updated (true)
    msg_content.switchs_on_off.data.at(0) = true;
    pouco2000_ros_msgs::Controller::ConstPtr msg_fill_true(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_TRUE(extractor.is_on(msg_fill_true));
    //test: data not updated 
    EXPECT_FALSE(extractor.is_on(msg_fill_true));
}


/**
 * @brief Test ExtractorSwitchMode  
 */
TEST(TestPouco2000ExtractorLib, test_switch_mode)
{
    pouco2000_ros_msgs::Controller msg_content;
    ExtractorSwitchMode extractor(0);  
    //test: data is missing 
    pouco2000_ros_msgs::Controller::ConstPtr msg_empty(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_FALSE(extractor.is_mode(msg_empty,12));
    //test: data added (11)
    msg_content.switchs_mode.data.push_back(11);
    pouco2000_ros_msgs::Controller::ConstPtr msg_fill_11(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_FALSE(extractor.is_mode(msg_fill_11,12));
    //test: data updated (12)
    msg_content.switchs_mode.data.at(0) = 12;
    pouco2000_ros_msgs::Controller::ConstPtr msg_fill_12(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_TRUE(extractor.is_mode(msg_fill_12,12));  
    //test: data not updated 
    EXPECT_FALSE(extractor.is_mode(msg_fill_12,12));
}

/**
 * @brief Test ExtractorPotentiometer (circle and slider) 
 */
TEST(TestPouco2000ExtractorLib, test_potentiometer)
{
    pouco2000_ros_msgs::Controller msg_content;
    ExtractorPotentiometerCircle extractor_poc(0);
    ExtractorPotentiometerSlider extractor_pos(0);  
    float value_poc;
    float value_pos;
    //test: data is missing 
    pouco2000_ros_msgs::Controller::ConstPtr msg_empty(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_FALSE(extractor_poc.extract_only_change(msg_empty,value_poc));
    EXPECT_FALSE(extractor_pos.extract_only_change(msg_empty,value_pos));
    //test: data added (15)
    msg_content.potentiometers_circle.data.push_back(15);
    msg_content.potentiometers_slider.data.push_back(15);
    pouco2000_ros_msgs::Controller::ConstPtr msg_fill_15(new pouco2000_ros_msgs::Controller(msg_content));
    EXPECT_TRUE(extractor_poc.extract_only_change(msg_fill_15,value_poc));
    EXPECT_TRUE(value_poc == 15);
    EXPECT_TRUE(extractor_pos.extract_only_change(msg_fill_15,value_pos));
    EXPECT_TRUE(value_pos == 15);
    //test: data not updated (extract_only_change)
    EXPECT_FALSE(extractor_poc.extract_only_change(msg_fill_15,value_poc));
    EXPECT_FALSE(extractor_pos.extract_only_change(msg_fill_15,value_pos));
    //test: data not updated (extract)
    EXPECT_TRUE(extractor_poc.extract(msg_fill_15,value_poc));
    EXPECT_TRUE(extractor_pos.extract(msg_fill_15,value_pos));
}


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "utester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}