#include "pouco2000_ros/pouco2000.hpp"
#include "pouco2000_ros/pouco2000_debug.hpp"
#include <gtest/gtest.h>

/**
 * @brief Test percent convertion 
 */
TEST(TestPouco2000Lib, testConvertisser)
{
    ConvertisserPotentiometerToPercent c(2,10);
    EXPECT_EQ(0,c.convert(2));
    EXPECT_EQ(100,c.convert(10));
    EXPECT_EQ(50,c.convert(6));
}

/**
 * @brief Test percent convertion on a msg 
 */
TEST(TestPouco2000Lib, testConvertisserMsg)
{
    pouco2000_ros_msgs::Potentiometers msg_pots;
    msg_pots.data.push_back(2);
    msg_pots.data.push_back(10);
    msg_pots.data.push_back(6);
    pouco2000_ros_msgs::Potentiometers msg_expected;
    msg_expected.data.push_back(0);
    msg_expected.data.push_back(100);
    msg_expected.data.push_back(50);
    ConvertisserPotentiometerToPercent c(2,10);
    pouco2000_ros_msgs::Potentiometers msg_computed = convert_potentiometers(c,msg_pots);
    EXPECT_EQ(msg_expected.data,msg_computed.data);
}

/**
 * @brief Test callback wrapper work properly 
 * in checking if the current msg of the controller is updated
 */
TEST(TestPouco2000Lib, testWrapper)
{ 
    Controller c;
    c.set_send_mode(SendMode::freq);
    boost::function<void(const pouco2000_ros_msgs::Buttons::ConstPtr&,pouco2000_ros_msgs::Controller& current_msg)> buttons_callback;
    buttons_callback = callback_field::buttons;
    auto wrapper_callback = c.wrapper(buttons_callback);
    pouco2000_ros_msgs::Buttons fake_msg;
    fake_msg.data.push_back(true);
    fake_msg.data.push_back(false);
    pouco2000_ros_msgs::ButtonsConstPtr fake_msg_ptr(new pouco2000_ros_msgs::Buttons(fake_msg));
    wrapper_callback(fake_msg_ptr);
    EXPECT_EQ(fake_msg.data,c.current_msg.buttons.data);
}

/**
 * @brief Test random int generator 
 */
TEST(TestPouco2000DebugLib, testRandomInt)
{
    rnd::init();
    int value_a = rnd::gen<int>(12,17);
    EXPECT_TRUE(value_a >= 12 && value_a <= 17);
    value_a = rnd::gen<int>(0,12);
    EXPECT_TRUE(value_a >= 0 && value_a <= 12);
    value_a = rnd::gen<int>(200,204);
    EXPECT_TRUE(value_a >= 200 && value_a <= 204);
}

/**
 * @brief Test random float generator  
 */
TEST(TestPouco2000DebugLib, testRandomFloat)
{
    rnd::init();
    float value_a = rnd::gen<float>(12,17);
    EXPECT_TRUE(value_a >= 12 && value_a <= 17);
    value_a = rnd::gen<float>(0,12);
    EXPECT_TRUE(value_a >= 0 && value_a <= 12);
    value_a = rnd::gen<float>(200,204);
    EXPECT_TRUE(value_a >= 200 && value_a <= 204);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "utester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}