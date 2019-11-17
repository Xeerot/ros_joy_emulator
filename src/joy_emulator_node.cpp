#include <cmath>
#include <linux/input.h>
#include <map>
#include <memory>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "joystick.h"
using std::placeholders::_1;

namespace JoyEmulator
{
class JoyEmulatorNode : public rclcpp::Node
{
public:
    JoyEmulatorNode()
        : Node("joy_emulator")
    {
        // TODO: Joystick factory/params
        joystick_ptr = std::make_shared<JoyEmulator::Xbox360JoyStick>();
        joystick_ptr->createDevice();
        // TODO: Need to add ros params and launch file. Topic should be param
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoyEmulatorNode::joy_callback, this, _1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) const
    {
        joystick_ptr->publish(msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    // TODO: Convert to abstract
    std::shared_ptr<JoyEmulator::Xbox360JoyStick> joystick_ptr;
};
} // namespace JoyEmulator

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyEmulator::JoyEmulatorNode>());
    rclcpp::shutdown();
    return 0;
}