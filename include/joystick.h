#ifndef JOY_EMULATOR_H
#define JOY_EMULATOR_H

#include "sensor_msgs/msg/joy.hpp"
#include <libevdev.h>
#include <libevdev-uinput.h>

namespace JoyEmulator
{
//TODO: Convert to abstract
class Xbox360JoyStick
{
public:
    void createDevice();
    void publish(const sensor_msgs::msg::Joy::SharedPtr msg);

private:
    struct libevdev_uinput *uinput_dev;
    struct libevdev *dev;

    std::map<int, unsigned int> button_index_mapping = {{0, BTN_SOUTH}, {1, BTN_EAST}, {2, BTN_NORTH}, {3, BTN_WEST}, {4, BTN_TL}, {5, BTN_TR}, {6, BTN_SELECT}, {7, BTN_START}, {8, BTN_MODE}, {9, BTN_THUMBL}, {10, BTN_THUMBR}};
    std::map<int, unsigned int> axes_index_mapping = {{0, ABS_X}, {1, ABS_Y}, {2, ABS_Z}, {3, ABS_RX}, {4, ABS_RY}, {5, ABS_RZ}, {6, ABS_HAT0X}, {7, ABS_HAT0Y}};
    std::map<int, std::pair<int, int>> axes_range_mapping = {{0, {-32768, 32768}}, {1, {-32768, 32768}}, {2, {0, 255}}, {3, {-32768, 32768}}, {4, {-32768, 32768}}, {5, {0, 255}}, {6, {-1, 1}}, {7, {-1, 1}}};

    int current_buttons[11] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    int current_axes[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

} // namespace JoyEmulator

#endif