#include <cmath>
#include <linux/input.h>
#include <sstream>
#include <map>
#include "joystick.h"
namespace JoyEmulator
{
void Xbox360JoyStick::createDevice()
{
    dev = libevdev_new();
    libevdev_set_name(dev, "Emulated Xbox360 Controller");

    // Support syncronizing events
    libevdev_enable_event_type(dev, EV_SYN);
    libevdev_enable_event_code(dev, EV_SYN, SYN_REPORT, NULL);
    libevdev_enable_event_code(dev, EV_SYN, SYN_CONFIG, NULL);
    libevdev_enable_event_code(dev, EV_SYN, SYN_MT_REPORT, NULL);
    libevdev_enable_event_code(dev, EV_SYN, SYN_DROPPED, NULL);
    libevdev_enable_event_code(dev, EV_SYN, SYN_MAX, NULL);

    // Joystick buttons
    libevdev_enable_event_type(dev, EV_KEY);
    libevdev_enable_event_code(dev, EV_KEY, BTN_SOUTH, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_EAST, NULL);
    // North and West are not as labelled. North = X and West = Y
    libevdev_enable_event_code(dev, EV_KEY, BTN_NORTH, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_WEST, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_TL, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_TR, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_SELECT, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_START, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_MODE, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_THUMBL, NULL);
    libevdev_enable_event_code(dev, EV_KEY, BTN_THUMBR, NULL);

    // Joystick analog sticks (Right and Left triggers are analog)
    libevdev_enable_event_type(dev, EV_ABS);
    auto axis_data_0 = input_absinfo();
    axis_data_0.value = 0;
    axis_data_0.minimum = -32768;
    axis_data_0.maximum = 32767;
    axis_data_0.fuzz = 16;
    axis_data_0.flat = 128;
    axis_data_0.resolution = 0;
    libevdev_enable_event_code(dev, EV_ABS, ABS_X, &axis_data_0);

    // Left X/Y and Left Trigger (Z)
    auto axis_data_1 = input_absinfo();
    axis_data_1.value = 0;
    axis_data_1.minimum = -32768;
    axis_data_1.maximum = 32767;
    axis_data_1.fuzz = 16;
    axis_data_1.flat = 128;
    axis_data_1.resolution = 0;
    libevdev_enable_event_code(dev, EV_ABS, ABS_Y, &axis_data_1);

    auto axis_data_2 = input_absinfo();
    axis_data_2.value = 0;
    axis_data_2.minimum = 0;
    axis_data_2.maximum = 255;
    axis_data_2.fuzz = 0;
    axis_data_2.flat = 0;
    axis_data_2.resolution = 0;
    libevdev_enable_event_code(dev, EV_ABS, ABS_Z, &axis_data_2);

    // Right X/Y and Right Trigger (Z)
    auto axis_data_3 = input_absinfo();
    axis_data_3.value = 0;
    axis_data_3.minimum = -32768;
    axis_data_3.maximum = 32767;
    axis_data_3.fuzz = 16;
    axis_data_3.flat = 128;
    axis_data_3.resolution = 0;
    libevdev_enable_event_code(dev, EV_ABS, ABS_RX, &axis_data_3);

    auto axis_data_4 = input_absinfo();
    axis_data_4.value = 0;
    axis_data_4.minimum = -32768;
    axis_data_4.maximum = 32767;
    axis_data_4.fuzz = 16;
    axis_data_4.flat = 128;
    axis_data_4.resolution = 0;
    libevdev_enable_event_code(dev, EV_ABS, ABS_RY, &axis_data_4);

    auto axis_data_5 = input_absinfo();
    axis_data_5.value = 0;
    axis_data_5.minimum = 0;
    axis_data_5.maximum = 255;
    axis_data_5.fuzz = 0;
    axis_data_5.flat = 0;
    axis_data_5.resolution = 0;
    libevdev_enable_event_code(dev, EV_ABS, ABS_RZ, &axis_data_5);

    // D pad is analog that only reports -1, 0, 1 for X/Y
    auto axis_data_16 = input_absinfo();
    axis_data_16.value = 0;
    axis_data_16.minimum = -1;
    axis_data_16.maximum = 1;
    axis_data_16.fuzz = 0;
    axis_data_16.flat = 0;
    axis_data_16.resolution = 0;
    libevdev_enable_event_code(dev, EV_ABS, ABS_HAT0X, &axis_data_16);

    auto axis_data_17 = input_absinfo();
    axis_data_17.value = 0;
    axis_data_17.minimum = -1;
    axis_data_17.maximum = 1;
    axis_data_17.fuzz = 0;
    axis_data_17.flat = 0;
    axis_data_17.resolution = 0;
    libevdev_enable_event_code(dev, EV_ABS, ABS_HAT0Y, &axis_data_17);

    int error = libevdev_uinput_create_from_device(dev,
                                                   LIBEVDEV_UINPUT_OPEN_MANAGED,
                                                   &uinput_dev);
    if (error != 0)
    {
        std::stringstream error_msg;
        error_msg << "Could not create device. Sudo? Error Code: " << error;
        throw std::runtime_error(error_msg.str());
    }
}

void Xbox360JoyStick::publish(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    bool should_publish = false;
    std::map<unsigned int, int> updated_buttons;
    std::map<unsigned int, int> updated_axes;

    //Check for updated buttons
    for (int i = 0; i < 11; ++i)
    {
        if (msg->buttons.at(i) != current_buttons[i])
        {
            auto current_state = msg->buttons.at(i);
            auto current_button = button_index_mapping.at(i);
            should_publish = true;
            current_buttons[i] = msg->buttons.at(i);
            updated_buttons.insert(std::map<unsigned int, int>::value_type(current_button, current_state));
        }
    }

    //Check for updated axes
    for (int i = 0; i < 8; ++i)
    {
        std::pair<int, int> range = axes_range_mapping.at(i);
        auto normalized_value = msg->axes.at(i);
        //Joy stick messages are between -1.0 - 1.0. % = (value - min) / (max - min)
        auto normalized_percent = (normalized_value + 1) / 2;
        int actual_value = std::round(normalized_percent * (range.second - range.first)) + range.first;

        if (actual_value != current_axes[i])
        {
            auto current_axis = axes_index_mapping.at(i);
            should_publish = true;
            current_axes[i] = actual_value;
            updated_axes.insert(std::map<unsigned int, int>::value_type(current_axis, actual_value));
        }
    }

    // Publish if changes detected
    if (should_publish)
    {
        if (!updated_buttons.empty())
        {
            for (auto it = updated_buttons.begin(); it != updated_buttons.end(); ++it)
            {
                libevdev_uinput_write_event(uinput_dev, EV_KEY, it->first, it->second);
            }
        }

        if (!updated_axes.empty())
        {
            for (auto it = updated_axes.begin(); it != updated_axes.end(); ++it)
            {
                libevdev_uinput_write_event(uinput_dev, EV_ABS, it->first, it->second);
            }
        }
        libevdev_uinput_write_event(uinput_dev, EV_SYN, SYN_REPORT, 0);
    }
}
} // namespace JoyEmulator