# ros_joy_emulator
Ros2 node to emulate a joystick on local device based on joy sensor message. Originally built with Dashing release in mind.

# Build
* sudo rosdep install -i --from-path <src_path> --rosdistro dashing -y
* colcon build

# Usage
This is still initial proof of concept and not a finalized serivce yet. Checks joy topic explicitly. Launching node with ros2 run.

On device you want to emulate joystick on:
* Build package
* Create su terminal session (needed to create uinput evdev). sudo su root
* . install/setup.bash
* ros2 run joy_emu_cpp joy_emulator
* check /dev/input for device number (event##)

For testing launch a service that publishes to /joy. Testing wtih a real joystick:
* Plug in joystick
* ros2 run joy joy_node

Validation:
* Watch joy topic with - ros2 topic echo /joy
* Watch emulated joystick output with - sudo evemu-record /dev/input/event##