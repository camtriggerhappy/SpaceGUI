#include "controller_read.hpp"
using std::placeholders::_1;

JoystickReader::JoystickReader()
: QObject(), Node("gui_controller_read")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/Driver/Joy", 10, std::bind(&JoystickReader::topic_callback, this, _1));
}

void JoystickReader::topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    bool connected = (msg->header.stamp.sec != 0);
    if (connected != isConnected) {
        isConnected = connected;
        emit statusChanged(isConnected ? "Connected" : "Disconnected");
    }
}
