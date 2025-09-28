#include "SpaceGUI/controller_read.hpp"
#include <string>
using std::placeholders::_1;

JoystickReader::JoystickReader(std::string topic_name)
: QObject(), Node("gui_controller_read")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        topic_name, 10, std::bind(&JoystickReader::topic_callback, this, _1));
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&JoystickReader::timer_callback, this));
}

void JoystickReader::topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    bool connected = (msg->header.stamp.sec != 0);
    last_msg_ = msg;
    if (connected != isConnected) {
        isConnected = connected;
        emit statusChanged(isConnected ? "Connected" : "Disconnected");
    }
}

void JoystickReader::timer_callback()
{
    if(last_msg_ == nullptr) {
        emit statusChanged("Disconnected");
         isConnected = false;
        return;
    }
    rclcpp::Time now = this->get_clock()->now();
        if ((isConnected && (now.seconds() - last_msg_->header.stamp.sec) > 0.5)) {
            isConnected = false;
            emit statusChanged("Disconnected");
        } 
}


#include "moc_controller_read.cpp"