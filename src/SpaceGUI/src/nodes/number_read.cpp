#include "SpaceGUI/number_read.hpp"
#include <std_msgs/msg/float64.hpp>
#include <QDebug>
using std::placeholders::_1;

NumberReader::NumberReader(std::string topic_name)
    : QObject(), rclcpp::Node("gui_number_reader")
{
    // Subscribe to numeric topic
    subscription_ = this->create_subscription<std_msgs::msg::Float64>(
        topic_name, 10, std::bind(&NumberReader::topic_callback, this, _1));

    // Timer to check connection status
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&NumberReader::timer_callback, this));
}

void NumberReader::topic_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    if (msg) {
        last_msg_ = msg;
        isConnected = true;
        emit newNumber(static_cast<float>(msg->data));
    }
}

void NumberReader::timer_callback()
{

}
#include "moc_number_read.cpp"