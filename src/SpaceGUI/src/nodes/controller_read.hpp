#pragma once

#include <QObject>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoystickReader : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit JoystickReader(std::string topic_name = "/joy");

signals:
    void statusChanged(const QString &status);

private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void timer_callback();

    bool isConnected = false;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Joy::SharedPtr last_msg_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};
