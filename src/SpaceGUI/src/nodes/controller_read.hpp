#pragma once

#include <QObject>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class JoystickReader : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit JoystickReader();

signals:
    void statusChanged(const QString &status);

private:
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

    bool isConnected = false;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};
