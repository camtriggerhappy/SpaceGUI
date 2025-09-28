#pragma once

#include <QObject>
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "std_msgs/msg/float64.hpp"

class NumberReader : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit NumberReader(std::string topic_name = "/data");

signals:
    void newNumber(const float &number);

private:
    void topic_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void timer_callback();

    bool isConnected = false;
    rclcpp::TimerBase::SharedPtr timer_;
    std_msgs::msg::Float64::SharedPtr last_msg_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
};
