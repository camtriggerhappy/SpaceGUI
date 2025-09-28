#pragma once

#include <QObject>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"

class LogReader : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit LogReader(const std::string &topic_name = "/rosout");

signals:
    void newLogMessage(const QString &msg);

private:
    void topic_callback(const rcl_interfaces::msg::Log::SharedPtr msg);
    void timer_callback();

    bool isConnected = false;
    rclcpp::TimerBase::SharedPtr timer_;
    rcl_interfaces::msg::Log::SharedPtr last_msg_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr subscription_;
};
