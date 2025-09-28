#pragma once

#include <QObject>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ImageReader : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit ImageReader(std::string topic_name = "/image");
    QImage rosImagerosImageToQImage(const sensor_msgs::msg::Image &msg);


signals:
    void imageChanged(const QImage &image);

private:
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void timer_callback();

    bool isConnected = false;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::Image::SharedPtr last_msg_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;

};
