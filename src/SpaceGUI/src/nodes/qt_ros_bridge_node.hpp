#pragma once
#include <QObject>
#include <memory>
#include "rclcpp/rclcpp.hpp"
 #include <QLineSeries>

class qt_ros_bridge_node : public QObject, rclcpp::Node
{
public:
    Q_OBJECT
    qt_ros_bridge_node(const std::string &node_name = "qt_ros_bridge_node")
        : QObject(), rclcpp::Node(node_name){

        }

    ~qt_ros_bridge_node() override = default;

private:
    QString display_data_string();
    


     

}