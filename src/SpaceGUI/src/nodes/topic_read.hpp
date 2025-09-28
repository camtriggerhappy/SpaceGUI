#ifndef TOPIC_READER_H
#define TOPIC_READER_H

#include <QObject>
#include <vector>
#include <utility>
#include <string>
#include "rclcpp/rclcpp.hpp"

class TopicReader : public QObject , public rclcpp::Node
{
    Q_OBJECT

public:    
    explicit TopicReader(QObject* parent = nullptr);
    void timer_callback();

    // Example method that simulates new data being available

signals:
    // Signal emitting a vector of std::pair<std::string, double>
    void dataReady(const std::vector<std::pair<std::string, std::string>> &data);

private:
    rclcpp::TimerBase::SharedPtr timer_;

};


#endif // TOPIC_READER_H
