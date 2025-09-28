#include "log_read.hpp"
#include <QMetaObject>
#include <QString>
#include <chrono>
#include "rcl_interfaces/msg/log.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

LogReader::LogReader(const std::string &topic_name)
: QObject(), Node("gui_log_reader")
{
    // Subscribe to the ROS 2 log topic
    subscription_ = this->create_subscription<rcl_interfaces::msg::Log>(
        topic_name, 10, std::bind(&LogReader::topic_callback, this, _1));

    // Timer to check connectivity (optional)
    timer_ = this->create_wall_timer(
        1s, std::bind(&LogReader::timer_callback, this));
}

void LogReader::topic_callback(const rcl_interfaces::msg::Log::SharedPtr msg)
{
    last_msg_ = msg;
    isConnected = true;

    // Properly format timestamp
    QString timeStr = QString("%1.%2")
                          .arg(msg->stamp.sec)
                          .arg(msg->stamp.nanosec, 9, 10, QChar('0')); // nanoseconds padded to 9 digits

    // Construct the full log string
    QString logStr = QString("[%1] [%2] %3")
                         .arg(timeStr)
                         .arg(QString::fromStdString(msg->name))
                         .arg(QString::fromStdString(msg->msg));

    // Emit Qt signal in a thread-safe way
    QMetaObject::invokeMethod(this, [this, logStr]() {
        emit newLogMessage(logStr);
    }, Qt::QueuedConnection);
}


void LogReader::timer_callback()
{
    // if (!last_msg_) {
    //     isConnected = false;
    //     return;
    // }

    // rclcpp::Time now = this->get_clock()->now();

    // // Optionally emit a "disconnected" message if no recent logs
    //     emit newLogMessage("[No recent log messages]");
}

#include "moc_log_read.cpp"
