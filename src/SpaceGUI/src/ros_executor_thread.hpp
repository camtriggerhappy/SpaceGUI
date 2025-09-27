#pragma once
#include <QThread>
#include <memory>
#include "rclcpp/rclcpp.hpp"


class RosExecutorThread : public QThread
{
    Q_OBJECT

public:
    explicit RosExecutorThread(std::shared_ptr<rclcpp::Node> node);
    ~RosExecutorThread() override;

protected:
    void run() override;

private:
    std::shared_ptr<rclcpp::Node> node_;
};
