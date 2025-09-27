#include "ros_executor_thread.hpp"

#include "rclcpp/rclcpp.hpp"
RosExecutorThread::RosExecutorThread(std::shared_ptr<rclcpp::Node> node)
  : node_(std::move(node))
{
    
}

RosExecutorThread::~RosExecutorThread()
{
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void RosExecutorThread::run()
{
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node_);
  exec.spin();
}
#include "moc_ros_executor_thread.cpp"
