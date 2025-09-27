#pragma once
#include "visualizer_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <memory>
#include <cstring>

template<typename MsgT>
class VisualizerTemplate : public VisualizerBase
{
public:
    explicit VisualizerTemplate(QWidget *parent = nullptr)
        : VisualizerBase(parent)
    {}

    ~VisualizerTemplate() override = default;

    void subscribe(const std::shared_ptr<rclcpp::Node> &node, const std::string &topic)
    {
        subscription_ = node->create_subscription<MsgT>(
            topic, 10,
            [this](typename MsgT::SharedPtr msg) {
                handleMessage(msg);
            });
    }

    void setSerializedMessage(const QByteArray &serialized) override
    {
        rclcpp::SerializedMessage s_msg;
        s_msg.reserve(static_cast<size_t>(serialized.size()));
        auto &rcl_serialized = s_msg.get_rcl_serialized_message();
        std::memcpy(rcl_serialized.buffer, serialized.constData(),
                    static_cast<size_t>(serialized.size()));
        rcl_serialized.buffer_length = static_cast<size_t>(serialized.size());

        auto msg = std::make_shared<MsgT>();
        rclcpp::Serialization<MsgT> serializer;
        serializer.deserialize_message(&s_msg, msg.get());

        handleMessage(msg);
    }

protected:
    virtual void handleMessage(const std::shared_ptr<MsgT> &msg) = 0;

private:
    rclcpp::Node::SharedPtr node {rclcpp::Node::make_shared("visualizer_node")};
    typename rclcpp::Subscription<MsgT>::SharedPtr subscription_;
};
