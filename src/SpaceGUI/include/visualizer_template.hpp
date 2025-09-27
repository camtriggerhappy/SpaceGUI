#pragma once
// visualizer_template.hpp
#include "visualizer_base.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <memory>
#include <cstring>

template<typename MsgT>
class VisualizerTemplate : public VisualizerBase
{
public:
    explicit VisualizerTemplate(QWidget *parent = nullptr)
    : VisualizerBase(parent) {}

    ~VisualizerTemplate() override = default;

    void setSerializedMessage(const QByteArray &serialized) override
    {
        // Build an rclcpp::SerializedMessage with the incoming bytes
        rclcpp::SerializedMessage s_msg;
        s_msg.reserve(static_cast<size_t>(serialized.size()));
        auto &rcl_serialized = s_msg.get_rcl_serialized_message();
        std::memcpy(rcl_serialized.buffer, serialized.constData(),
                    static_cast<size_t>(serialized.size()));
        rcl_serialized.buffer_length = static_cast<size_t>(serialized.size());

        // Deserialize into a message instance
        auto msg = std::make_shared<MsgT>();
        rclcpp::Serialization<MsgT> serializer;
        serializer.deserialize_message(&s_msg, msg.get());

        // Forward to concrete handler implemented by subclass
        handleMessage(msg);
    }

protected:
    // NOTE: signature must match exactly in overrides below
    virtual void handleMessage(const std::shared_ptr<MsgT> & msg) = 0;
};
