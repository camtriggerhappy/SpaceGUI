#pragma once
// visualizer_base.hpp
#include <QWidget>
#include <QByteArray>
#include <QString>

class VisualizerBase : public QWidget
{
    Q_OBJECT
public:
    explicit VisualizerBase(QWidget *parent = nullptr);
    ~VisualizerBase() override;

    // Called by GUI thread to deliver a serialized message payload for this visualizer.
    virtual void setSerializedMessage(const QByteArray &serialized) = 0;

    // Optional: provide topic/type metadata
    virtual void setTopicInfo(const QString &topic, const QString &ros_type) {
        Q_UNUSED(topic); Q_UNUSED(ros_type);
    }
};
