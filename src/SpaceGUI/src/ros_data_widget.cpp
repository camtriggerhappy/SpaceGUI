// ros_data_widget.cpp
#include "ros_data_widget.hpp"
#include <QDebug>

// Example: image visualizer
#include <QLabel>
#include <QPixmap>

RosDataWidget::RosDataWidget(const QString &topicType, QWidget *parent)
    : QWidget(parent)
{
    setFixedSize(200, 150); // default size
    setStyleSheet("background: white; border: 1px solid black;");

    layout = new QVBoxLayout(this);
    QWidget *visualizer = createVisualizer(topicType);
    if (visualizer)
        layout->addWidget(visualizer);
}

QWidget *RosDataWidget::createVisualizer(const QString &topicType)
{
    if (topicType == "sensor_msgs/msg/Image") {
        // Placeholder image visualizer
        QLabel *img = new QLabel("Image Viewer");
        img->setAlignment(Qt::AlignCenter);
        img->setStyleSheet("background: black; color: white;");
        img->setMinimumSize(180, 120);
        return img;
    }
    else if (topicType == "sensor_msgs/msg/Joy") {
        QLabel *joy = new QLabel("Joystick Data");
        joy->setAlignment(Qt::AlignCenter);
        return joy;
    }
    else {
        QLabel *fallback = new QLabel("Unsupported type:\n" + topicType);
        fallback->setAlignment(Qt::AlignCenter);
        return fallback;
    }
}
