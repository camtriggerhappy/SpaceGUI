// ros_data_widget.hpp
#ifndef ROS_DATA_WIDGET_HPP
#define ROS_DATA_WIDGET_HPP

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QString>

class RosDataWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RosDataWidget(const QString &topicType, QWidget *parent = nullptr);

private:
    QWidget *createVisualizer(const QString &topicType);

    QVBoxLayout *layout;
};

#endif // ROS_DATA_WIDGET_HPP
