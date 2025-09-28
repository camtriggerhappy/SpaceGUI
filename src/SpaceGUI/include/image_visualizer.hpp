#pragma once
// image_visualizer.hpp
#include "visualizer_template.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <QLabel>
#include <QVBoxLayout>
#include <QPixmap>
#include <QImage>

class ImageVisualizer : public VisualizerTemplate<sensor_msgs::msg::Image>
{
    Q_OBJECT
public:
    explicit ImageVisualizer(QWidget* parent = nullptr);
    ~ImageVisualizer() override = default;

protected:
    void handleMessage(const std::shared_ptr<sensor_msgs::msg::Image> &msg) override;

private:
    QLabel* m_label;
    QVBoxLayout* m_layout;
    // helper to convert ROS image to QImage (returns a deep-copied QImage)
    QImage rosImageToQImage(const sensor_msgs::msg::Image &msg);
};
