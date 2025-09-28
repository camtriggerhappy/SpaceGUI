#include "SpaceGUI/image_read.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <string>
#include <QImage>
using std::placeholders::_1;

ImageReader::ImageReader(std::string topic_name)
: QObject(), Node("gui_controller_read")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic_name, 10, std::bind(&ImageReader::topic_callback, this, _1));
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&ImageReader::timer_callback, this));
}

void ImageReader::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
        emit imageChanged(rosImagerosImageToQImage(*msg).copy());
        last_msg_ = msg;
    
}

void ImageReader::timer_callback()
{
    if(last_msg_ == nullptr) {
        emit imageChanged(QImage().copy());
         isConnected = false;
        return;
    }
    rclcpp::Time now = this->get_clock()->now();
        if ((isConnected && (now.seconds() - last_msg_->header.stamp.sec) > 0.5)) {
            isConnected = false;
            emit imageChanged(QImage().copy());
        } 
}

QImage ImageReader::rosImagerosImageToQImage(const sensor_msgs::msg::Image &msg)
{
    if (msg.data.empty())
        return QImage();

    if (msg.encoding == "rgb8")
    {
        return QImage(msg.data.data(), msg.width, msg.height, msg.step, QImage::Format_RGB888).copy();
    }
    else if (msg.encoding == "bgr8")
    {
        // QImage expects RGB, so swap channels
        QImage img(msg.width, msg.height, QImage::Format_RGB888);
        for (size_t y = 0; y < msg.height; ++y)
        {
            const uint8_t *row = &msg.data[y * msg.step];
            for (size_t x = 0; x < msg.width; ++x)
            {
                uint8_t b = row[3 * x + 0];
                uint8_t g = row[3 * x + 1];
                uint8_t r = row[3 * x + 2];
                img.setPixelColor(x, y, QColor(r, g, b));
            }
        }
        return img;
    }
    else if (msg.encoding == "mono8")
    {
        return QImage(msg.data.data(), msg.width, msg.height, msg.step, QImage::Format_Grayscale8).copy();
    }
    else
    {
        throw std::runtime_error("Unsupported image encoding: " + msg.encoding);
    }
}


#include "moc_image_read.cpp"