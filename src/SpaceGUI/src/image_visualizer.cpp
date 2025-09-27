// image_visualizer.cpp
#include "image_visualizer.hpp"
#include "visualizer_factory.hpp"
#include <cstring> // memcpy
#include <QDebug>

ImageVisualizer::ImageVisualizer(QWidget* parent)
: VisualizerTemplate(parent)
{
    setFixedSize(320, 240); // default size (adjust as needed)
    setAttribute(Qt::WA_StyledBackground, true);
    setStyleSheet("background: #111; border: 1px solid #666;");

    m_layout = new QVBoxLayout(this);
    m_layout->setContentsMargins(4,4,4,4);
    m_label = new QLabel("no image", this);
    m_label->setAlignment(Qt::AlignCenter);
    m_label->setWordWrap(true);
    m_layout->addWidget(m_label);
    setLayout(m_layout);
}

void ImageVisualizer::handleMessage(const std::shared_ptr<sensor_msgs::msg::Image> &msg)
{
    if (!msg) return;

    QImage qimg = rosImageToQImage(*msg);
    if (!qimg.isNull()) {
        QPixmap pm = QPixmap::fromImage(qimg);
        m_label->setPixmap(pm.scaled(size(), Qt::KeepAspectRatio));
    } else {
        m_label->setText(QString("Unsupported encoding: %1").arg(QString::fromStdString(msg->encoding)));
    }
}

// Very small converter supporting common encodings: rgb8, bgr8, mono8.
// Produces a deep copy QImage (safe after msg goes out of scope).
QImage ImageVisualizer::rosImageToQImage(const sensor_msgs::msg::Image &msg)
{
    const int w = static_cast<int>(msg.width);
    const int h = static_cast<int>(msg.height);
    const std::string enc = msg.encoding;
    const auto step = static_cast<int>(msg.step);
    const unsigned char* data = msg.data.empty() ? nullptr : msg.data.data();

    if (!data || w <= 0 || h <= 0) return QImage();

    // rgb8
    if (enc == "rgb8" && step >= 3*w) {
        // QImage expects RGB888 ordering; we can construct referencing data and then copy
        QImage img(const_cast<unsigned char*>(data), w, h, step, QImage::Format_RGB888);
        return img.copy(); // deep copy to own data
    }

    // bgr8 -> convert to RGB
    if (enc == "bgr8" && step >= 3*w) {
        QImage img(w, h, QImage::Format_RGB888);
        for (int y = 0; y < h; ++y) {
            const unsigned char* row = data + y * step;
            unsigned char* out = img.scanLine(y);
            for (int x = 0; x < w; ++x) {
                // b g r -> r g b
                out[3*x + 0] = row[3*x + 2];
                out[3*x + 1] = row[3*x + 1];
                out[3*x + 2] = row[3*x + 0];
            }
        }
        return img;
    }

    // mono8 -> grayscale
    if (enc == "mono8" && step >= w) {
        QImage img(const_cast<unsigned char*>(data), w, h, step, QImage::Format_Grayscale8);
        return img.copy();
    }

    // Add more encodings as needed (rgba8, bgra8, etc.)
    qDebug() << "ImageVisualizer: unsupported encoding:" << QString::fromStdString(enc);
    return QImage();
}
#include "moc_image_visualizer.cpp"
