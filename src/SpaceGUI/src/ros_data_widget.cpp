#include "ros_data_widget.hpp"
#include <QMouseEvent>
#include <QLabel>
#include <QVBoxLayout>

RosDataWidget::RosDataWidget(const QString &dataText, QWidget *parent)
    : QWidget(parent), m_dataText(dataText)
{
    // Visible size and look
    setFixedSize(200, 100);
    setWindowFlags(Qt::Widget); // ensure it's a child widget, not top-level
    setAttribute(Qt::WA_StyledBackground, true);
    setStyleSheet("background-color: #f0f0f0; border: 1px solid #444; border-radius: 6px;");

    // Internal layout + label — layout here is fine (it's inside this widget)
    auto layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 6, 8, 6);
    layout->setSpacing(4);

    m_label = new QLabel(m_dataText, this);
    m_label->setAlignment(Qt::AlignCenter);
    m_label->setWordWrap(true);

    layout->addWidget(m_label);
    setLayout(layout);
}

void RosDataWidget::setDataText(const QString &text)
{
    m_dataText = text;
    if (m_label) m_label->setText(m_dataText);
}

void RosDataWidget::setFixedWidgetSize(const QSize &s)
{
    setFixedSize(s);
}

void RosDataWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton) {
        // record offset from the top-left of this widget to the global cursor position
#if (QT_VERSION >= QT_VERSION_CHECK(6,0,0))
        m_dragStartPos = event->globalPosition().toPoint() - frameGeometry().topLeft();
#else
        m_dragStartPos = event->globalPos() - frameGeometry().topLeft();
#endif
        raise(); // bring to front when clicked
        event->accept();
    }
    QWidget::mousePressEvent(event);
}

void RosDataWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (event->buttons() & Qt::LeftButton) {
#if (QT_VERSION >= QT_VERSION_CHECK(6,0,0))
        QPoint newPos = event->globalPosition().toPoint() - m_dragStartPos;
#else
        QPoint newPos = event->globalPos() - m_dragStartPos;
#endif

        if (parentWidget()) {
            // clamp inside parent area
            QRect parentRect = parentWidget()->rect();
            QSize widgetSize = size();

            int x = std::max(0, std::min(newPos.x(), parentRect.width() - widgetSize.width()));
            int y = std::max(0, std::min(newPos.y(), parentRect.height() - widgetSize.height()));

            move(x, y);
        } else {
            move(newPos);
        }
        event->accept();
    } else {
        QWidget::mouseMoveEvent(event);
    }
}

#include "moc_ros_data_widget.cpp"