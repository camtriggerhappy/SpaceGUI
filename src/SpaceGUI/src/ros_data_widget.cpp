#include "ros_data_widget.hpp"

RosDataWidget::RosDataWidget(const QString &title, QWidget *parent)
    : QWidget(parent)
{
    setFixedSize(320, 240);
    setStyleSheet("background: white; border: 1px solid black;");

    layout = new QVBoxLayout(this);
    layout->setContentsMargins(2, 2, 2, 2);

    titleLabel = new QLabel(title);
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("background: #444; color: white;");
    layout->addWidget(titleLabel);
}

void RosDataWidget::setContent(QWidget *content) {
    if (contentWidget) {
        layout->removeWidget(contentWidget);
        contentWidget->deleteLater();
    }
    contentWidget = content;
    if (contentWidget) {
        layout->addWidget(contentWidget, 1);
    }
}

void RosDataWidget::mousePressEvent(QMouseEvent *event) {
    dragStart = event->pos();
}

void RosDataWidget::mouseMoveEvent(QMouseEvent *event) {
    if (event->buttons() & Qt::LeftButton) {
        move(mapToParent(event->pos() - dragStart));
    }
}
