#include "SpaceGUI/moveable_widget.hpp"

MovableWidget::MovableWidget(QWidget *parent)
    : QWidget(parent)
{
    setAttribute(Qt::WA_StyledBackground, true); // optional, allows background styling
}

void MovableWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        // store offset between click and widget top-left
        dragStartPos_ = event->globalPosition().toPoint() - this->pos();
        event->accept();
    }
}

void MovableWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        // move widget following the cursor
        move(event->globalPosition().toPoint() - dragStartPos_);
        event->accept();
    }
}
#include "moc_moveable_widget.cpp"