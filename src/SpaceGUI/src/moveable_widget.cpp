#include "SpaceGUI/moveable_widget.hpp"

#include "moveable_widget.hpp"

MovableWidget::MovableWidget(QWidget *parent)
    : QWidget(parent)
{
    setMouseTracking(true);
    setMinimumSize(50, 50);
    setAttribute(Qt::WA_StyledBackground, true);
}

MovableWidget::ResizeRegion MovableWidget::detectRegion(const QPoint &p)
{
    bool left   = p.x() < borderWidth_;
    bool right  = p.x() > width() - borderWidth_;
    bool top    = p.y() < borderWidth_;
    bool bottom = p.y() > height() - borderWidth_;

    if (top && left)    return TOPLEFT;
    if (top && right)   return TOPRIGHT;
    if (bottom && left) return BOTTOMLEFT;
    if (bottom && right)return BOTTOMRIGHT;
    if (top)            return TOP;
    if (bottom)         return BOTTOM;
    if (left)           return LEFT;
    if (right)          return RIGHT;

    return NONE;
}

void MovableWidget::updateCursor(ResizeRegion region)
{
    switch (region) {
    case LEFT:
    case RIGHT:         setCursor(Qt::SizeHorCursor);  break;

    case TOP:
    case BOTTOM:        setCursor(Qt::SizeVerCursor);  break;

    case TOPLEFT:
    case BOTTOMRIGHT:   setCursor(Qt::SizeFDiagCursor); break;

    case TOPRIGHT:
    case BOTTOMLEFT:    setCursor(Qt::SizeBDiagCursor); break;

    default:            setCursor(Qt::ArrowCursor);
    }
}

void MovableWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        resizeRegion_ = detectRegion(event->pos());
        initialGeometry_ = geometry();
        dragStartPos_ = event->globalPosition().toPoint();
        event->accept();
    }
}

void MovableWidget::mouseMoveEvent(QMouseEvent *event)
{
    QPoint globalPos = event->globalPosition().toPoint();

    if (!(event->buttons() & Qt::LeftButton)) {
        updateCursor(detectRegion(event->pos()));
        return;
    }

    if (resizeRegion_ != NONE) {
        QRect g = initialGeometry_;
        int dx = globalPos.x() - dragStartPos_.x();
        int dy = globalPos.y() - dragStartPos_.y();

        switch (resizeRegion_) {
        case LEFT:        g.setLeft(g.left() + dx); break;
        case RIGHT:       g.setRight(g.right() + dx); break;
        case TOP:         g.setTop(g.top() + dy); break;
        case BOTTOM:      g.setBottom(g.bottom() + dy); break;

        case TOPLEFT:     g.setTop(g.top() + dy);
                          g.setLeft(g.left() + dx); break;

        case TOPRIGHT:    g.setTop(g.top() + dy);
                          g.setRight(g.right() + dx); break;

        case BOTTOMLEFT:  g.setBottom(g.bottom() + dy);
                          g.setLeft(g.left() + dx); break;

        case BOTTOMRIGHT: g.setBottom(g.bottom() + dy);
                          g.setRight(g.right() + dx); break;

        default: break;
        }

        setGeometry(g.normalized());
        return;
    }

    // MOVE WIDGET
    move(globalPos - dragStartPos_);
}

void MovableWidget::mouseReleaseEvent(QMouseEvent *event)
{
    resizeRegion_ = NONE;
    setCursor(Qt::ArrowCursor);
    event->accept();
}

void MovableWidget::leaveEvent(QEvent *)
{
    if (!mouseGrabber()) setCursor(Qt::ArrowCursor);
}

#include "moc_moveable_widget.cpp"