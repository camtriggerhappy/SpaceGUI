#pragma once

#include <QWidget>
#include <QMouseEvent>

class MovableWidget : public QWidget
{
    Q_OBJECT
public:
    explicit MovableWidget(QWidget *parent = nullptr);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void leaveEvent(QEvent *event) override;

private:
private:
    enum ResizeRegion {
        NONE,
        LEFT, RIGHT, TOP, BOTTOM,
        TOPLEFT, TOPRIGHT, BOTTOMLEFT, BOTTOMRIGHT
    };

    ResizeRegion detectRegion(const QPoint &pos);
    void updateCursor(const ResizeRegion region);

    QPoint dragStartPos_;
    QRect initialGeometry_;
    ResizeRegion resizeRegion_ = NONE;
    const int borderWidth_ = 16;   // Resize margin around edges


};
