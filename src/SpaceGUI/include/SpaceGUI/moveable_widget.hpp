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

private:
    QPoint dragStartPos_;
};
