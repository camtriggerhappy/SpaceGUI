#pragma once

#include <QWidget>
#include <QVBoxLayout>
#include <QMouseEvent>
#include <QLabel>

class RosDataWidget : public QWidget {
    Q_OBJECT
public:
    explicit RosDataWidget(const QString &title, QWidget *parent = nullptr);

    void setContent(QWidget *content);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

private:
    QPoint dragStart;
    QLabel *titleLabel {nullptr};
    QWidget *contentWidget {nullptr};
    QVBoxLayout *layout {nullptr};
};
