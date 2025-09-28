#pragma once

#include <QWidget>
#include <QTextEdit>
#include <QVBoxLayout>
#include <QString>
#include "moveable_widget.hpp"

class MovableLogWidget : public MovableWidget
{
    Q_OBJECT
public:
    explicit MovableLogWidget(const QString &title = "Log Viewer", QWidget *parent = nullptr);

public slots:
    void appendLogMessage(const QString &msg);

private:
    QTextEdit *logEdit_;
    QVBoxLayout *layout_;
};
