#pragma once

#include "SpaceGUI/moveable_widget.hpp"
#include "qcustomplot.h"
#include <QVBoxLayout>
#include <QLabel>
#include <deque>

class MovableNumericWidget : public MovableWidget
{
    Q_OBJECT
public:
    explicit MovableNumericWidget(const QString &title, QWidget *parent = nullptr);

public slots:
    void addDataPoint(double value); // connect this to your ROS numeric topic

private:
    QVBoxLayout *layout_;
    QLabel *valueLabel_;
    QCustomPlot *plot_;
    QCPGraph *graph_;

    int maxPoints_ = 200; // number of points to keep
    std::deque<double> dataX_;
    std::deque<double> dataY_;
    double time_ = 0.0;
};
