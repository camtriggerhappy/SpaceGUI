#include "moveable_numeric_widget.hpp"

MovableNumericWidget::MovableNumericWidget(const QString &title, QWidget *parent)
    : MovableWidget(parent)
{
    layout_ = new QVBoxLayout(this);

    // Display current value
    valueLabel_ = new QLabel("0.0", this);
    valueLabel_->setAlignment(Qt::AlignCenter);


    layout_->addWidget(valueLabel_);
        QFont font = valueLabel_->font();
    font.setPointSize(10);  // adjust this number as needed
    valueLabel_->setFont(font);
    valueLabel_->setFixedHeight(20); // adjust as needed
    layout_->addWidget(valueLabel_);


    // Plot area
    plot_ = new QCustomPlot(this);
    layout_->addWidget(plot_);

    graph_ = plot_->addGraph();
    graph_->setPen(QPen(Qt::blue));

    plot_->xAxis->setLabel("Time (s)");
    plot_->yAxis->setLabel(title);

    plot_->xAxis->setRange(0, maxPoints_);
    plot_->yAxis->setRange(0, 1.0); // initial range
}

void MovableNumericWidget::addDataPoint(double value)
{
    time_ += 0.1; // adjust to your ROS message rate

    dataX_.push_back(time_);
    dataY_.push_back(value);

    if (dataX_.size() > maxPoints_) {
        dataX_.pop_front();
        dataY_.pop_front();
    }

    QVector<double> x(dataX_.begin(), dataX_.end());
    QVector<double> y(dataY_.begin(), dataY_.end());

    graph_->setData(x, y);

    plot_->xAxis->setRange(time_ - maxPoints_ * 0.1, time_);
    plot_->yAxis->rescale(true);

    valueLabel_->setText(QString::number(value, 'f', 2));

    plot_->replot();
}
#include "moc_moveable_numeric_widget.cpp"