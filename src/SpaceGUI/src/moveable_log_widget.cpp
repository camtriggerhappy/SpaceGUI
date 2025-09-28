#include "moveable_log_widget.hpp"
#include <QDateTime>

MovableLogWidget::MovableLogWidget(const QString &title, QWidget *parent)
: MovableWidget(parent)
{
    setWindowTitle(title);
    setGeometry(50, 50, 400, 300); // default size and position

    layout_ = new QVBoxLayout(this);
    logEdit_ = new QTextEdit(this);
    logEdit_->setReadOnly(true);
    layout_->addWidget(logEdit_);
    setLayout(layout_);

    show();
}

void MovableLogWidget::appendLogMessage(const QString &msg)
{
    // Add timestamp in format: [HH:mm:ss]
    QString timestamp = QDateTime::currentDateTime().toString("[HH:mm:ss] ");
    logEdit_->append(timestamp + msg);
}
#include "moc_moveable_log_widget.cpp"