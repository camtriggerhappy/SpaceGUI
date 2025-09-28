#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "image_read.hpp"
#include "SpaceGUI/moveable_widget.hpp"
#include "SpaceGUI/moveable_numeric_widget.hpp"
#include "SpaceGUI/number_read.hpp"
#include "SpaceGUI/moveable_log_widget.hpp"
#include "SpaceGUI/log_read.hpp"

#include <QMenu>
#include <QLabel>
#include <QVBoxLayout>
#include <QAction>
#include <QMessageBox>
#include <QDebug>
#include <QEvent>
#include <QResizeEvent>
#include <QMouseEvent>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent),
                                          ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    floatingArea = new QWidget(ui->centralwidget);
    floatingArea->setAttribute(Qt::WA_TransparentForMouseEvents, false); // children receive events
    floatingArea->setAttribute(Qt::WA_StyledBackground, true);
    floatingArea->setStyleSheet("background: transparent;"); // visually transparent
    floatingArea->setGeometry(ui->centralwidget->rect());
    floatingArea->raise(); // keep on top
    floatingArea->show();

    // Size and position the floatingArea to match centralWidget
    ui->centralwidget->installEventFilter(this);


}

void MainWindow::show_left_controller()
{
    controller_reader = std::make_shared<JoystickReader>();
    ros_threads.push_back(std::make_unique<RosExecutorThread>(controller_reader));
    ros_threads.back()->start();

    connect(controller_reader.get(), &JoystickReader::statusChanged, this,
            [this](const QString &status)
            {
                ui->controllerLeft->setText(status);
            });
}

void MainWindow::show_right_controller()
{
    controller_reader = std::make_shared<JoystickReader>("/right_joy");
    ros_threads.push_back(std::make_unique<RosExecutorThread>(controller_reader));
    ros_threads.back()->start();

    connect(controller_reader.get(), &JoystickReader::statusChanged, this,
            [this](const QString &status)
            {
                ui->controllerRight->setText(status);
            });
}

bool MainWindow::eventFilter(QObject *watched, QEvent *event)
{
    if (watched == ui->centralwidget)
    {
        if (event->type() == QEvent::Resize || event->type() == QEvent::Show)
        {
            const QSize s = ui->centralwidget->size();
            floatingArea->setGeometry(0, 0, s.width(), s.height());
            floatingArea->raise();
            return false; // allow normal processing
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

void MainWindow::populate_top_menu()
{
    menu_options.push_back(std::make_shared<QMenu>(tr("Main Menu")));
    menu_options.push_back(std::make_shared<QMenu>(tr("autonomy mission")));
    menu_options.push_back(std::make_shared<QMenu>(tr("add Topic")));
    topic_reader = std::make_shared<TopicReader>();
    topic_reader_thread = std::make_shared<RosExecutorThread>(topic_reader);
    topic_reader_thread->start();

    for (auto &item : menu_options)
    {
        menuBar()->addMenu(item.get());
    }

    connect(topic_reader.get(), &TopicReader::dataReady, this,
            [this](const std::vector<std::pair<std::string, std::string>> &topics)
            {
                add_topic_list_to_menu(menu_options[2].get(), topics);
            });
}

void MainWindow::add_topic_list_to_menu(
    QMenu *menu,
    const std::vector<std::pair<std::string, std::string>> &topics)
{
    if (!menu)
        return;
    menu->clear();

    for (const auto &topic : topics)
    {
        // Label e.g. "/camera/image (sensor_msgs/msg/Image)"
        QString label = QString::fromStdString(topic.first + " (" + topic.second + ")");
        QAction *action = new QAction(label, menu);
        menu->addAction(action);

        // Create a widget when this action is triggered
        connect(action, &QAction::triggered, this, [this, topic]()
                {
    if (topic.second == "sensor_msgs/msg/Image")
    {
        createImageWidget(topic.first);
    }
    else if (topic.second == "std_msgs/msg/Float64")
    {
        createNumericWidget(topic.first);
    } 
    else if(topic.second == "rcl_interfaces/msg/Log"){
        createLogWidget(topic.first);
    }
    else
    {
        QMessageBox::information(this, "Unsupported Topic Type",
                                 QString("Topic type '%1' is not supported for widget creation.").arg(QString::fromStdString(topic.second)));
    } });
    }
}
void MainWindow::createImageWidget(const std::string &topic_name)
{
    auto imageWidget = new MovableWidget(floatingArea);
    imageWidget->setWindowFlags(Qt::Widget);

    auto layout = new QVBoxLayout(imageWidget);
    auto label = new QLabel(imageWidget);
    label->setMinimumSize(320, 240);
    layout->addWidget(label);
    imageWidget->setLayout(layout);
    imageWidget->show();

    auto imageReader = std::make_shared<ImageReader>(topic_name);
    ros_threads.push_back(std::make_shared<RosExecutorThread>(imageReader));
    ros_threads.back()->start();

    connect(imageReader.get(), &ImageReader::imageChanged, this, [label](const QImage &img)
            { label->setPixmap(QPixmap::fromImage(img).scaled(
                  label->size(),
                  Qt::KeepAspectRatio,
                  Qt::SmoothTransformation)); }, Qt::QueuedConnection);
}

void MainWindow::createNumericWidget(const std::string &topic_name)
{
    auto numWidget = new MovableNumericWidget(QString::fromStdString(topic_name), floatingArea);
    numWidget->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
    numWidget->setGeometry(50, 50, 360, 250);
    numWidget->show();

    auto numReader = std::make_shared<NumberReader>(topic_name);
    ros_threads.push_back(std::make_shared<RosExecutorThread>(numReader));
    ros_threads.back()->start();

    connect(numReader.get(), &NumberReader::newNumber, numWidget,
            &MovableNumericWidget::addDataPoint, Qt::QueuedConnection);
}

void MainWindow::createLogWidget(const std::string &topic_name)
{
    auto logWidget = new MovableLogWidget("ROS Log", floatingArea);
    logWidget->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
    logWidget->setGeometry(50, 50, 400, 300);
    logWidget->show();

    // ROS log reader
    auto logReader = std::make_shared<LogReader>(topic_name);
    ros_threads.push_back(std::make_shared<RosExecutorThread>(logReader));
    ros_threads.back()->start();

    // Connect signal to widget (thread-safe)
    connect(logReader.get(), &LogReader::newLogMessage,
            logWidget, &MovableLogWidget::appendLogMessage,
            Qt::QueuedConnection);
}

MainWindow::~MainWindow()
{
    delete ui;
}

#include "moc_mainwindow.cpp"