#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "visualizer_base.hpp"
#include "visualizer_factory.hpp"
#include "visualizer_template.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_visualizer.hpp"

#include <QMenu>
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

    QWidget *dbg = new QWidget(floatingArea);
    dbg->setStyleSheet("background: rgba(255,0,0,0.25); border: 1px solid red;");
    dbg->setGeometry(10, 10, 120, 60);
    dbg->show();
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
    RosDataWidget *container = new RosDataWidget(QString::fromStdString(topic.first), floatingArea);

    if (QString::fromStdString(topic.second).contains("Image")) {
        ImageVisualizer *visualizer = new ImageVisualizer(container);
        container->setContent(visualizer);
    } else {
        QLabel *label = new QLabel("Unsupported type: " + QString::fromStdString(topic.second));
        label->setAlignment(Qt::AlignCenter);
        container->setContent(label);
    }

    container->move(50, 50);
    container->show();
    ros_widgets.push_back(container); });
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
