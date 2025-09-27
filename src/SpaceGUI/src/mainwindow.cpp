#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "visualizer_base.hpp"
#include "visualizer_factory.hpp"

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
    if (!menu) return;
    menu->clear();

    for (const auto &topic : topics)
    {
        // Label e.g. "/camera/image (sensor_msgs/msg/Image)"
        QString label = QString::fromStdString(topic.first + " (" + topic.second + ")");
        QAction *action = new QAction(label, menu);
        menu->addAction(action);

        // Create a widget when this action is triggered
        connect(action, &QAction::triggered, this, [this, topic]() {
            RosDataWidget *widget = new RosDataWidget(
                QString::fromStdString(topic.second),  // type
                floatingArea);                         // parent is floating area

            // Position widget with a simple offset so they don’t all overlap
            int offset = static_cast<int>(ros_widgets.size()) * 30;
            widget->move(20 + offset, 20 + offset);

            widget->show();
            widget->raise();

            ros_widgets.push_back(widget);

            qDebug() << "Created widget for topic:" << QString::fromStdString(topic.first);
        });
    }
}


MainWindow::~MainWindow()
{
    delete ui;
}
