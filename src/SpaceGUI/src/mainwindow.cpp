#include "mainwindow.h"
#include "ui_mainwindow.h"



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

void MainWindow::show_robot_status()
{
    controller_reader = std::make_shared<JoystickReader>();
    ros_thread = std::make_unique<RosExecutorThread>(controller_reader);
    ros_thread->start();

connect(controller_reader.get(), &JoystickReader::statusChanged, this,
        [this](const QString &status) {
            ui->robotStatusLabel->setText(status);
        });
}

MainWindow::~MainWindow()
{
    delete ui;
}

