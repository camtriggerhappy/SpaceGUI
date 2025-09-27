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
    ui->robot_status->setReadOnly(true);
    ui->robot_status->setText("Robot is not connected");
}

MainWindow::~MainWindow()
{
    delete ui;
}
