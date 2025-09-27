#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "nodes/controller_read.hpp"
#include "ros_executor_thread.hpp"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    void show_robot_status();
    ~MainWindow();

private:
    std::unique_ptr<RosExecutorThread> ros_thread;
    std::shared_ptr<JoystickReader> controller_reader;
    Ui::MainWindow *ui;
    std::unique_ptr<RosExecutorThread> node_executor_thread;
};

#endif // MAINWINDOW_H
