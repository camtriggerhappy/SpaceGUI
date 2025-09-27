#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGridLayout>
#include "nodes/controller_read.hpp"
#include "ros_executor_thread.hpp"
#include "nodes/topic_read.hpp"
#include "ros_data_widget.hpp"

namespace Ui
{
    class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    void show_left_controller();
    void show_right_controller();
    void populate_top_menu();
    bool eventFilter(QObject *watched, QEvent *event) override;
void add_topic_list_to_menu(QMenu *menu, const std::vector<std::pair<std::string, std::string>> &topics);

    ~MainWindow();

private:
    std::vector<std::shared_ptr<RosExecutorThread>> ros_threads;
    std::shared_ptr<rclcpp::Node> gui_node_;
    std::unique_ptr<RosExecutorThread> ros_thread_;
    std::vector<std::shared_ptr<QMenu>> menu_options;
    std::shared_ptr<RosExecutorThread> ros_thread;
    std::shared_ptr<JoystickReader> controller_reader;
    std::shared_ptr<TopicReader> topic_reader;
    std::shared_ptr<RosExecutorThread> topic_reader_thread;
    QWidget *floatingArea = nullptr;
    std::vector<RosDataWidget*>ros_widgets;

    Ui::MainWindow *ui;
    std::unique_ptr<RosExecutorThread> node_executor_thread;
};

#endif // MAINWINDOW_H
