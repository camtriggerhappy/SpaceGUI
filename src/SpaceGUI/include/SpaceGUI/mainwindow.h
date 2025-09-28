#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGridLayout>
#include "SpaceGUI/controller_read.hpp"
#include "ros_executor_thread.hpp"
#include "SpaceGUI/topic_read.hpp"

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
    std::vector<std::shared_ptr<QMenu>> menu_options;
    std::shared_ptr<JoystickReader> controller_reader;
    std::shared_ptr<TopicReader> topic_reader;
    std::shared_ptr<RosExecutorThread> topic_reader_thread;
    QWidget *floatingArea = nullptr;

    Ui::MainWindow *ui;
    std::unique_ptr<RosExecutorThread> node_executor_thread;
};

#endif // MAINWINDOW_H