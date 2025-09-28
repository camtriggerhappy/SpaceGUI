#include <cstdio>
#include <QtWidgets>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "visualizer_factory.hpp"
#include "image_visualizer.hpp"
#include "autonomywindow.h"
//#include "ui_autonomywindow.h"
// #include "mainwindow.h"
// #include "ui_mainwindow.h"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);


// register_visualizers.cpp (or in main.cpp init)

    VisualizerFactory::instance().registerCreator("sensor_msgs/msg/Image",
        [](QWidget* parent) -> VisualizerBase* {
            return new ImageVisualizer(parent);
        });


// call register_visualizers() once at startup before creating any visualizer
// e.g. in main(): register_visualizers();

    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));

    app.setPalette(darkPalette);


    QWidget window;
    AUTONOMYWindow w;    
    
    w.show_left_controller();
     //w.show_right_controller();
    w.populate_top_menu();

    w.show(); 

    return app.exec();

    rclcpp::shutdown();
 
}
