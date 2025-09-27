cmake_minimum_required(VERSION 3.8)
project(SpaceGUI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui)

add_executable(AstroGUI
  src/AstroGUI.cpp
  src/mainwindow.cpp
  src/nodes/controller_read.cpp
  src/ros_executor_thread.cpp
  src/mainwindow.ui
  src/nodes/topic_read.cpp
  src/ros_data_widget.cpp
)

target_include_directories(AstroGUI PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(AstroGUI
  Qt6::Core
  Qt6::Widgets
  Qt6::Gui
)

ament_target_dependencies(AstroGUI
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

install(TARGETS AstroGUI DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
