cmake_minimum_required(VERSION 3.8)
project(SpaceGUI)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<depepndency> REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui)

add_executable(AstroGUI src/autonomywindow.cpp src/AstroGUI.cpp)


target_include_directories(AstroGUI PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  target_link_libraries(AstroGUI 
  Qt6::Core
  Qt6::Widgets
  Qt6::Gui)
target_compile_features(AstroGUI PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

target_sources(AstroGUI PRIVATE
  src/AstroGUI.cpp
  src/mainwindow.ui
  src/mainwindow.cpp
  src/autonomywindow.cpp
)

install(TARGETS AstroGUI
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
