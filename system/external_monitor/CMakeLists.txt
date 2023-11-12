cmake_minimum_required(VERSION 3.14)
project(external_monitor)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(external_monitor
  src/external_monitor/external_monitor_node.cpp
  src/external_monitor/external_monitor_core.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)