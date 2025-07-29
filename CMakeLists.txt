cmake_minimum_required(VERSION 3.5)
project(no_detection_area_module)

# ビルド型とC++標準
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(autoware_auto_mapping_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)

find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_extension REQUIRED)

find_package(Eigen3 REQUIRED)

# 実行ファイル
add_executable(object_exclude_on_lanelet_filter src/object_exclude_on_lanelet.cpp)

ament_target_dependencies(object_exclude_on_lanelet_filter
  rclcpp
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  autoware_auto_mapping_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  lanelet2_core
  lanelet2_io
  lanelet2_projection
  lanelet2_extension
)

# install
install(TARGETS object_exclude_on_lanelet_filter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

