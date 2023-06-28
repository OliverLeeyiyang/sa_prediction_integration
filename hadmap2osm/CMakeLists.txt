cmake_minimum_required(VERSION 3.8)
project(hadmap2osm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_auto_mapping_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lanelet2_io REQUIRED)


include_directories(
  include
  autoware
  ${catkin_include_dirs}
)
set(Module_name hadmap2osm)

add_executable(${Module_name} src/2osm.cpp)
ament_target_dependencies(${Module_name} rclcpp lanelet2_io autoware_auto_mapping_msgs)

install(TARGETS
  ${Module_name}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
