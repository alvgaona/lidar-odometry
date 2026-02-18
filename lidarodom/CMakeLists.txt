cmake_minimum_required(VERSION 3.10)
project(lidarodom)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mocap4r2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development)

add_executable(kiss_converter src/kiss_converter.cpp)
target_include_directories(kiss_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(kiss_converter Python3::Python)
ament_target_dependencies(kiss_converter
  rclcpp
  nav_msgs
  geometry_msgs
  mocap4r2_msgs
  tf2_ros
)

add_executable(glim_converter src/glim_converter.cpp)
target_include_directories(glim_converter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(glim_converter Python3::Python)
ament_target_dependencies(glim_converter
  rclcpp
  nav_msgs
  geometry_msgs
  mocap4r2_msgs
  tf2_ros
)

install(TARGETS kiss_converter glim_converter
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
