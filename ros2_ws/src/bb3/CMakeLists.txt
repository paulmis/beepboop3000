cmake_minimum_required(VERSION 3.5)
project(bb3)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
find_package(pigpio REQUIRED)

# executable
add_executable(bb3 src/main.cpp)
ament_target_dependencies(bb3 rclcpp std_msgs geometry_msgs pigpio)
target_link_libraries(bb3 pigpio "${CMAKE_THREAD_LIBS_INIT}")
install(TARGETS bb3 DESTINATION lib/${PROJECT_NAME})

ament_package()
