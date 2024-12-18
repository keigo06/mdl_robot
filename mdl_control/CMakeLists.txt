cmake_minimum_required(VERSION 3.8)
project(mdl_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

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
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    config
  DESTINATION
    share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    scripts/cube.py
    scripts/assembly.py
    scripts/trasition_state.py
    scripts/planner.py
    scripts/server.py
    scripts/cube_visualizer.py
    scripts/simple_cube_publisher.py
    scripts/publisher_member_function.py

  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
