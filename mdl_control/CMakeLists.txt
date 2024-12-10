cmake_minimum_required(VERSION 3.8)
project(mdl_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

install(
    PROGRAMS
    src/cube_node.py       # CubeNodeモジュール
    src/cube.py            # Cubeモジュール
    src/assembly.py        # Assemblyモジュール
    src/trasition_state.py # Stateモジュール
    src/planner.py         # Plannerモジュール
    src/server.py          # Serverモジュール
    DESTINATION lib/${PROJECT_NAME} # インストール先をlib/{パッケージ名}に指定
)

# launchファイルのインストール設定
# launchディレクトリ内の全てのファイルをインストールします
# これにより、ROS 2のlaunchシステムでlaunchファイルが見つけられるようになります
install(
    DIRECTORY launch                # launchディレクトリ内のすべてのファイルをインストール
    DESTINATION share/${PROJECT_NAME} # share/{パッケージ名}にインストール
)

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
