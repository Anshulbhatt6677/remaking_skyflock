cmake_minimum_required(VERSION 3.8)
project(my_digirc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Command.srv"
)
install(
  PROGRAMS
    src/swarm_controller.py
    src/swarm_manager.py
    src/swarm_orchestrator.py
    src/swarm_keyboard_controller.py
    src/swarm_telemetry_logger.py
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY src/manager
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY src/hal
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
