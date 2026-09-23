cmake_minimum_required(VERSION 3.8)
project(lab1_pkg)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  lab1_pkg/talker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME talker
)

install(PROGRAMS
  lab1_pkg/relay.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME relay
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
