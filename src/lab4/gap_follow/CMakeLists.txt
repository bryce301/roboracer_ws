cmake_minimum_required(VERSION 3.5)
project(gap_follow)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# The Python controller is also available under the default executable name.
install(PROGRAMS
  scripts/reactive_node.py
  RENAME reactive_node
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/reactive_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files: the autograder starts your nodes with them
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The untouched C++ starter remains as a reference; this package submits
  # the Python controller installed above.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
