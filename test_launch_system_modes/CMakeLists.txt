cmake_minimum_required(VERSION 3.5)
project(test_launch_system_modes)

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)
  file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/test_modes.yaml" MODELFILE)
  file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/expected_output" EXPECTED_OUTPUT)

  # Test launch actions, events, and event handlers with a node
  configure_file(
      "test/node_test.launch.py.in"
      "test/node_test.launch.py"
      @ONLY
    )
  add_launch_test(
    "${CMAKE_CURRENT_BINARY_DIR}/test/node_test.launch.py"
    TARGET "node_test"
    TIMEOUT 30
    ENV)

  # Test launch actions, events, and event handlers with a system
  configure_file(
      "test/system_test.launch.py.in"
      "test/system_test.launch.py"
      @ONLY
    )
  add_launch_test(
    "${CMAKE_CURRENT_BINARY_DIR}/test/system_test.launch.py"
    TARGET "system_test"
    TIMEOUT 30
    ENV)
endif()

ament_export_include_directories(include)
ament_export_libraries(mode)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(lifecycle_msgs)
ament_export_dependencies(system_modes_msgs)
ament_package()
