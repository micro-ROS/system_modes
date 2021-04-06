cmake_minimum_required(VERSION 3.5)
project(system_modes)

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
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Mode.msg"
  "msg/ModeEvent.msg"
  "srv/ChangeMode.srv"
  "srv/GetMode.srv"
  "srv/GetAvailableModes.srv"
  ADD_LINTER_TESTS
  DEPENDENCIES builtin_interfaces
)

add_library(mode SHARED
  src/system_modes/mode.cpp
  src/system_modes/mode_impl.cpp
  src/system_modes/mode_handling.cpp
  src/system_modes/mode_inference.cpp
  src/system_modes/mode_observer.cpp)
target_include_directories(mode PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mode
  "rclcpp"
  "rcl_lifecycle"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "rosidl_typesupport_cpp"
  "std_msgs"
  "builtin_interfaces"
)
#rosidl_target_interfaces(mode
# ${PROJECT_NAME} "rosidl_typesupport_cpp")
# TODO Should work with the two lines above, but doesn't
include_directories(../../build/system_modes/rosidl_generator_cpp/)
link_directories(../../build/system_modes/)
target_link_libraries(mode system_modes__rosidl_typesupport_cpp)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(mode PRIVATE "SYSTEMMODES_BUILDING_LIBRARY")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS mode
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# mode manager
add_executable(mode_manager src/system_modes/mode_manager.cpp
                            src/mode_manager_node.cpp)
target_include_directories(mode_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mode_manager mode)
install(TARGETS mode_manager
  DESTINATION lib/${PROJECT_NAME})

# mode monitor
add_executable(mode_monitor src/system_modes/mode_monitor.cpp
                            src/mode_monitor_node.cpp)
target_include_directories(mode_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mode_monitor mode)
install(TARGETS mode_monitor
  DESTINATION lib/${PROJECT_NAME})

  # launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(MODE_FILE_CORRECT ${CMAKE_CURRENT_SOURCE_DIR}/test/test_modes.yaml)
  set(MODE_FILE_RULES ${CMAKE_CURRENT_SOURCE_DIR}/test/test_modes_rules.yaml)
  set(MODE_FILE_WRONG ${CMAKE_CURRENT_SOURCE_DIR}/test/test_modes_wrong.yaml)
  configure_file(test/modefiles.h.in ${CMAKE_CURRENT_BINARY_DIR}/system_modes/modefiles.h)
  include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR})

  # Unit Tests
  ament_add_gtest(test_state_and_mode_struct test/test_state_and_mode_struct.cpp)
  if(TARGET test_state_and_mode_struct)
    target_include_directories(test_state_and_mode_struct PUBLIC
      ${rclcpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_state_and_mode_struct mode)
  endif()

  ament_add_gtest(test_default_mode test/test_default_mode.cpp)
  if(TARGET test_default_mode)
    target_include_directories(test_default_mode PUBLIC
      ${rclcpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_default_mode mode)
  endif()

  ament_add_gtest(test_mode test/test_mode.cpp)
  if(TARGET test_mode)
    target_include_directories(test_mode PUBLIC
      ${rclcpp_INCLUDE_DIRS}
    )
    target_link_libraries(test_mode mode)
  endif()

  ament_add_gtest(test_mode_inference test/test_mode_inference.cpp)
  if(TARGET test_mode_inference)
    target_include_directories(test_mode_inference PUBLIC
      ${rclcpp_INCLUDE_DIRS}
      ${CMAKE_CURRENT_BINARY_DIR}/system_modes/
    )
    target_link_libraries(test_mode_inference mode)
  endif()

  ament_add_gtest(test_mode_handling test/test_mode_handling.cpp)
  if(TARGET test_mode_handling)
    target_include_directories(test_mode_handling PUBLIC
      ${rclcpp_INCLUDE_DIRS}
      ${CMAKE_CURRENT_BINARY_DIR}/system_modes/
    )
    target_link_libraries(test_mode_handling mode)
  endif()

  ament_add_gtest(test_mode_manager
    test/test_mode_manager.cpp
    src/system_modes/mode_manager.cpp)
  if(TARGET test_mode_manager)
    target_include_directories(test_mode_manager PUBLIC
      ${rclcpp_INCLUDE_DIRS}
      ${CMAKE_CURRENT_BINARY_DIR}/system_modes/
    )
    target_link_libraries(test_mode_manager mode)
  endif()

  ament_add_gtest(test_mode_monitor
    test/test_mode_monitor.cpp
    src/system_modes/mode_monitor.cpp)
  if(TARGET test_mode_monitor)
    target_include_directories(test_mode_monitor PUBLIC
      ${rclcpp_INCLUDE_DIRS}
      ${CMAKE_CURRENT_BINARY_DIR}/system_modes/
    )
    target_link_libraries(test_mode_monitor mode)
  endif()

  ament_add_gtest(test_mode_observer test/test_mode_observer.cpp)
  if(TARGET test_mode_observer)
    target_include_directories(test_mode_observer PUBLIC
      ${rclcpp_INCLUDE_DIRS}
      ${CMAKE_CURRENT_BINARY_DIR}/system_modes/
    )
    target_link_libraries(test_mode_observer mode)
  endif()

  # mode observer test node
  add_executable(modes_observer_test_node test/launchtest/modes_observer_test_node.cpp)
  target_include_directories(modes_observer_test_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  target_link_libraries(modes_observer_test_node mode)
  install(TARGETS modes_observer_test_node DESTINATION lib/${PROJECT_NAME})

  # Launch Tests
  find_package(launch_testing_ament_cmake REQUIRED)
  set(launch_tests
    "two_lifecycle_nodes"           # Mode Manager with Lifecycle Nodes
    "two_mixed_nodes"               # Mode Manager with Lifecycle and non-Lifecycle Nodes
    "two_independent_hierarchies"   # Mode Manager for two independent hierarchies of nodes
    "manager_and_monitor"           # Mode Manager and Mode Monitor
    "redundant_mode_changes"        # Ignore redundant mode changes
    "modes_observer")               # Mode Observer

  # Launch Test: Mode Manager with Lifecycle Nodes
  foreach(test_name ${launch_tests})
    file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/launchtest/${test_name}.py" TEST_NODES)
    file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/launchtest/${test_name}_modes.yaml" MODELFILE)
    file(TO_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/launchtest/${test_name}_expected_output" EXPECTED_OUTPUT)
    configure_file(
        "test/launchtest/${test_name}.launch.py.in"
        "test/${test_name}.launch.py"
        @ONLY
      )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test/${test_name}.launch.py"
      TARGET "${test_name}"
      TIMEOUT 30
      ENV
    )
  endforeach()
endif()

ament_export_include_directories(include)
#ament_export_interfaces(export_${PROJECT_NAME})
ament_export_libraries(mode)
ament_export_libraries(${PROJECT_NAME}__rosidl_typesupport_introspection_cpp)
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(lifecycle_msgs)
ament_export_dependencies(std_msgs)
ament_package()
