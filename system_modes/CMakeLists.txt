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
find_package(builtin_interfaces REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_lifecycle REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(Boost 1.58 COMPONENTS program_options REQUIRED)

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

add_library(mode
  src/system_modes/mode.cpp
  src/system_modes/mode_impl.cpp
  src/system_modes/mode_inference.cpp)
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
  "boost_program_options"
  "builtin_interfaces"
)
#rosidl_target_interfaces(mode
# ${PROJECT_NAME} "rosidl_typesupport_cpp")
# TODO Should work with the two lines above, but doesn't
include_directories(../../build/system_modes/rosidl_generator_cpp/)
link_directories(../../build/system_modes/)
target_link_libraries(mode system_modes__rosidl_typesupport_cpp boost_program_options)

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
add_executable(mode-manager src/system_modes/mode_manager.cpp
                            src/mode-manager-node.cpp)
target_include_directories(mode-manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mode-manager mode)
install(TARGETS mode-manager
  DESTINATION bin)

# mode monitor
add_executable(mode-monitor src/system_modes/mode_monitor.cpp
                            src/mode-monitor-node.cpp)
target_include_directories(mode-monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mode-monitor mode)
install(TARGETS mode-monitor
  DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  # the following line skips the linter which checks for copyrights
  # remove the line when a copyright and license is present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # remove the line when this package is a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(MODE_FILE_CORRECT ${CMAKE_CURRENT_SOURCE_DIR}/test/test_modes.yaml)
  set(MODE_FILE_WRONG ${CMAKE_CURRENT_SOURCE_DIR}/test/test_modes_wrong.yaml)
  configure_file(test/modefiles.h.in ${CMAKE_CURRENT_BINARY_DIR}/system_modes/modefiles.h)
  include_directories(BEFORE ${CMAKE_CURRENT_BINARY_DIR})

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
