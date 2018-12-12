cmake_minimum_required(VERSION 3.5)
project(system_modes_examples)

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
find_package(rcl_lifecycle REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(Boost 1.58 COMPONENTS program_options REQUIRED)
find_package(system_modes REQUIRED)

# drive_base
add_executable(drive_base src/drive_base.cpp)
target_include_directories(drive_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(drive_base
  "rclcpp"
  "rcl_lifecycle"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "rosidl_typesupport_cpp"
  "std_msgs"
  "boost_program_options"
  "system_modes"
)
install(TARGETS drive_base
  EXPORT export_${PROJECT_NAME}
  DESTINATION bin)

# manipulator
add_executable(manipulator src/manipulator.cpp)
target_include_directories(manipulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(manipulator
  "rclcpp"
  "rcl_lifecycle"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "rosidl_typesupport_cpp"
  "std_msgs"
  "boost_program_options"
  "system_modes"
)
install(TARGETS manipulator
  EXPORT export_${PROJECT_NAME}
  DESTINATION bin)

# mode file
install(FILES example_modes.yaml
  DESTINATION share
)

ament_package()
