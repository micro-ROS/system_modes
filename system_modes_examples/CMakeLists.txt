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
find_package(system_modes REQUIRED)

# drive_base
add_executable(drive_base src/drive_base.cpp)
target_include_directories(drive_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(drive_base
  "system_modes"
)
install(TARGETS drive_base
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# manipulator
add_executable(manipulator src/manipulator.cpp)
target_include_directories(manipulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(manipulator
  "system_modes"
)
install(TARGETS manipulator
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

  # launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# SMH file
install(FILES example_modes.yaml
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

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
endif()

ament_package()
