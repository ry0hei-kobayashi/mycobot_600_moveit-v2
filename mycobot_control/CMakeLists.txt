cmake_minimum_required(VERSION 3.16)
project(mycobot_control LANGUAGES CXX)

# Enable additional compiler warnings for GCC/Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# Dependencies for hardware interface
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)

foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Library for mycobot_control
add_library(
  mycobot_control 
  SHARED
  src/mycobot_hardware.cpp
)

# Set C++ standard
target_compile_features(mycobot_control PUBLIC cxx_std_17)

# Include directories
target_include_directories(
  mycobot_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mycobot_control>
)

# Link dependencies
ament_target_dependencies(
  mycobot_control PUBLIC
  ${HW_IF_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface mycobot_ros2_control.xml)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/mycobot_control
)

# Install library
install(
  TARGETS mycobot_control
  EXPORT export_mycobot_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Export targets and dependencies
ament_export_targets(export_mycobot_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${HW_IF_INCLUDE_DEPENDS})

# Finalize the package
ament_package()

