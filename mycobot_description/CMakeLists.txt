cmake_minimum_required(VERSION 3.5)
project(mycobot_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
