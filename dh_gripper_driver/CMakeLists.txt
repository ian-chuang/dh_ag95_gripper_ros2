cmake_minimum_required(VERSION 3.8)
project(dh_gripper_driver)

include(GNUInstallDirs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

add_library(
  dh_gripper_driver
  SHARED
  src/hardware_interface.cpp
  src/dh_ag95_can.cpp
  src/dh_device.cpp
  src/dh_dh3_can.cpp
  src/dh_lagacy_gripper.cpp
  src/dh_modbus_gripper.cpp
  src/dh_rgi.cpp
)
target_link_libraries(dh_gripper_driver atomic)
target_include_directories(
  dh_gripper_driver
  PRIVATE
  include
)
ament_target_dependencies(
  dh_gripper_driver
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# INSTALL
install(
  TARGETS dh_gripper_driver
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)


# EXPORTS

ament_export_include_directories(
  include
)
ament_export_libraries(
  dh_gripper_driver
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# INSTALL

install(DIRECTORY config launch urdf rviz 
  DESTINATION share/${PROJECT_NAME}
)

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

ament_package()
