cmake_minimum_required(VERSION 3.8)
project(enable_manager)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

add_library(enable_manager_controller SHARED
  src/enable_manager_controller.cpp
)
target_compile_features(enable_manager_controller PUBLIC cxx_std_17)
target_compile_options(enable_manager_controller PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
target_include_directories(enable_manager_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(enable_manager_controller
  controller_interface
  controller_manager_msgs
  diagnostic_msgs
  pluginlib
  rclcpp
  robot_interfaces
)

add_executable(rt_disable_once src/rt_disable_once.cpp)
target_compile_features(rt_disable_once PUBLIC cxx_std_17)
target_compile_options(rt_disable_once PRIVATE -Wall -Wextra -Wpedantic -Wconversion)
ament_target_dependencies(
  rt_disable_once controller_manager_msgs lifecycle_msgs rclcpp robot_interfaces
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(hardware_interface REQUIRED)

  ament_add_gtest(test_enable_manager_characterization
    test/test_enable_manager_characterization.cpp
  )
  target_compile_features(test_enable_manager_characterization PUBLIC cxx_std_17)
  target_compile_options(
    test_enable_manager_characterization PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_link_libraries(
    test_enable_manager_characterization enable_manager_controller
  )
  ament_target_dependencies(
    test_enable_manager_characterization
    controller_manager_msgs
    hardware_interface
    rclcpp
    robot_interfaces
  )
endif()

pluginlib_export_plugin_description_file(
  controller_interface enable_manager_plugins.xml
)

install(TARGETS enable_manager_controller
  EXPORT export_enable_manager_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS rt_disable_once DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES enable_manager_plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_enable_manager_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface controller_manager_msgs diagnostic_msgs lifecycle_msgs pluginlib rclcpp
  robot_interfaces
)

ament_package()
