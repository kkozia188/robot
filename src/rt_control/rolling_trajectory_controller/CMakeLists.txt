cmake_minimum_required(VERSION 3.8)
project(rolling_trajectory_controller)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

add_library(rolling_trajectory_core
  src/cubic_hermite.cpp
  src/limit_checker.cpp
  src/rolling_buffer.cpp
  src/rolling_snapshot.cpp
  src/session_core.cpp
)
target_compile_features(rolling_trajectory_core PUBLIC cxx_std_17)
target_compile_options(
  rolling_trajectory_core PRIVATE -Wall -Wextra -Wpedantic -Wconversion
)
target_include_directories(rolling_trajectory_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(rolling_trajectory_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(rolling_trajectory_controller_plugin SHARED
  src/rolling_trajectory_controller.cpp
)
target_compile_features(rolling_trajectory_controller_plugin PUBLIC cxx_std_17)
target_compile_options(
  rolling_trajectory_controller_plugin PRIVATE -Wall -Wextra -Wpedantic -Wconversion
)
target_include_directories(rolling_trajectory_controller_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rolling_trajectory_controller_plugin rolling_trajectory_core)
ament_target_dependencies(
  rolling_trajectory_controller_plugin
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  robot_interfaces
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  ament_add_gtest(test_rolling_buffer
    test/test_rolling_buffer.cpp
  )
  target_compile_features(test_rolling_buffer PUBLIC cxx_std_17)
  target_compile_options(
    test_rolling_buffer PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_rolling_buffer PRIVATE include)
  target_link_libraries(test_rolling_buffer rolling_trajectory_core)

  ament_add_gtest(test_protocol_vectors
    test/test_protocol_vectors.cpp
  )
  target_compile_features(test_protocol_vectors PUBLIC cxx_std_17)
  target_compile_options(
    test_protocol_vectors PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_protocol_vectors PRIVATE include)
  target_link_libraries(test_protocol_vectors rolling_trajectory_core)

  ament_add_gtest(test_interpolation_and_limits
    test/test_interpolation_and_limits.cpp
  )
  target_compile_features(test_interpolation_and_limits PUBLIC cxx_std_17)
  target_compile_options(
    test_interpolation_and_limits PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_interpolation_and_limits PRIVATE include)
  target_link_libraries(test_interpolation_and_limits rolling_trajectory_core)

  ament_add_gtest(test_stopping_envelope
    test/test_stopping_envelope.cpp
  )
  target_compile_features(test_stopping_envelope PUBLIC cxx_std_17)
  target_compile_options(
    test_stopping_envelope PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_stopping_envelope PRIVATE include)
  target_link_libraries(test_stopping_envelope rolling_trajectory_core)

  ament_add_gtest(test_rolling_snapshot
    test/test_rolling_snapshot.cpp
  )
  target_compile_features(test_rolling_snapshot PUBLIC cxx_std_17)
  target_compile_options(
    test_rolling_snapshot PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_rolling_snapshot PRIVATE include)
  target_link_libraries(test_rolling_snapshot rolling_trajectory_core)

  ament_add_gtest(test_controller_lifecycle
    test/test_controller_lifecycle.cpp
  )
  target_compile_features(test_controller_lifecycle PUBLIC cxx_std_17)
  target_compile_options(
    test_controller_lifecycle PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_controller_lifecycle PRIVATE include)
  ament_target_dependencies(
    test_controller_lifecycle controller_interface hardware_interface rclcpp
  )
  target_link_libraries(
    test_controller_lifecycle rolling_trajectory_controller_plugin
  )

  ament_add_gtest(test_controller_switch
    test/test_controller_switch.cpp
  )
  target_compile_features(test_controller_switch PUBLIC cxx_std_17)
  target_compile_options(
    test_controller_switch PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_controller_switch PRIVATE include)
  ament_target_dependencies(
    test_controller_switch
    controller_interface
    controller_manager
    controller_manager_msgs
    hardware_interface
    pluginlib
    rclcpp
  )
  target_link_libraries(
    test_controller_switch rolling_trajectory_controller_plugin
  )

  ament_add_gtest(test_controller_callbacks
    test/test_controller_callbacks.cpp
  )
  target_compile_features(test_controller_callbacks PUBLIC cxx_std_17)
  target_compile_options(
    test_controller_callbacks PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_controller_callbacks PRIVATE include)
  ament_target_dependencies(
    test_controller_callbacks
    controller_interface
    hardware_interface
    rclcpp
    robot_interfaces
  )
  target_link_libraries(
    test_controller_callbacks
    rolling_trajectory_controller_plugin
    rolling_trajectory_core
  )

  ament_add_gtest(test_rt_update
    test/test_rt_update.cpp
  )
  target_compile_features(test_rt_update PUBLIC cxx_std_17)
  target_compile_options(
    test_rt_update PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_rt_update PRIVATE include)
  ament_target_dependencies(
    test_rt_update
    controller_interface
    hardware_interface
    rclcpp
    robot_interfaces
  )
  target_link_libraries(
    test_rt_update
    rolling_trajectory_controller_plugin
    rolling_trajectory_core
  )

  ament_add_gtest(test_state_publisher
    test/test_state_publisher.cpp
  )
  target_compile_features(test_state_publisher PUBLIC cxx_std_17)
  target_compile_options(
    test_state_publisher PRIVATE -Wall -Wextra -Wpedantic -Wconversion
  )
  target_include_directories(test_state_publisher PRIVATE include)
  ament_target_dependencies(
    test_state_publisher
    controller_interface
    hardware_interface
    rclcpp
    robot_interfaces
  )
  target_link_libraries(
    test_state_publisher
    rolling_trajectory_controller_plugin
    rolling_trajectory_core
  )
endif()

pluginlib_export_plugin_description_file(
  controller_interface rolling_trajectory_controller_plugins.xml
)

install(
  TARGETS rolling_trajectory_core rolling_trajectory_controller_plugin
  EXPORT export_rolling_trajectory_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(
  FILES rolling_trajectory_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_rolling_trajectory_core HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface hardware_interface pluginlib rclcpp robot_interfaces
)

ament_package()
