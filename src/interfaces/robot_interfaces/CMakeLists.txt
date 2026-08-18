cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlcIoState.msg"
  "msg/RollingJointControlState.msg"
  "msg/RollingJointPoint.msg"
  "msg/RollingJointTargetBatch.msg"
  "srv/CloseRollingJointSession.srv"
  "srv/OpenRollingJointSession.srv"
  "srv/RtEnable.srv"
  "srv/SetJointControlMode.srv"
  DEPENDENCIES builtin_interfaces std_msgs unique_identifier_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_rolling_joint_schema
    test/test_rolling_joint_schema.cpp
  )
  target_compile_features(test_rolling_joint_schema PUBLIC cxx_std_17)
  target_compile_options(test_rolling_joint_schema PRIVATE -Wall -Wextra -Wpedantic)
  ament_target_dependencies(test_rolling_joint_schema unique_identifier_msgs)
  rosidl_get_typesupport_target(
    cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
  target_link_libraries(test_rolling_joint_schema "${cpp_typesupport_target}")
endif()

ament_package()
