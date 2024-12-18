cmake_minimum_required(VERSION 3.8)
project(lbr_intel_camera_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_typesupport_c REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(sensor_msgs)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(msg_files
  "msg/CalibrationData.msg"
)

set(srv_files
  "srv/MonoCalibration.srv"
  "srv/StereoCalibration.srv"
  "srv/PatternCalibration.srv"
  "srv/ChangeProfile.srv"
  "srv/ChangeState.srv"
  "srv/IntelCameraInformation.srv"
  "srv/ChoiceCamera.srv"
)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_files}
  ${srv_files}

  DEPENDENCIES sensor_msgs
)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
