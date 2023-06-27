cmake_minimum_required(VERSION 3.5)
project(robomaster_msgs)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WheelSpeeds.msg"
  "msg/LEDEffect.msg"
  "msg/ChassisStatus.msg"
  "msg/SBus.msg"
  "msg/PWM.msg"
  "msg/GripperState.msg"
  "msg/CameraConfig.msg"
  "msg/ArmorHit.msg"
  "msg/DetectedLine.msg"
  "msg/DetectedGesture.msg"
  "msg/DetectedMarker.msg"
  "msg/DetectedPerson.msg"
  "msg/DetectedRobot.msg"
  "msg/Detection.msg"
  "msg/RegionOfInterest.msg"
  "msg/Serial.msg"
  "msg/ServoCommand.msg"
  "msg/GimbalCommand.msg"
  "msg/BlasterLED.msg"
  "msg/Mode.msg"
  "msg/AudioData.msg"
  "msg/AudioLevel.msg"
  "msg/AudioOpus.msg"
  "msg/H264Packet.msg"
  "msg/SensorAdapter.msg"
  "msg/ServoRawState.msg"
  "msg/SpeakerCommand.msg"
  "action/Move.action"
  "action/MoveArm.action"
  "action/GripperControl.action"
  "action/PlaySound.action"
  "action/MoveServo.action"
  "action/MoveGimbal.action"
  "action/RecenterGimbal.action"
  "srv/GetADC.srv"
  "srv/GetIO.srv"
  "srv/GetPulse.srv"
  "srv/GetServoAngle.srv"
  DEPENDENCIES std_msgs
 )

ament_package()
