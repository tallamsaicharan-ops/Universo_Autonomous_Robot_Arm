cmake_minimum_required(VERSION 3.8)
project(universo_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/xbox_servo.py
    scripts/servo_starter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
