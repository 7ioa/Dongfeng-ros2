cmake_minimum_required(VERSION 3.8)
project(dongfeng_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME} RENAME teleop_keyboard)
install(PROGRAMS scripts/command_guard.py
  DESTINATION lib/${PROJECT_NAME} RENAME command_guard)
install(FILES scripts/command_logic.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(control_logic test/test_control.py)
endif()

ament_package()
