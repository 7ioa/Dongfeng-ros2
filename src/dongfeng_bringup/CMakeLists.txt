cmake_minimum_required(VERSION 3.8)
project(dongfeng_bringup)

find_package(ament_cmake REQUIRED)
find_package(gz_gui_vendor REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
find_package(gz-gui8 REQUIRED)
find_package(gz-rendering8 REQUIRED COMPONENTS ogre)
find_package(Qt5 REQUIRED COMPONENTS Core Quick)
find_package(PkgConfig REQUIRED)

# Gazebo Harmonic uses the system Ogre 1.9 ABI, not RViz's private Ogre build.
# Prefer the system .pc files even after sourcing a ROS workspace with RViz.
set(saved_pkg_config_path "$ENV{PKG_CONFIG_PATH}")
set(ENV{PKG_CONFIG_PATH} "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/pkgconfig:/usr/share/pkgconfig:$ENV{PKG_CONFIG_PATH}")
pkg_check_modules(OGRE REQUIRED IMPORTED_TARGET
  OGRE OGRE-RTShaderSystem OGRE-Paging OGRE-Terrain OGRE-Overlay)
set(ENV{PKG_CONFIG_PATH} "${saved_pkg_config_path}")

qt5_add_resources(signal_resources gui/TrafficLightMaterialSync.qrc)
add_library(TrafficLightMaterialSync SHARED
  gui/TrafficLightMaterialSync.cc ${signal_resources})
target_compile_features(TrafficLightMaterialSync PRIVATE cxx_std_17)
target_link_libraries(TrafficLightMaterialSync PRIVATE
  gz-gui8::gz-gui8 gz-rendering8::ogre gz-plugin2::register
  PkgConfig::OGRE Qt5::Core Qt5::Quick)
install(TARGETS TrafficLightMaterialSync LIBRARY DESTINATION lib/${PROJECT_NAME})

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
