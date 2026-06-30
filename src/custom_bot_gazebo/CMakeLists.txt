cmake_minimum_required(VERSION 3.8)
project(custom_bot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  custom_bot_gazebo/twist_stamped_to_twist.py
  custom_bot_gazebo/tf_static_republisher.py
  custom_bot_gazebo/dynamic_rplidar_tf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
