cmake_minimum_required(VERSION 3.8)
project(picking_cell_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveJ.srv"
  "srv/MoveL.srv"
  "srv/MoveJB.srv"
  "srv/MoveLB.srv"
  "srv/MoveT.srv"
  "srv/GripperWidth.srv"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
