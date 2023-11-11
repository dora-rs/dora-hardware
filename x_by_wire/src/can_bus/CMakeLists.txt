cmake_minimum_required(VERSION 3.5)
project(can_bus)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  ${${PROJECT_NAME}_BUILD_DEPENDS}
  ${${PROJECT_NAME}_BUILDTOOL_DEPENDS}  
)

set(CAN_BUS_SRC
  src/can_bus_node.cpp
  src/can_bus.cpp
  src/socket_can.cpp  
)

add_executable(can_bus
  src/can_bus_node.cpp
  src/can_bus.cpp
  src/socket_can.cpp
)
target_link_libraries(can_bus ${catkin_LIBRARIES})
ament_target_dependencies(can_bus rclcpp msg_interfaces)

install(TARGETS can_bus
  DESTINATION lib/${PROJECT_NAME}    
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
