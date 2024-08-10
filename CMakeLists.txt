cmake_minimum_required(VERSION 3.8)
project(smart_rabo)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

find_package(micro_ros_agent REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include Cpp "include" directory
include_directories(include)


add_executable(hamester_info src/hamester_info.cpp)


target_include_directories(hamester_info PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hamester_info PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS hamester_info
  DESTINATION lib/${PROJECT_NAME})


# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/keyBoard_node.py  
  scripts/controller_node.py
  scripts/odometry_node.py 
  scripts/maze_solve.py  
  scripts/odom_rviz.py      
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files.
install(
  DIRECTORY  launch urdf rviz
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
