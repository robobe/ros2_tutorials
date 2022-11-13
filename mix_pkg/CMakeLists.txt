cmake_minimum_required(VERSION 3.8)
project(mix_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_pub src/simple_pub.cpp)
ament_target_dependencies(simple_pub rclcpp std_msgs)

# Install Cpp executables
install(TARGETS
  simple_pub
	DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
	mix_pkg/simple_sub.py
	DESTINATION lib/${PROJECT_NAME}
	)

ament_package()
