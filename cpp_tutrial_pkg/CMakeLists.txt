cmake_minimum_required(VERSION 3.8)
project(cpp_tutrial_pkg)

# set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# set(SOURCE
#   src/param_demo.cpp
# )
# add_executable(param_demo ${SOURCE})
# ament_target_dependencies(param_demo
#   rclcpp)

include_directories(include/cpp_tutrial_pkg)

set(
  HEADERS
  param_hello.hpp
)

set(
  SOURCES
  src/param_hello_node.cpp
  src/param_hello.cpp
)

# add_library(my_lib ${SOURCES})
# ament_export_targets(my_lib HAS_LIBRARY_TARGET)

add_executable(param_hello ${SOURCES})
ament_target_dependencies(param_hello rclcpp)

install(TARGETS 
  param_hello
  DESTINATION lib/${PROJECT_NAME})
# install(
#   DIRECTORY include/cpp_tutrial_pkg
#   DESTINATION include
# )
# install(
#   TARGETS my_lib
#   EXPORT my_lib
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )
# add_executable(main src/main.cpp)  
# target_link_libraries(main PUBLIC my_lib)

# install(
#   TARGETS main
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(
#   TARGETS param_demo
#   DESTINATION lib/${PROJECT_NAME}
# )

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)

#   set(TESTFILES
#     test/main.cpp
#     test/saving_account_test.cpp
#   )

#   ament_add_gtest(${PROJECT_NAME}_test ${TESTFILES})
#   target_link_libraries(${PROJECT_NAME}_test my_lib)

#   install(TARGETS
#     ${PROJECT_NAME}_test
#     DESTINATION lib/${PROJECT_NAME}
#   )
# endif()
ament_package()
