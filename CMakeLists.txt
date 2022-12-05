cmake_minimum_required(VERSION 3.8)
project(cpp_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


find_package(rosidl_default_generators REQUIRED)

include_directories(
  include
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ModifyString.srv"
 )
# add_subdirectory(src)
# add_subdirectory(test)
add_library(pub_lib src/publisher_member_function.cpp)

ament_target_dependencies(pub_lib rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pub_lib "${cpp_typesupport_target}")

add_executable(talker src/publisher_member_function.cpp)
target_link_libraries(talker pub_lib)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(
  TARGETS pub_lib
  EXPORT pub_lib
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(TESTFILES test/main.cpp)
  ament_add_gtest(pub_test ${TESTFILES})
  target_link_libraries(pub_test pub_lib)
  ament_target_dependencies(pub_test rclcpp geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
  install(
    TARGETS pub_test
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
