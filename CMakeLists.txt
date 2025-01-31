cmake_minimum_required(VERSION 3.8)
project(tutorial_ros_podstawy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


# LIBRARIES
add_library(fake_sensor_utils src/fake_sensor_utils.cpp)
add_library(tutorial_ros_podstawy::fake_sensor_utils ALIAS fake_sensor_utils)
target_compile_features(fake_sensor_utils PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(fake_sensor_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(
  TARGETS fake_sensor_utils
  EXPORT export_fake_sensor_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# EXECUTABLES


add_executable(minimal_node src/minimal_node.cpp)
target_include_directories(minimal_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(minimal_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  minimal_node
  rclcpp
)

# SMALL PUBLISHER
add_executable(small_publisher src/small_publisher.cpp)
target_include_directories(small_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(small_publisher PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  small_publisher
  rclcpp
  std_msgs
)

# SMALL SUBSCRIBER
add_executable(small_subscriber src/small_subscriber.cpp)
target_include_directories(small_subscriber PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(small_subscriber PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  small_subscriber
  rclcpp
  std_msgs
)

# FAKE LIDAR
add_executable(fake_lidar src/fake_lidar.cpp src/fake_lidar_config.cpp)
target_include_directories(fake_lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(fake_lidar PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(fake_lidar fake_sensor_utils)
ament_target_dependencies(
  fake_lidar
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS minimal_node small_publisher small_subscriber fake_lidar
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
