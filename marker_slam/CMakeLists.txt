cmake_minimum_required(VERSION 3.5)
project(marker_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-function -Wno-deprecated-declarations -Wno-format)


# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fiducial_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED 
                   COMPONENTS filesystem system)
# find_package(image_transport REQUIRED)
# find_package(cv_bridge REQUIRED)

set(TARGET1 marker_slam)

add_executable(${TARGET1} src/fiducial_slam.cpp
                          src/map.cpp
                          src/transform_with_variance.cpp)

target_include_directories(${TARGET1} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
# Add dependent msgs to project
ament_target_dependencies(${TARGET1} 
                          rclcpp
                          std_msgs
                          sensor_msgs
                          fiducial_msgs
                          tf2_ros
                          visualization_msgs
                          tf2_geometry_msgs
                          )

target_link_libraries(${TARGET1} Boost::filesystem Boost::system)

install(TARGETS ${TARGET1}
DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
