cmake_minimum_required(VERSION 3.8)
project(kalman_predict)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(armor_parameter REQUIRED)
find_package(image_transport REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(predict src/kalman_node.cpp)
set_target_properties(predict PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)
ament_target_dependencies(
  predict
  "OpenCV"
  "rclcpp"
  "cv_bridge"
  "sensor_msgs"
  "armor_parameter"
  "image_transport"
)




add_library(kalman_filter SHARED src/kalman_filter.cpp)
target_link_libraries(predict kalman_filter)

get_filename_component(PROJECT_PARENT_DIR "${CMAKE_SOURCE_DIR}/../.." ABSOLUTE)

link_directories (${PROJECT_PARENT_DIR}/install/armor_dectetor/lib)

message("link_directories: ${PROJECT_PARENT_DIR}/install/armor_dectetor/lib")

include_directories( ${PROJECT_PARENT_DIR}/install/armor_dectetor/include)
target_link_libraries (predict  ${PROJECT_PARENT_DIR}/install/armor_dectetor/lib/libarmor_match_lib.so)

install(TARGETS
kalman_filter
predict
DESTINATION lib/${PROJECT_NAME}
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
