cmake_minimum_required(VERSION 3.8)
project(armor_dectetor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(armor_parameter REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

add_executable(armor_node src/armor_node.cpp)
set_target_properties(armor_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
)
ament_target_dependencies(
armor_node
"OpenCV"
"cv_bridge"
"rclcpp"
"sensor_msgs"  
"tf2"
"armor_parameter"
)

add_library(armor_match_lib SHARED src/armor_match.cpp)
target_link_libraries(armor_node armor_match_lib)

add_library(pnp_solver_lib SHARED src/pnp_solver.cpp)
target_link_libraries(armor_node pnp_solver_lib)


install(TARGETS
armor_match_lib  # 添加动态库安装
pnp_solver_lib   # 添加动态库安装
armor_node
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
