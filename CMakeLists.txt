cmake_minimum_required(VERSION 3.8)
project(components_attach_shelf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

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

# Generate custom srv messages
rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/GoToLoading.srv"
	# DEPENDENCIES sensor_msgs geometry_msgs
)

# Declare libraries
include_directories(include)

# pre_approach
add_library(pre_approach SHARED src/pre_approach.cpp)
target_compile_definitions(pre_approach PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(pre_approach
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "rosidl_default_generators"
  )
rclcpp_components_register_nodes(pre_approach "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:pre_approach>\n")

# attach_server
add_library(attach_server SHARED src/attach_server.cpp)
target_compile_definitions(attach_server PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_server
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "nav_msgs"
  "std_msgs"
  "sensor_msgs"
  "tf2"
  "tf2_ros"
  "rosidl_default_generators"
  )
rclcpp_components_register_nodes(attach_server "my_components::AttachServer")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:attach_server>\n")

# Get the typesupport target name
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Link against the generated interface library
# NOTE: CMake, no keywords (PRIVATE, PUBLIC) or all keywords (can't mix)
target_link_libraries(pre_approach ${typesupport_target})
target_link_libraries(attach_server ${typesupport_target})

# Install
install(TARGETS
  pre_approach
  attach_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
