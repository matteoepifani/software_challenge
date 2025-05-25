cmake_minimum_required(VERSION 3.8)
project(software_training)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Reset.srv"
  "msg/Distance.msg"
  "action/MovingTurtle.action"
)
ament_export_dependencies(rosidl_default_runtime)

# ---- clear_turtle ----
add_library(clear_turtle SHARED
  src/clear_turtle.cpp)
target_include_directories(clear_turtle PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(clear_turtle
  PRIVATE "SOFTWARE_TRAINING_CPP_BUILDING_DLL")
ament_target_dependencies(clear_turtle
  "std_srvs"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_node(clear_turtle PLUGIN "software_training::ClearTurtle" EXECUTABLE clear_turtle_node)
install(TARGETS
  clear_turtle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ---- turtle1_circle ----
add_library(turtle1_circle SHARED
  src/turtle1_circle.cpp)
target_include_directories(turtle1_circle PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(turtle1_circle
  PRIVATE "SOFTWARE_TRAINING_CPP_BUILDING_DLL")
ament_target_dependencies(turtle1_circle
  "geometry_msgs"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_node(turtle1_circle PLUGIN "software_training::Turtle1Circle" EXECUTABLE turtle1_circle_node)
install(TARGETS
  turtle1_circle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ---- spawn_turtle ----
add_library(spawn_turtle SHARED
  src/spawn_turtle.cpp)
target_include_directories(spawn_turtle PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(spawn_turtle
  PRIVATE "SOFTWARE_TRAINING_CPP_BUILDING_DLL")
ament_target_dependencies(spawn_turtle
  "turtlesim"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_node(spawn_turtle PLUGIN "software_training::SpawnTurtle" EXECUTABLE spawn_turtle_node)
install(TARGETS
  spawn_turtle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ---- reset_moving_turtle ----
add_library(reset_moving_turtle SHARED
  src/reset_moving_turtle.cpp)
target_include_directories(reset_moving_turtle PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(reset_moving_turtle
  PRIVATE "SOFTWARE_TRAINING_CPP_BUILDING_DLL")
ament_target_dependencies(reset_moving_turtle
  "turtlesim"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_node(reset_moving_turtle PLUGIN "software_training::ResetMovingTurtle" EXECUTABLE reset_moving_turtle_node)
install(TARGETS
  reset_moving_turtle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
rosidl_target_interfaces(reset_moving_turtle
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

  # ---- reset_moving_turtle_client ----
add_library(reset_moving_turtle_client SHARED
  src/reset_moving_turtle_client.cpp)
target_include_directories(reset_moving_turtle_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(reset_moving_turtle_client
  PRIVATE "SOFTWARE_TRAINING_CPP_BUILDING_DLL")
ament_target_dependencies(reset_moving_turtle_client
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_node(reset_moving_turtle_client PLUGIN "software_training::ResetMovingTurtleClient" EXECUTABLE reset_moving_turtle_client_node)
install(TARGETS
  reset_moving_turtle_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
rosidl_target_interfaces(reset_moving_turtle_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---- distance_publisher ----
add_library(distance_publisher SHARED
  src/distance_publisher.cpp)
target_include_directories(distance_publisher PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(distance_publisher
  PRIVATE "SOFTWARE_TRAINING_CPP_BUILDING_DLL")
ament_target_dependencies(distance_publisher
  "turtlesim"
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_node(distance_publisher PLUGIN "software_training::DistancePublisher" EXECUTABLE distance_publisher_node)
install(TARGETS
  distance_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
rosidl_target_interfaces(distance_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---- distance_subscriber ----
add_library(distance_subscriber SHARED
  src/distance_subscriber.cpp)
target_include_directories(distance_subscriber PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(distance_subscriber
  PRIVATE "SOFTWARE_TRAINING_CPP_BUILDING_DLL")
ament_target_dependencies(distance_subscriber
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_node(distance_subscriber PLUGIN "software_training::DistanceSubscriber" EXECUTABLE distance_subscriber_node)
install(TARGETS
  distance_subscriber
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
rosidl_target_interfaces(distance_subscriber
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---- moving_turtle_server ----
add_library(moving_turtle_server SHARED
  src/moving_turtle_server.cpp)
target_include_directories(moving_turtle_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(moving_turtle_server
  PRIVATE "SOFTWARE_TRAINING_CPP_BUILDING_DLL")
ament_target_dependencies(moving_turtle_server
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "turtlesim"
  "rclcpp_components")
rclcpp_components_register_node(moving_turtle_server PLUGIN "software_training::MovingTurtleServer" EXECUTABLE moving_turtle_server_node)
install(TARGETS
  moving_turtle_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
rosidl_target_interfaces(moving_turtle_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ---- moving_turtle_client ----
add_library(moving_turtle_client SHARED
  src/moving_turtle_client.cpp)
target_include_directories(moving_turtle_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(moving_turtle_client
  PRIVATE "SOFTWARE_TRAINING_CPP_BUILDING_DLL")
ament_target_dependencies(moving_turtle_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(moving_turtle_client PLUGIN "software_training::MovingTurtleClient" EXECUTABLE moving_turtle_client_node)
install(TARGETS
  moving_turtle_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
rosidl_target_interfaces(moving_turtle_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
