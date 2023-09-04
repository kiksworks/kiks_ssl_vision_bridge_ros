cmake_minimum_required(VERSION 3.5)
project(kiks_ssl_vision_bridge)

# set and find packages
set(ros2_required_packages
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros)

foreach(ros2_required_package IN LISTS ros2_required_packages)
  find_package(${ros2_required_package} REQUIRED)
endforeach()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Qt6 COMPONENTS Network REQUIRED)

# set compile options
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O2 -Wall -Wextra -Wpedantic)
endif()

# set source files
file(GLOB_RECURSE src_files 
  LIST_DIRECTORIES false 
  "src/${PROJECT_NAME}/*")

file(GLOB app_files 
  LIST_DIRECTORIES false
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/src
  "src/*")

# set library
add_library(${PROJECT_NAME}_lib
  ${src_files}
)

target_include_directories(${PROJECT_NAME}_lib
  PRIVATE include
  INTERFACE include
)
ament_target_dependencies(${PROJECT_NAME}_lib ${ros2_required_packages})

#set proto files
protobuf_generate_cpp(proto_src proto_hdr
  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/ssl-vision/src/shared/proto/messages_robocup_ssl_detection.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/ssl-vision/src/shared/proto/messages_robocup_ssl_geometry.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/3rd/ssl-vision/src/shared/proto/messages_robocup_ssl_wrapper.proto)
add_library(${PROJECT_NAME}_proto_lib
  ${proto_src} ${proto_hdr}
)

target_link_libraries(${PROJECT_NAME}_proto_lib
  PRIVATE   protobuf::libprotobuf
  INTERFACE protobuf::libprotobuf
)

target_include_directories(${PROJECT_NAME}_proto_lib
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
  INTERFACE ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(${PROJECT_NAME}_lib ${PROJECT_NAME}_proto_lib)

# set msg files
file(GLOB msg_files
  LIST_DIRECTORIES false
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  "msg/*")
rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} DEPENDENCIES ${ros2_required_packages})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(${PROJECT_NAME}_lib ${cpp_typesupport_target})

#qt
target_link_libraries(${PROJECT_NAME}_lib Qt6::Network)

# make executor
foreach(app_file IN LISTS app_files)
  string(FIND ${app_file} . index)
  string(SUBSTRING ${app_file} 0 ${index} executor)
  set(executors ${executors};${executor})
  add_executable(${executor}
    src/${app_file}
  )
  target_link_libraries(${executor} ${PROJECT_NAME}_lib)
endforeach()

# install executor
install(TARGETS
  ${executors}
  DESTINATION lib/${PROJECT_NAME}
)

#linter
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()