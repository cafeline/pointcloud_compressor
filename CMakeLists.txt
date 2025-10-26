cmake_minimum_required(VERSION 3.8)
project(pointcloud_compressor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
find_package(yaml-cpp REQUIRED)

# ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIRS}
)

# Generate messages
set(msg_files
  "msg/PatternDictionary.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

# Core library source files
set(CORE_SOURCES
  src/io/PcdIO.cpp
  src/io/PlyIO.cpp
  src/io/FileFormatDetector.cpp
  src/io/PointCloudIO.cpp
  src/io/HDF5IO.cpp
  src/model/VoxelGrid.cpp
  src/core/VoxelProcessor.cpp
  src/core/PatternDictionaryBuilder.cpp
  src/core/PointCloudCompressor.cpp
  src/core/CompressionReportFormatter.cpp
  src/core/BlockSizeReportFormatter.cpp
  src/config/CompressorConfig.cpp
  src/config/CliConfigParser.cpp
  src/config/ConfigTransforms.cpp
)

# Create core library
add_library(${PROJECT_NAME}_core ${CORE_SOURCES})
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

# Compression integration library
add_library(${PROJECT_NAME}_bridge
  src/bridge/Bridge.cpp
  src/common/CompressionDataUtils.cpp
  src/common/CompressionArtifacts.cpp
  src/services/CompressionExecutor.cpp
  src/io/CompressionReportBuilder.cpp
  src/io/Hdf5Writers.cpp
)
target_link_libraries(${PROJECT_NAME}_bridge PUBLIC ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_bridge PUBLIC cxx_std_17)

# Add OpenMP support if available
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_core PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC USE_OPENMP)
endif()

# Link HDF5 libraries
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${HDF5_CXX_LIBRARIES} ${HDF5_HL_LIBRARIES} yaml-cpp)

# Main executable
add_executable(${PROJECT_NAME}_cli src/apps/cli_main.cpp)
target_link_libraries(${PROJECT_NAME}_cli ${PROJECT_NAME}_core ${PROJECT_NAME}_bridge)

# ROS2 Node executable
add_executable(${PROJECT_NAME}_node src/ros/pointcloud_compressor_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_bridge)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  builtin_interfaces
)

# C++ viewer removed - using Python viewer only

# Ensure message headers are generated before building the nodes
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")

# Build block size optimizer node
add_executable(block_size_optimizer_node src/ros/block_size_optimizer_node.cpp)
target_link_libraries(block_size_optimizer_node ${PROJECT_NAME}_core)
ament_target_dependencies(block_size_optimizer_node
  rclcpp
  std_msgs
  std_srvs
)

# Installation
install(TARGETS ${PROJECT_NAME}_core
  DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_bridge
  DESTINATION lib)
install(TARGETS ${PROJECT_NAME}_cli ${PROJECT_NAME}_node block_size_optimizer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_end_to_end test/test_end_to_end.cpp)
  target_link_libraries(test_end_to_end ${PROJECT_NAME}_core)

  ament_add_gtest(test_bridge_api test/test_bridge_api.cpp)
  target_link_libraries(test_bridge_api ${PROJECT_NAME}_bridge)

  ament_add_gtest(test_compression_executor test/test_compression_executor.cpp)
  target_link_libraries(test_compression_executor ${PROJECT_NAME}_bridge)
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
