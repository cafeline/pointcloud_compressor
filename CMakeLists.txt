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
  src/core/PatternEncoder.cpp
  src/core/PointCloudCompressor.cpp
  src/config/CompressorConfig.cpp
)

# Create core library
add_library(${PROJECT_NAME}_core ${CORE_SOURCES})
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)

# Runtime integration library
add_library(${PROJECT_NAME}_runtime
  src/ros/runtime/pointcloud_compressor.cpp
  src/ros/runtime/RuntimeHelpers.cpp
  src/ros/runtime/CompressionArtifacts.cpp
  src/ros/runtime/TempFileManager.cpp
)
target_link_libraries(${PROJECT_NAME}_runtime PUBLIC ${PROJECT_NAME}_core)
target_include_directories(${PROJECT_NAME}_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${PROJECT_NAME}_runtime PUBLIC cxx_std_17)

# Add OpenMP support if available
if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME}_core PUBLIC OpenMP::OpenMP_CXX)
  target_compile_definitions(${PROJECT_NAME}_core PUBLIC USE_OPENMP)
endif()

# Link HDF5 libraries
target_link_libraries(${PROJECT_NAME}_core PUBLIC ${HDF5_CXX_LIBRARIES} ${HDF5_HL_LIBRARIES} yaml-cpp)

# Main executable
add_executable(${PROJECT_NAME}_cli src/apps/cli_main.cpp)
target_link_libraries(${PROJECT_NAME}_cli ${PROJECT_NAME}_core ${PROJECT_NAME}_runtime)

# ROS2 Node executable
add_executable(${PROJECT_NAME}_node src/ros/pointcloud_compressor_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_runtime)
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
install(TARGETS ${PROJECT_NAME}_runtime
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
  
  # Test data directory
  set(TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/test/test_data")
  
  # PCD I/O Test
  ament_add_gtest(test_pcd_io test/test_pcd_io.cpp)
  target_link_libraries(test_pcd_io ${PROJECT_NAME}_core)
  target_compile_definitions(test_pcd_io PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
  
  # PLY I/O Test
  ament_add_gtest(test_ply_io test/test_ply_io.cpp)
  target_link_libraries(test_ply_io ${PROJECT_NAME}_core)
  target_compile_definitions(test_ply_io PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
  
  # PLY property order Test (ASCII extra property before x,y,z)
  ament_add_gtest(test_ply_property_order test/test_ply_property_order.cpp)
  target_link_libraries(test_ply_property_order ${PROJECT_NAME}_core)
  
  # File Format Detector Test
  ament_add_gtest(test_file_format_detector test/test_file_format_detector.cpp)
  target_link_libraries(test_file_format_detector ${PROJECT_NAME}_core)
  target_compile_definitions(test_file_format_detector PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
  
  # PointCloud I/O Test
  ament_add_gtest(test_pointcloud_io test/test_pointcloud_io.cpp)
  target_link_libraries(test_pointcloud_io ${PROJECT_NAME}_core)
  target_compile_definitions(test_pointcloud_io PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
  
  # PCD binary with extra property
  ament_add_gtest(test_pcd_binary_extra_property test/test_pcd_binary_extra_property.cpp)
  target_link_libraries(test_pcd_binary_extra_property ${PROJECT_NAME}_core)
  
  # Voxel Processor Test
  ament_add_gtest(test_voxel_processor test/test_voxel_processor.cpp)
  target_link_libraries(test_voxel_processor ${PROJECT_NAME}_core)
  
  # Dictionary Builder Test
  ament_add_gtest(test_dictionary_builder test/test_dictionary_builder.cpp)
  target_link_libraries(test_dictionary_builder ${PROJECT_NAME}_core)
  
  # Pattern Encoder Test
  ament_add_gtest(test_pattern_encoder test/test_pattern_encoder.cpp)
  target_link_libraries(test_pattern_encoder ${PROJECT_NAME}_core)
  
  # Integration Test
  ament_add_gtest(test_integration test/test_integration.cpp)
  target_link_libraries(test_integration ${PROJECT_NAME}_core)
  target_compile_definitions(test_integration PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
  
  # Large Pattern Indices Test
  ament_add_gtest(test_large_pattern_indices test/test_large_pattern_indices.cpp)
  target_link_libraries(test_large_pattern_indices ${PROJECT_NAME}_core)
  
  # Dynamic Index Encoding Test
  ament_add_gtest(test_dynamic_index_encoding test/test_dynamic_index_encoding.cpp)
  target_link_libraries(test_dynamic_index_encoding ${PROJECT_NAME}_core)
  
  # Decompression Test
  ament_add_gtest(test_decompression test/test_decompression.cpp)
  target_link_libraries(test_decompression ${PROJECT_NAME}_core)
  
  # Bit Voxel Grid Test
  ament_add_gtest(test_bit_voxel_grid test/test_bit_voxel_grid.cpp)
  target_link_libraries(test_bit_voxel_grid ${PROJECT_NAME}_core)
  
  # ROS Message Utils Test
  ament_add_gtest(test_ros_message_utils test/test_ros_message_utils.cpp)
  target_link_libraries(test_ros_message_utils 
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}__rosidl_typesupport_cpp
  )
  add_dependencies(test_ros_message_utils ${PROJECT_NAME}__rosidl_typesupport_cpp)
  
  # ROS Message Utils Advanced (32/64-bit) Test
  ament_add_gtest(test_ros_message_utils_advanced test/test_ros_message_utils_advanced.cpp)
  target_link_libraries(test_ros_message_utils_advanced 
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}__rosidl_typesupport_cpp
  )
  add_dependencies(test_ros_message_utils_advanced ${PROJECT_NAME}__rosidl_typesupport_cpp)
  
  # Voxel Grid Reuse Test
  ament_add_gtest(test_voxel_grid_reuse test/test_voxel_grid_reuse.cpp)
  target_link_libraries(test_voxel_grid_reuse ${PROJECT_NAME}_core)
  
  # Block Size Optimization Test
  ament_add_gtest(test_block_size_optimization test/test_block_size_optimization.cpp)
  target_link_libraries(test_block_size_optimization ${PROJECT_NAME}_core)
  
  # Voxel Round Trip Test
  ament_add_gtest(test_voxel_round_trip test/test_voxel_round_trip.cpp)
  target_link_libraries(test_voxel_round_trip ${PROJECT_NAME}_core)
  
  # HDF5 I/O Test
  ament_add_gtest(test_hdf5_io test/test_hdf5_io.cpp)
  target_link_libraries(test_hdf5_io ${PROJECT_NAME}_core)

  # Runtime API Test
  ament_add_gtest(test_runtime_api test/test_runtime_api.cpp)
  target_link_libraries(test_runtime_api ${PROJECT_NAME}_runtime)
  target_compile_definitions(test_runtime_api PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")

  ament_add_gtest(test_runtime_helpers test/test_runtime_helpers.cpp)
  target_link_libraries(test_runtime_helpers ${PROJECT_NAME}_runtime)

  ament_add_gtest(test_runtime_artifacts test/test_runtime_artifacts.cpp)
  target_link_libraries(test_runtime_artifacts ${PROJECT_NAME}_runtime)

  ament_add_gtest(test_runtime_temp_manager test/test_runtime_temp_manager.cpp)
  target_link_libraries(test_runtime_temp_manager ${PROJECT_NAME}_runtime)

  ament_add_gtest(test_compressor_config test/test_compressor_config.cpp)
  target_link_libraries(test_compressor_config ${PROJECT_NAME}_core)
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()
