cmake_minimum_required(VERSION 3.20)
project(jit_control_arch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

# system dependencies
# --- LLVM setup ---
set(LLVM_DIR "/usr/lib/llvm-20/lib/cmake/llvm")
find_package(LLVM REQUIRED CONFIG)
message(INFO " Found LLVM ${LLVM_PACKAGE_VERSION}")
message(INFO " Using LLVMConfig.cmake in: ${LLVM_DIR}")

include_directories(SYSTEM ${LLVM_INCLUDE_DIRS})
separate_arguments(LLVM_DEFINITIONS_LIST NATIVE_COMMAND ${LLVM_DEFINITIONS})
add_definitions(${LLVM_DEFINITIONS})

find_package(robif2b REQUIRED)

# ros2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)

include_directories(
  include
  ${kdl_parser_INCLUDE_DIRS}
  ${robif2b_INCLUDE_DIRS}
)

# add controllers library
add_library(controllers SHARED
  src/controllers.c
)
target_include_directories(controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(controllers PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(controllers)

ament_export_targets(controllers HAS_LIBRARY_TARGET)

llvm_map_components_to_libnames(llvm_libs support core orcjit native irreader)

# add jit test node
add_executable(jit_test_node src/jit_test_node.cpp)
target_link_libraries(jit_test_node ${llvm_libs} ffi)
target_include_directories(jit_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(jit_test_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(jit_test_node rclcpp ament_index_cpp)

# install kinova test node
add_executable(kinova_test_node src/kinova_test_node.cpp)
target_include_directories(kinova_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(kinova_test_node PUBLIC c_std_99 cxx_std_20)
ament_target_dependencies(kinova_test_node rclcpp kdl_parser orocos_kdl ament_index_cpp)
target_link_libraries(kinova_test_node
  controllers
  ${llvm_libs}
  robif2b::kinova_gen3
)

# install kinova hl test node
add_executable(kinova_hl_test_node src/kinova_hl_test_node.cpp)
target_include_directories(kinova_hl_test_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(kinova_hl_test_node PUBLIC c_std_99 cxx_std_20)
ament_target_dependencies(kinova_hl_test_node rclcpp kdl_parser orocos_kdl ament_index_cpp)
target_link_libraries(kinova_hl_test_node
  ${llvm_libs}
  robif2b::kinova_gen3
)

# crf-screwing-hl
add_executable(crf_screwing_hl_node src/crf_screwing_hl_node.cpp)
target_include_directories(crf_screwing_hl_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(crf_screwing_hl_node PUBLIC c_std_99 cxx_std_20)
ament_target_dependencies(crf_screwing_hl_node rclcpp kdl_parser orocos_kdl ament_index_cpp)
target_link_libraries(crf_screwing_hl_node
  controllers
  ${llvm_libs}
  robif2b::kinova_gen3
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# install targets
install(TARGETS controllers
  EXPORT controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(TARGETS jit_test_node kinova_test_node kinova_hl_test_node crf_screwing_hl_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
