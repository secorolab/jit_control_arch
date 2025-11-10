cmake_minimum_required(VERSION 3.20)
project(jit_control_arch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# --- LLVM setup ---
set(LLVM_DIR "/usr/lib/llvm-20/lib/cmake/llvm")
find_package(LLVM REQUIRED CONFIG)
message(INFO " Found LLVM ${LLVM_PACKAGE_VERSION}")
message(INFO " Using LLVMConfig.cmake in: ${LLVM_DIR}")

include_directories(SYSTEM ${LLVM_INCLUDE_DIRS})
separate_arguments(LLVM_DEFINITIONS_LIST NATIVE_COMMAND ${LLVM_DEFINITIONS})
add_definitions(${LLVM_DEFINITIONS})

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

add_executable(jit_node src/jit_node.cpp)
llvm_map_components_to_libnames(llvm_libs support core orcjit native irreader)
target_link_libraries(jit_node ${llvm_libs} ffi)
target_include_directories(jit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(jit_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(jit_node rclcpp ament_index_cpp)

install(TARGETS controllers
  EXPORT controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

install(TARGETS jit_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
