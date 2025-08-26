cmake_minimum_required(VERSION 3.5.1)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW) # to remove warning "Policy CMP0144 is not set"
project(pbr_gazebo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch worlds models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
