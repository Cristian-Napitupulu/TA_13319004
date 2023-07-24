cmake_minimum_required(VERSION 3.8)
project(cpp_gantry_crane)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})

set (dependencies "realsense2")

set (gantry_crane_SRCS
  src/point_cloud_lidar.cpp
)

add_executable(${PROJECT_NAME} ${gantry_crane_SRCS})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} realsense2::realsense2 ${OpenCV_LIBS} ${PCL_LIBRARIES})

install (TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
