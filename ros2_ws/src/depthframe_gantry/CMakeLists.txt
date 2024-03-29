cmake_minimum_required(VERSION 3.8)
project(depthframe_gantry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common io visualization segmentation filters)

set (dependencies 
  realsense2
  Qt5
)

add_executable(${PROJECT_NAME}_node src/depthframe_gantry.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  ${dependencies}
)

target_link_libraries(${PROJECT_NAME}_node
  realsense2::realsense2
)
target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  Qt5::Core
  Qt5::Widgets
  ${PCL_LIBRARIES}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  ${PCL_FILTERS_LIBRARIES}
  ${PCL_SEGMENTATION_LIBRARIES}
)

install (TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
