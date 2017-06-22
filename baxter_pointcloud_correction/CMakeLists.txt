cmake_minimum_required(VERSION 2.8.3)
project(baxter_pointcloud_correction)

find_package(PCL REQUIRED COMPONENTS common io features visualization)
find_package(Boost COMPONENTS system filesystem REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

generate_dynamic_reconfigure_options(cfg/SegmentationParameters.cfg)

add_service_files(
  FILES
  SegmentPointCloud.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime geometry_msgs pcl_conversions pcl_msgs pcl_ros sensor_msgs std_msgs dynamic_reconfigure eigen_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pointcloud_segmentation_server
	src/pointcloud_segmentation_server.cpp
)
add_dependencies(pointcloud_segmentation_server
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(pointcloud_segmentation_server
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
)

add_executable(model_composer
	src/model_composer.cpp
)
add_dependencies(model_composer
	${${PROJECT_NAME}_EXPORTED_TARGETS}
)
target_link_libraries(model_composer
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  ${PCL_COMMON_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
)

