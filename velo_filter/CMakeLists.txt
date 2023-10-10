cmake_minimum_required(VERSION 3.0.2)
project(velo_filter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/velo_filter_node.cpp src/velo_filter_core.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(velo_filter_add src/velo_filter_add_node.cpp src/velo_filter_add_core.cpp)
target_link_libraries(velo_filter_add ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#add_executable(velo_filter_trans src/velo_filter_add_node.cpp src/velo_filter_trans_core.cpp)
#target_link_libraries(velo_filter_add ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(euclidean_node src/euclidean_cluster_node.cpp src/euclidean_cluster_core.cpp)
target_link_libraries(euclidean_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(euclidean_node_u_turn src/euclidean_cluster_node.cpp src/euclidean_cluster_u_turn_core.cpp)
target_link_libraries(euclidean_node_u_turn ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(euclidean_color_node src/euclidean_cluster_color_core.cpp src/euclidean_cluster_node.cpp)
target_link_libraries(euclidean_color_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
