cmake_minimum_required(VERSION 2.8.3)
project(semantic_3d_avp)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)
# set(GTSAM_INCLUDE_DIR "/usr/local/include/gtsam")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV 3.2.0  REQUIRED)
find_package(Ceres REQUIRED)

find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED QUIET)
message("GTSAM version  is ： ${GTSAM_INCLUDE_DIR}")
message("PCL version  is ： ${PCL_INCLUDE_DIRS}")
message("Ceres version  is ： ${CERES_INCLUDE_DIRS}")
message("OpenCV version  is ： ${OpenCV_INCLUDE_DIRS}")

include_directories(
  include
  	 ${PCL_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS} 
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
     DEPENDS EIGEN3 PCL 
   INCLUDE_DIRS include
)

add_executable(pubimage src/pubimage.cpp)
target_link_libraries(pubimage ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(pubpose src/pubpose.cpp)
target_link_libraries(pubpose ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(pubpose2 src/pubpose2.cpp)
target_link_libraries(pubpose2 ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(pointCloudFromRGB src/pointCloudFromRGB.cpp)
target_link_libraries(pointCloudFromRGB ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(pointCloudFromtxt src/pointCloudFromtxt.cpp)
target_link_libraries(pointCloudFromtxt ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(mapping src/mapping.cpp)
target_link_libraries(mapping ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(txt_mapping src/txt_mapping.cpp)
target_link_libraries(txt_mapping ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(localization src/localization.cpp)
target_link_libraries(localization ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PROJECT_NAME})

add_executable(localization_cube src/localization_cube.cpp)
target_link_libraries(localization_cube ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${PROJECT_NAME})

add_executable(slam src/slam.cpp)
target_link_libraries(slam ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(txt_localization src/txt_localization.cpp)
target_link_libraries(txt_localization ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(transfer_point_cloud src/transfer_point_cloud.cpp)
target_link_libraries(transfer_point_cloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(ransac_point2line src/ransac_point2line.cpp)
target_link_libraries(ransac_point2line ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(ransac_point2line_connection src/ransac_point2line_connection.cpp)
target_link_libraries(ransac_point2line_connection ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(pub_pointcloud src/pub_pointcloud.cpp)
target_link_libraries(pub_pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(line2pointcloud src/line2pointcloud.cpp)
target_link_libraries(line2pointcloud ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(PGO 
  src/laserPosegraphOptimization.cpp
  include/scancontext/Scancontext.cpp
)
target_compile_options(PGO 
  PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(PGO 
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES}
  ${OpenMP_CXX_FLAGS}
  gtsam
)

# 头文件路径
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)
## Declare a C++ library
add_library(${PROJECT_NAME}  SHARED
src/matrix_utils.cpp 
)
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
)


add_executable(object_map_node src/object_map.cpp)
target_link_libraries(object_map_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pose_graph_optimizer src/pose_graph_optimizer.cpp)
target_link_libraries(pose_graph_optimizer ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)

add_executable(pose_graph_optimizer2 src/pose_graph_optimizer2.cpp)
target_link_libraries(pose_graph_optimizer2 ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so)