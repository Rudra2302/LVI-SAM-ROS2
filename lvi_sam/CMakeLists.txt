cmake_minimum_required(VERSION 3.8)
project(lvi_sam)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(Eigen3_DIR "/opt/eigen/share/eigen3/cmake")

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED  COMPONENTS kdtree filters)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)
find_package(Boost REQUIRED COMPONENTS atomic filesystem program_options system timer)
find_package(OpenMP REQUIRED)
find_package(rosbag2_cpp REQUIRED)
# find_package(camodocal REQUIRED)


# Custom messages
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_generator_cpp REQUIRED)
find_package(lvi_sam_msgs REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/CloudInfo.msg"
#   DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs nav_msgs rclcpp rosidl_default_generators rosidl_generator_cpp visualization_msgs
# )

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Visual feature files
file(GLOB_RECURSE visual_feature_files
  "src/visual_odometry/visual_feature/*.cpp"	
  "src/visual_odometry/visual_feature/camera_models/*.cc"
)

file(GLOB_RECURSE visual_estimator_files
  # "src/visual_odometry/visual_estimator/factor/*.cpp"
  # "src/visual_odometry/visual_estimator/initial/*.cpp"
  # "src/visual_odometry/visual_estimator/utility/*.cpp"
  # "src/visual_odometry/visual_estimator/feature_manager.cpp"
  # "src/visual_odometry/visual_estimator/parameters.cpp"
  # "src/visual_odometry/visual_estimator/estimator.cpp"
  # "src/visual_odometry/visual_estimator/estimator_node.cpp"
  "src/visual_odometry/visual_estimator/*.cpp"
)

file(GLOB_RECURSE visual_loop_files
  "src/visual_odometry/visual_loop/*.cpp"
)

# Visual modules
add_executable(visual_feature_node ${visual_feature_files})
target_link_libraries(visual_feature_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
ament_target_dependencies(visual_feature_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs cv_bridge tf2 tf2_geometry_msgs pcl_conversions lvi_sam_msgs)

add_executable(visual_estimator_node ${visual_estimator_files})
target_link_libraries(visual_estimator_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
ament_target_dependencies(visual_estimator_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs cv_bridge tf2 tf2_geometry_msgs pcl_conversions lvi_sam_msgs)

add_executable(visual_loop_node ${visual_loop_files})
target_link_libraries(visual_loop_node ${PCL_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
ament_target_dependencies(visual_loop_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs cv_bridge tf2 tf2_geometry_msgs pcl_conversions lvi_sam_msgs)

# Lidar modules
add_executable(imu_preintegration_node src/lidar_odometry/imuPreintegration.cpp)
target_link_libraries(imu_preintegration_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam Boost::timer Boost::atomic)
ament_target_dependencies(imu_preintegration_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs cv_bridge tf2 tf2_geometry_msgs pcl_conversions lvi_sam_msgs rosbag2_cpp)

add_executable(image_projection_node src/lidar_odometry/imageProjection.cpp)
target_link_libraries(image_projection_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(image_projection_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs cv_bridge tf2 tf2_geometry_msgs pcl_conversions lvi_sam_msgs rosbag2_cpp)

add_executable(feature_extraction_node src/lidar_odometry/featureExtraction.cpp)
target_link_libraries(feature_extraction_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
ament_target_dependencies(feature_extraction_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs cv_bridge tf2 tf2_geometry_msgs pcl_conversions lvi_sam_msgs rosbag2_cpp)

add_executable(map_optimization_node src/lidar_odometry/mapOptmization.cpp)
target_compile_options(map_optimization_node PRIVATE -fopenmp)
target_link_libraries(map_optimization_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam Boost::timer OpenMP::OpenMP_CXX Boost::atomic)
ament_target_dependencies(map_optimization_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs cv_bridge tf2 tf2_geometry_msgs pcl_conversions lvi_sam_msgs rosbag2_cpp)

add_executable(pcap_publisher_node src/lidar_odometry/pcap_publisher_node.cpp)
target_link_libraries(pcap_publisher_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} pcap gtsam Boost::timer Boost::atomic)
ament_target_dependencies(pcap_publisher_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs cv_bridge tf2 tf2_geometry_msgs pcl_conversions lvi_sam_msgs rosbag2_cpp image_transport)

add_executable(trajectory_node src/lidar_odometry/trajectory2D.cpp)
target_link_libraries(trajectory_node ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam Boost::timer Boost::atomic)
ament_target_dependencies(trajectory_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs cv_bridge tf2 tf2_geometry_msgs pcl_conversions lvi_sam_msgs rosbag2_cpp image_transport)

# Install targets
install(TARGETS
  visual_feature_node
  visual_estimator_node
  visual_loop_node
  imu_preintegration_node
  image_projection_node
  feature_extraction_node
  map_optimization_node
  pcap_publisher_node
  trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
