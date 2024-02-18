#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.0.2)
project(polar_height_map)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(${PROJECT_NAME}_CATKIN_DEPS
    nodelet
    roscpp
    pcl_ros
    std_msgs
    sensor_msgs
    tf)

find_package(OpenCV)

find_package(catkin REQUIRED COMPONENTS
             ${${PROJECT_NAME}_CATKIN_DEPS}
             angles
             pcl_conversions
             roslaunch rostest)
find_package(Boost)
find_package(PCL REQUIRED QUIET COMPONENTS common)

include_directories(include
                    ${Boost_INCLUDE_DIR}
                    ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIR}
 		    ${OpenCV_INCLUDE_DIRS}	)

catkin_package(CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

add_executable(polarheightmap_node src/polarheightmap_node.cpp src/polarheightmap.cpp)
#add_executable(polarheightmap_node_full src/polarheightmap_node.cpp src/polarheightmap_full.cpp)
add_executable(heightmap_node_grid_stereo src/heightmap_grid_stereo.cpp)
add_executable(heightmap_full_stereo src/heightmap_full_stereo.cpp)
add_executable(polarheightmap_full_stereo src/polar_heightmap_full_stereo.cpp)

target_link_libraries(polarheightmap_node
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
 		      ${OpenCV_LIBRARIES})
target_link_libraries(heightmap_node_grid_stereo
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
 		      ${OpenCV_LIBRARIES})
target_link_libraries(heightmap_full_stereo
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
 		      ${OpenCV_LIBRARIES})
target_link_libraries(polarheightmap_full_stereo
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
 		      ${OpenCV_LIBRARIES})
#target_link_libraries(polarheightmap_node_full
#                      ${catkin_LIBRARIES}
#                      ${Boost_LIBRARIES}
#                      ${PCL_LIBRARIES}
# 		      ${OpenCV_LIBRARIES})

add_library(polarheightmap_nodelet src/polarheightmap_nodelet.cpp src/polarheightmap.cpp)
target_link_libraries(polarheightmap_nodelet
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${PCL_LIBRARIES}
			${OpenCV_LIBRARIES})

# install the files needed to run this package
install(TARGETS polarheightmap_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#install(TARGETS polarheightmap_node_full
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS polarheightmap_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(FILES nodelets.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

# unit tests
if (CATKIN_ENABLE_TESTING)
  message("-- ${PROJECT_NAME} unit testing enabled")

  # Download a packet capture (PCAP) file containing test data.
  # Store it in devel-space, so rostest can easily find it.
  catkin_download_test_data(
    ${PROJECT_NAME}_tests_class.pcap
    http://download.ros.org/data/velodyne/class.pcap
    DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
    MD5 65808d25772101358a3719b451b3d015)
  
  # declare rostest scripts
  add_rostest(tests/polarheightmap_node_hz.test)
  add_rostest(tests/polarheightmap_nodelet_hz.test)
  
  # parse check all the launch/*.launch files
  ##roslaunch_add_file_check(launch)

else ()
  message("-- ${PROJECT_NAME} unit testing disabled")
endif (CATKIN_ENABLE_TESTING)
