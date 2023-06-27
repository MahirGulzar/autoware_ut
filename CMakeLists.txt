cmake_minimum_required(VERSION 2.8.3)
project(autoware_ut)

find_package(catkin REQUIRED COMPONENTS
        autoware_msgs
        libvectormap
        points_downsampler
        rviz
        tf
        topic_tools
        op_ros_helpers
)

catkin_package(
        CATKIN_DEPENDS
        autoware_msgs
        geometry_msgs
        message_filters
        points_downsampler
        roscpp
        rviz
        sensor_msgs
        tf
        topic_tools
        op_ros_helpers
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

include_directories(
        ${catkin_INCLUDE_DIRS})


add_executable(tl_visualizer 
        nodes/tl_visualizer/tl_visualizer.cpp)
target_link_libraries(tl_visualizer
        ${catkin_LIBRARIES})
add_dependencies(tl_visualizer
        ${catkin_EXPORTED_TARGETS})

install(TARGETS
        tl_visualizer
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(vectormap_filter nodes/vectormap_filter/vectormap_filter.cpp)
target_link_libraries(vectormap_filter ${catkin_LIBRARIES})
add_dependencies(vectormap_filter ${catkin_EXPORTED_TARGETS})

install(
        TARGETS vectormap_filter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
        config launch maps nodes rviz scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})