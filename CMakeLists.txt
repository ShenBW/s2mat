cmake_minimum_required(VERSION 2.8)
project(s2mat)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_CXX_STANDARD 14)

cmake_policy(SET CMP0015 NEW)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    tf
    tf2_ros
    tf2_geometry_msgs
    roscpp
    rospy
    message_filters
    cv_bridge
    image_transport
    pcl_ros
    pcl_conversions
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf_conversions
    eigen_conversions
    interactive_markers
    jsk_recognition_msgs
)



## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(OpenMP REQUIRED)
# find_package(Newmat REQUIRED)

# 添加 OpenMP 编译标志
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories(${Eigen3_INCLUDE_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
    Submap.msg
    TrackedObject.msg
    TrackedObjects.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    jsk_recognition_msgs
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    DEPENDS PCL
#  LIBRARIES s2mat
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
)

# link directories
link_directories(
    include
    ${OpenCV_LIBRARY_DIRS}
    ${PCL_LIBRARY_DIRS}  
)

set(NNT_DIR src/front_end/nnt)

set(NNT_SOURCES
    ${NNT_DIR}/ros/geometry_utils.cpp
    ${NNT_DIR}/ros/params.cpp
    ${NNT_DIR}/ros/nnt_ros.cpp
    ${NNT_DIR}/ekf.cpp
    ${NNT_DIR}/imm_filter.cpp
    ${NNT_DIR}/nnt.cpp
    ${NNT_DIR}/nnt_tracker.cpp

    ${NNT_DIR}/occlusion_handling/basic_occlusion_manager.cpp

    ${NNT_DIR}/data_association/greedy_nearest_neighbor_data_association.cpp
)

## Declare a C++ library
add_library(rimg_based_front_end_library
    src/front_end/rimg_front_end.cpp
    src/front_end/rimg_detector/scan_preprocessor.cpp
    src/front_end/rimg_detector/rimg_object_detector.cpp
    src/front_end/rimg_detector/rimg_cluster.cpp
    src/front_end/tracks_processor/tracks_processor.cpp
    include/rectangular_lsap/rectangular_lsap.cpp
    ${NNT_SOURCES}
    src/utils.cpp
)

add_library(back_end_library
    src/back_end/back_end.cpp
    src/back_end/voi_back_end.cpp
    src/utils.cpp
)

add_library(map_merger_library
    src/map_merger/map_merger.cpp
    src/utils.cpp
)

target_link_libraries(rimg_based_front_end_library
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARY} OpenMP::OpenMP_CXX
)

target_link_libraries(back_end_library
    ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARY} ${OCTOMAP_LIBRARIES} OpenMP::OpenMP_CXX
)

target_link_libraries(map_merger_library
  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARY} OpenMP::OpenMP_CXX
)

MESSAGE("OpenCV_LIBS: " ${OpenCV_INCLUDE_DIRS})

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(rimg_front_end_node src/front_end/rimg_front_end_node.cpp)
add_executable(voi_back_end_node src/back_end/voi_back_end_node.cpp)
add_executable(map_merger_node src/map_merger/map_merger_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(rimg_front_end_node ${catkin_EXPORTED_TARGETS})
add_dependencies(voi_back_end_node ${catkin_EXPORTED_TARGETS})
add_dependencies(map_merger_node ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(rimg_front_end_node rimg_based_front_end_library)
target_link_libraries(voi_back_end_node back_end_library)
target_link_libraries(map_merger_node map_merger_library)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_s2mat.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
