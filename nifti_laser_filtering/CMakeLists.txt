cmake_minimum_required(VERSION 2.8.3)
project(nifti_laser_filtering)

set(THIS_PACKAGE_ROS_DEPS
    filters
    geometric_shapes
    geometry_msgs
    laser_geometry
    message_filters
    moveit_ros_perception
    nifti_robot_description
    pluginlib
    roscpp
    sensor_msgs
    stamped_msgs
    tf
    tf_conversions
    urdf
    visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS ${THIS_PACKAGE_ROS_DEPS} tf2_sensor_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(PCL REQUIRED)

find_package(ASSIMP QUIET)
if (NOT ASSIMP_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(ASSIMP assimp)
endif()
if (ASSIMP_FOUND)
    if (NOT ${ASSIMP_VERSION} VERSION_LESS "2.0.1150")
        add_definitions(-DASSIMP_UNIFIED_HEADER_NAMES)
        message(STATUS "Assimp version has unified headers")
    else()
        message(STATUS "Assimp version does not have unified headers")
    endif()
else()
    message(STATUS "could not find assimp (perhaps available thorugh ROS package?), so assuming assimp v2")
    set(ASSIMP_INCLUDE_DIRS )
    set(ASSIMP_LIBRARY_DIRS )
    set(ASSIMP_LIBRARIES assimp)
    set(ASSIMP_LIBRARY_DIRS)
    set(ASSIMP_CXX_FLAGS)
    set(ASSIMP_CFLAGS_OTHER)
    set(ASSIMP_LINK_FLAGS)
    set(ASSIMP_INCLUDE_DIRS)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tradr_laser_filters
  CATKIN_DEPENDS ${THIS_PACKAGE_ROS_DEPS}
)

###########
## Build ##
###########

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag(-std=c++11 HAS_STD_CPP11_FLAG)
if(HAS_STD_CPP11_FLAG)
  add_compile_options(-std=c++11)
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ASSIMP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(tradr_laser_filters
    src/TradrLaserShadowFilter.cpp
    src/NearFilter.cpp
    src/RobotSelfFilter.cpp)
target_link_libraries(tradr_laser_filters ${catkin_LIBRARIES} ${PCL_LIBRARIES} assimp)
add_dependencies(tradr_laser_filters ${catkin_EXPORTED_TARGETS})

add_executable(nifti_laser_filtering
    src/nodes/nifti_laser_filtering.cpp)
target_link_libraries(nifti_laser_filtering ${catkin_LIBRARIES} assimp)

#############
## Install ##
#############

install(TARGETS tradr_laser_filters nifti_laser_filtering
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
    tradr_laser_filters.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES
    config/nifti_laser_filters.yaml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES
  launch/nifti_laser_filtering.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
