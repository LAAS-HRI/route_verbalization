cmake_minimum_required(VERSION 2.8.3)
project(route_verbalization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ontologenius
  message_generation
  semantic_route_description
)
find_package(ontologenius 0.2.7 REQUIRED)
find_package(OpenCV REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
   FILES
   VerbalizeRegionRoute.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

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
#  LIBRARIES semantic_route_drawer
  CATKIN_DEPENDS ontologenius roscpp semantic_route_description
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
  ${OpenCV_INCLUDE_DIRS}
  ${ontoloGenius_lib_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(route_verbalization_lib
  src/mapManipulators/MapReader.cpp
  src/mapManipulators/MapDrawer.cpp
  src/Verbalizers/RegionVerbalizer.cpp
  src/Verbalizers/PlaceVerbalizer.cpp
  src/Verbalizers/sentences.cpp
)
 target_link_libraries(route_verbalization_lib ${catkin_LIBRARIES})
 add_dependencies(route_verbalization_lib ${catkin_EXPORTED_TARGETS})
 target_link_libraries(route_verbalization_lib ${OpenCV_LIBS})

add_executable(draw_route nodes/draw.cpp)
target_link_libraries(draw_route ${catkin_LIBRARIES})
add_dependencies(draw_route ${catkin_EXPORTED_TARGETS})
target_link_libraries(draw_route route_verbalization_lib)

add_executable(verbalize_route nodes/verbalize.cpp)
target_link_libraries(verbalize_route ${catkin_LIBRARIES})
add_dependencies(verbalize_route ${catkin_EXPORTED_TARGETS})
add_dependencies(verbalize_route route_verbalization_gencpp)
target_link_libraries(verbalize_route route_verbalization_lib)

add_executable(verbalize_test nodes/tester.cpp)
target_link_libraries(verbalize_test ${catkin_LIBRARIES})
add_dependencies(verbalize_test ${catkin_EXPORTED_TARGETS})
add_dependencies(verbalize_test route_verbalization_gencpp)
target_link_libraries(verbalize_test route_verbalization_lib)
