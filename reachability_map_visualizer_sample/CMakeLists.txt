cmake_minimum_required(VERSION 3.0.2)
project(reachability_map_visualizer_sample)

add_compile_options(-std=c++14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  reachability_map_visualizer
  choreonoid_viewer
  roslib
  jvrc_models
  manta_models
  )

find_package(choreonoid REQUIRED)

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${CHOREONOID_INCLUDE_DIRS}
)

add_library(CnoidReachabilityMapVisualizerSamplePlugin SHARED
  src/CnoidReachabilityMapVisualizerSamplePlugin.cpp
  src/jaxon_hand.cpp
  src/jaxon_foot.cpp
  src/manta_tail.cpp
  src/visualize.cpp
  src/solvability_bar.cpp
  )
target_link_libraries(CnoidReachabilityMapVisualizerSamplePlugin
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_PLUGIN_LIBRARIES}
  )
set_target_properties(CnoidReachabilityMapVisualizerSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
