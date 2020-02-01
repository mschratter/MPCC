cmake_minimum_required(VERSION 3.5.1)
project(mpcc)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O2")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  tf2
  mpcc_msgs
)

# Includes
catkin_package(
  INCLUDE_DIRS include
)

include_directories(C++/.)
include_directories(C++/External/blasfeo/lib/include)
include_directories(C++/External/hpipm/lib/include)
include_directories(C++/External/Eigen)
include_directories(C++/External/Json/include)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ../../C++/.
  ../../C++/External/blasfeo/lib/include
  ../../C++/External/hpipm/lib/include
  ../../C++/External/Eigen
  ../../C++/External/Json/include
)

# Project sources
set(SRCS
  ../../C++/types.cpp
  ../../C++/types.h
  ../../C++/config.h
  ../../C++/Params/params.cpp
  ../../C++/Spline/cubic_spline.cpp
  ../../C++/Spline/arc_length_spline.cpp
  ../../C++/Interfaces/hpipm_interface.cpp
  ../../C++/Interfaces/solver_interface.cpp
  ../../C++/Constraints/constraints.cpp
  ../../C++/Constraints/bounds.cpp
  ../../C++/Cost/cost.cpp
  ../../C++/Model/model.cpp
  ../../C++/Model/integrator.cpp
  ../../C++/MPC/mpc.cpp
  ../../C++/Params/track.cpp
  ../../C++/Params/track.h
)


# SimpleSim
add_library(libsimple_sim 
  src/SimpleSim.cpp
  ${SRCS}
)

add_dependencies(libsimple_sim 
  ${catkin_EXPORTED_TARGETS}
  mpcc_msgs_generate_messages_cpp
)

target_include_directories(libsimple_sim PRIVATE)
target_link_libraries(libsimple_sim ${catkin_LIBRARIES})
target_link_libraries(libsimple_sim ${CMAKE_SOURCE_DIR}/MPCC/C++/External/hpipm/lib/lib/libhpipm.a ${CMAKE_SOURCE_DIR}/MPCC/C++/External/blasfeo/lib/lib/libblasfeo.a m)


add_executable(simple_sim_node src/SimpleSimNode.cpp)
target_link_libraries(simple_sim_node libsimple_sim)
add_dependencies(simple_sim_node
        ${catkin_EXPORTED_TARGETS}
)


# MPCC
add_library(libmpcc 
  src/MPCC.cpp
  ${SRCS}
)

add_dependencies(libmpcc 
  ${catkin_EXPORTED_TARGETS}
  mpcc_msgs_generate_messages_cpp
)

target_include_directories(libmpcc PRIVATE)
target_link_libraries(libmpcc ${catkin_LIBRARIES})
target_link_libraries(libmpcc ${CMAKE_SOURCE_DIR}/MPCC/C++/External/hpipm/lib/lib/libhpipm.a ${CMAKE_SOURCE_DIR}/MPCC/C++/External/blasfeo/lib/lib/libblasfeo.a m)

add_executable(mpcc_node src/MPCCNode.cpp)
target_link_libraries(mpcc_node libmpcc)
add_dependencies(mpcc_node
  ${catkin_EXPORTED_TARGETS}
)



#############
## Install ##
#############

## Install executables and/or libraries
install(TARGETS mpcc_node libmpcc simple_sim_node libsimple_sim
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install script and launch installers
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install config folder
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

