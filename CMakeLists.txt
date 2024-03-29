cmake_minimum_required(VERSION 2.8.3)
project(MPF)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg -g3 -O3 -m64 -Ofast -flto -msse2 -msse3 -march=native  -mfpmath=sse")

set(CMAKE_CXX_STANDARD 14)

include_directories(${PROJECT_SOURCE_DIR}/include)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)

#ECBS
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/ecbs)
include_directories(${PROJECT_SOURCE_DIR}/third_party/ecbs/include)

#BOOST14
find_package(Boost 1.58 REQUIRED COMPONENTS program_options)

#EIGEN
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
message( STATUS "===========Eigen3 ${Eigen3_INCLUDE_DIRS}")

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

#OCTOMAP
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
message( STATUS "===========octomap ${OCTOMAP_INCLUDE_DIRS}")
link_libraries(${OCTOMAP_LIBRARIES})
add_definitions(-DOCTOMAP_NODEBUGOUT)

#EDT
find_package(dynamicEDT3D REQUIRED)
include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
message( STATUS "===========dynamicEDT3D ${DYNAMICEDT3D_INCLUDE_DIRS}")
link_libraries(${DYNAMICEDT3D_LIBRARIES})

#CPLEX
add_definitions(-DNDEBUG)
add_definitions(-DIL_STD)
set(CPLEX_PREFIX_DIR      /opt/ibm/ILOG/CPLEX_Studio129)
#set(CPLEX_PREFIX_DIR      /opt/ibm/ILOG/CPLEX_Studio_Community129/)
set(CPLEX_INCLUDE_DIR     ${CPLEX_PREFIX_DIR}/cplex/include)
set(CPLEX_LIBRARIES_DIR   ${CPLEX_PREFIX_DIR}/cplex/lib/x86-64_linux/static_pic)
set(CONCERT_INCLUDE_DIR   ${CPLEX_PREFIX_DIR}/concert/include)
set(CONCERT_LIBRARIES_DIR ${CPLEX_PREFIX_DIR}/concert/lib/x86-64_linux/static_pic)
include_directories(${CPLEX_INCLUDE_DIR} ${CONCERT_INCLUDE_DIR})
link_directories(${CPLEX_LIBRARIES_DIR} ${CONCERT_LIBRARIES_DIR})

#CATKIN
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  nav_msgs
  geometry_msgs
  octomap_ros
  octomap_msgs
)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp roslib message_runtime
)

#MATPLOTLIB-CPP
find_package(PythonLibs 2.7)

#BUILD
add_executable(random_map_generator src/random_map_generator.cpp)
target_link_libraries(random_map_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(following_traj_planner src/following_traj_planner.cpp)
target_link_libraries(following_traj_planner
  ${catkin_LIBRARIES}
  ${OCTOMAP_INCLUDE_DIRS}
  ${SIPP_LINK_LIBS}
  ${PYTHON_LIBRARIES}
  m
  ilocplex
  cplex
  concert
  pthread
  dl
)