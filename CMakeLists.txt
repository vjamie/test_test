cmake_minimum_required(VERSION 2.8.3)
project(astar)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(CUDA 6.5 REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(OpenCV 3.4 COMPONENTS core highgui imgproc video REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)



add_definitions(-std=c++11)# -m64) #-Wall)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test
  CATKIN_DEPENDS roscpp std_msgs

#  DEPENDS system_lib
   sensor_msgs
   cv_bridge
   image_transport
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${Boost_LIBRARY_DIRS})


add_executable(astar src/astar.cpp)

target_link_libraries(
	astar
        ${catkin_LIBRARIES}
        ${ZED_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}  
        ${CUDA_LIBRARIES}
        ${Boost_LIBRARIES}
)


