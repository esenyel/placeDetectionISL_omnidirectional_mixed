cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

project(placeDetectionISL_omnidirectional_mixed)

FIND_PACKAGE(Qt4 REQUIRED)
SET(QT_USE_QTSQL TRUE)

FIND_PACKAGE( OpenCV REQUIRED )

INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/bubble)
MESSAGE( STATUS "CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR} )


set(bubble_space_SOURCES src/bubble/bubbleprocess.cpp src/database/databasemanager.cpp)# src/pcprocessing.cpp)
set(bubble_space_NSOURCES src/bubble/dfc.cpp src/bubble/invariants.cpp src/imageprocess/imageprocess.cpp src/Utility.cpp)
set(bubble_space_HEADERS src/bubble/bubbleprocess.h src/database/databasemanager.h)# include/bubble_space/pcprocessing.h)
set(bubble_space_NHEADERS src/imageprocess/imageprocess.h src/Utility.h)# include/bubble_space/pcprocessing.h)

QT4_WRAP_CPP(bubble_space_HEADERS_MOC ${bubble_space_HEADERS})
qt4_automoc(${bubble_space_SOURCES})

#ADD_EXECUTABLE(bubble_space ${bubble_space_SOURCES} ${bubble_space_HEADERS_MOC})
#TARGET_LINK_LIBRARIES(bubble_space ${QT_LIBRARIES})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} opencv_highgui opencv_imgproc opencv_core)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(placeDetectionNode src/main.cpp ${bubble_space_SOURCES} ${bubble_space_NSOURCES} ${bubble_space_NHEADERS} ${bubble_space_HEADERS_MOC})
target_link_libraries(placeDetectionNode ${PROJECT_NAME} ${QT_LIBRARIES})
