cmake_minimum_required(VERSION 3.5)

project(grid_map_generator LANGUAGES CXX)
add_compile_options(-std=c++11)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
 LIBRARIES map_filter
 CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs
 DEPENDS system_lib PCL
)

include_directories(
  include
  common
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcd_grid_divider
                src/main.cpp
                src/mainwindow.cpp
                include/mainwindow.h
                src/mainwindow.ui
                common/macro.h
                common/util.h
                common/util.cpp
                include/pcd_grid_divider.h
                src/pcd_grid_divider.cpp
)

target_link_libraries(pcd_grid_divider PRIVATE Qt5::Widgets ${catkin_LIBRARIES})
