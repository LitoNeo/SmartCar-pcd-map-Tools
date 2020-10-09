cmake_minimum_required(VERSION 3.5)

project(traj_generator LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# add_dependencies(traj_generator utils)

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

find_package(Qt5 COMPONENTS Widgets REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)
# find_package(utils REQUIRED)

include_directories(include)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}                   #The exported libraries from the project
        DEPENDS PCL
        CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs std_msgs
)

include_directories(   # # 去哪里找头文件
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        # "include"
)
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})     # 去哪里找库文件
add_definitions(${PCL_DEFINITIONS})

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

add_executable(traj_generator
  src/main.cpp
  src/mainwindow.cpp
  include/mainwindow.h
  src/mainwindow.ui
  include/traj_extract.h
  src/traj_extract.cpp
)
add_dependencies(traj_generator utils)
target_link_libraries(traj_generator PRIVATE Qt5::Widgets utils ${PCL_LIBRARIES} ${catkin_LIBRARIES})

# ########安装脚本##########
# # 将生成的target安装到指定目录下，这里 detect_cl,img_tool是动态库,test_detect是可执行程序,
# # cmake会自动根据target的类型将可执行程序(如.exe)和动态库(如.dll)作为RUNTIME类型复制到bin下,
# # 将动态库的导入库(.lib .a)作为ARCHIVE类型复制lib/static目录下
# install(TARGETS detect_cl img_tool test_detect # 动态库,动态库, 可执行程序
#         RUNTIME DESTINATION bin
#         LIBRARY DESTINATION lib
#         ARCHIVE DESTINATION lib/static)

# # 复制源码中的指定的头文件到 include下
# install(FILES ${PROJECT_SOURCE_DIR}/facedetect/detect_cl_types.h DESTINATION include)

# # 复制sample代码到 sample下
# install(FILES ${PROJECT_SOURCE_DIR}/facedetect/test_detect.cpp DESTINATION sample)
