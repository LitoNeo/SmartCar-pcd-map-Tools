cmake_minimum_required(VERSION 2.8.3)

project(utils)
add_compile_options(-std=c++11)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)  # 如果要声明某一类型,使用COMPONENTS thread
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  nav_msgs
)
# link_directories(${PROJECT_SOURCE_DIR}/lib)  # 把第三方库的.a/.so 及.h文件放到lib目录下

catkin_package(
        # INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}                   #The exported libraries from the project
        DEPENDS PCL
        CATKIN_DEPENDS pcl_ros roscpp rospy nav_msgs
)

include_directories(   # # 去哪里找头文件
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        # "include"
)
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})     # 去哪里找库文件
add_definitions(${PCL_DEFINITIONS})

# 设置可执行文件生成路径
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../bin)


# # 生成debug版本
SET(CMAKE_BUILD_TYPE "Release") 
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")


set(CMAKE_INCLUDE_CURRENT_DIR ON)

SET(LIBUTILS_SRC common.h common.cpp)
ADD_LIBRARY(${PROJECT_NAME} SHARED ${LIBUTILS_SRC})
target_link_libraries(${PROJECT_NAME} 
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    )
SET_TARGET_PROPERTIES (${PROJECT_NAME} PROPERTIES VERSION 1.0 SOVERSION 1)

