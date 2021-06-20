
cmake_minimum_required ( VERSION 2.6 FATAL_ERROR)   #对于cmake版本的最低版本的要求
project(map_label)                                        #建立的工程名，例如源代码目录路径的变量名为CH_DIR
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED)     

 # 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
         
find_package(catkin REQUIRED COMPONENTS
        message_generation
        std_msgs
        sensor_msgs
        roscpp
        pcl_conversions
        pcl_ros
        cv_bridge
        )

find_package(OpenCV REQUIRED)
include_directories(${OpenCV})


include_directories(${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(map_label main.cpp)
target_link_libraries(map_label ${catkin_LIBRARIES} ${OpenCV_LIBS}        #注意此处添加lib,都要添加
        ${roslib_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} 
        ${PCL_VISUALIZATION_LIBRARIES} ${PCL_REGISTRATION_LIBRARIES} )
