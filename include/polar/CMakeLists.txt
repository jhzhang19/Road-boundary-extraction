cmake_minimum_required(VERSION 2.6)
project(polar)
 
find_package(PCL 1.2 REQUIRED)
 
# 加入Boost setting
find_package(Boost COMPONENTS program_options REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
add_executable(polar polar_detect.cpp)
 
target_link_libraries (polar ${PCL_LIBRARIES} ${Boost_LIBRARIES}) #此处也有修改
 
install(TARGETS polar RUNTIME DESTINATION bin)
