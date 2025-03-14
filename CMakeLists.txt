cmake_minimum_required(VERSION 3.5)
project(can_test)



include_directories(
    include
    can
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3")

link_libraries(pthread)
link_directories (${PROJECT_SOURCE_DIR}/can)


# CAN收发节点
add_executable(can_node src/Tangair_usb2can.cpp src/can_node.cpp)
target_link_libraries(can_node 
pthread
usb_can
${CMAKE_THREAD_LIBS_INIT}  
${CMAKE_DL_LIBS}
)





