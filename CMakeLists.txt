cmake_minimum_required(VERSION 3.0.2)
project(eth_data_forwarder)

set(OS "linux")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
set(OS_LIBS pthread rt)


#file(GLOB ARM_CONTROL_SOURCES ${CMAKE_CURRENT_LIST_DIR}/include/elephant_robotics_p5_soem/arm_control/*.c)
#add_library(arm_control STATIC ${ARM_CONTROL_SOURCES})
#target_link_libraries(arm_control ${OS_LIBS})

add_executable(eth_data_forwarder server/server.c)
target_include_directories(eth_data_forwarder PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
#target_link_libraries(eth_data_forwarder soem)
