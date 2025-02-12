cmake_minimum_required(VERSION 3.0.2)
project(eth_data_forwarder)

set(OS "linux")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -Werror")
set(OS_LIBS pthread rt)


file(GLOB CLIENT_API_SOURCES ${CMAKE_CURRENT_LIST_DIR}/client_api/*.c)
add_library(client_api STATIC ${CLIENT_API_SOURCES})
target_include_directories(client_api PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(client_api ${OS_LIBS})

add_executable(eth_data_forwarder server/server.c)
target_include_directories(eth_data_forwarder PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_link_libraries(eth_data_forwarder ${OS_LIBS})

add_executable(eth_data_client client/client.c)
target_include_directories(eth_data_client PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(eth_data_client PUBLIC ${CMAKE_CURRENT_LIST_DIR}/client_api)
target_link_libraries(eth_data_client client_api)

add_executable(can_client can_client/can_client.c)
target_include_directories(can_client PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(can_client PUBLIC ${CMAKE_CURRENT_LIST_DIR}/client_api)
target_link_libraries(can_client client_api)

add_executable(uart_client uart_client/uart_client.c)
target_include_directories(uart_client PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(uart_client PUBLIC ${CMAKE_CURRENT_LIST_DIR}/client_api)
target_link_libraries(uart_client client_api)

add_executable(imu_client imu_client/imu_client.c)
target_include_directories(imu_client PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)
target_include_directories(imu_client PUBLIC ${CMAKE_CURRENT_LIST_DIR}/client_api)
target_link_libraries(imu_client client_api)
