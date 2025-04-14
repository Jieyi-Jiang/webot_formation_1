cmake_minimum_required(VERSION 3.10)

set(PROJECT_1 server)
project(${PROJECT_1} LANGUAGES CXX)
set(PROJECT_2 client)
project(${PROJECT_2} LANGUAGES CXX)

add_executable(${PROJECT_1} server.cpp)
target_link_libraries(${PROJECT_1} PRIVATE ws2_32)  # PRIVATE - 该库只对当前目标有用，不会影响其他目标

add_executable(${PROJECT_2} client.cpp)
target_link_libraries(${PROJECT_2} PRIVATE ws2_32)  # PRIVATE - 该库只对当前目标有用，不会影响其他目标

add_custom_command(TARGET ${PROJECT_2} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT_2}.exe ${CMAKE_SOURCE_DIR})

add_custom_command(TARGET ${PROJECT_1} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
copy ${CMAKE_BINARY_DIR}/${PROJECT_1}.exe ${CMAKE_SOURCE_DIR}

)