cmake_minimum_required(VERSION 3.15)

# Set the compiler
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# mavlink header only library
##################################################################################
# reference: https://github.com/PX4/PX4-Autopilot/blob/e835a7c4ea4bd2c8af179e37c561521a29d715d7/src/modules/mavlink/CMakeLists.txt#66
set(MAVLINK_LIBRARY_DIR mavlink_cpp)
set(MAVLINK_DIALECT common)
set(MAVLINK_LIBS mavlink_cpp)
add_library(${MAVLINK_LIBS} INTERFACE)  # INTERFACE to make it header only
target_sources(${MAVLINK_LIBS} INTERFACE ${CMAKE_SOURCE_DIR}/${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}/${MAVLINK_DIALECT}.h PROPERTIES GENERATED true)
target_include_directories(${MAVLINK_LIBS} INTERFACE 
    ${MAVLINK_LIBRARY_DIR} 
    ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT}
)
target_compile_options(${MAVLINK_LIBS} INTERFACE -Wno-address-of-packed-member -Wno-cast-align)
#################################################################################  

# webots librarys setup
##################################################################################
# set (WEBOTS_LIBS m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
set (WEBOTS_LIBS m 
Controller.dll car.dll driver.dll
CppController.dll CppCar.dll CppDriver.dll
)
# Set the  Webots home path (change it according to your installation method)
set(WEBOTS_HOME "C:/Program Files/Webots")
# Link with the Webots controller library.
link_directories($ENV{WEBOTS_HOME}/lib/controller)
include_directories(
  $ENV{WEBOTS_HOME}/include/controller/c 
  $ENV{WEBOTS_HOME}/include/controller/cpp
)
##################################################################################

# SDL library
##################################################################################
# set the output directory for built objects.
# This makes sure that the dynamic library goes into the build directory automatically.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/$<CONFIGURATION>")

# This assumes the SDL source is available in vendored/SDL
# add_subdirectory(C:/Configuration/sdl_home EXCLUDE_FROM_ALL)

include_directories(
    C:/Configuration/sdl_home/include/
)
link_directories(
    C:/Configuration/sdl_home/lib
    C:/Configuration/sdl_home/bin
)

set(SLD_LIB SDL3)
##################################################################################

# Eigen Library
##################################################################################
set(EIGEN3_INCLUDE_DIR D:/Project/Robot/libs/eigen/Eigen/)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)
##################################################################################

# Setup the project.
##################################################################################
# Its name is defined to be the controller directory name.
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT} LANGUAGES CXX)  
# message("PROJECT: " ${PROJECT})

# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})
# message("SOURCES: " ${SOURCES})

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT} 
  PRIVATE ${WEBOTS_LIBS}
  PRIVATE ${MAVLINK_LIBS}
  PRIVATE Ws2_32 # for winsock2.h
  PRIVATE ${SLD_LIB}  #SDL 3
)

# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT}.exe ${CMAKE_SOURCE_DIR}
)