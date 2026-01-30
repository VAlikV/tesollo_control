cmake_minimum_required(VERSION 3.10)

set(PROJECT DG_CONTROL)

project(${PROJECT} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(GCC_COVERAGE_COMPILE_FLAGS "-W -fexceptions -O2 -DNDEBUG -Dlinux")

find_library(DGSDK_LIB DGSDK NAMES libDGSDK.so -libDGSDK HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

set(CONTROL ${CMAKE_CURRENT_SOURCE_DIR}/dg_control)
set(LOCKFREE ${CMAKE_CURRENT_SOURCE_DIR}/lockfree)
set(UDP ${CMAKE_CURRENT_SOURCE_DIR}/udp)

add_executable(${PROJECT}
    # developer.cpp
    main.cpp
    ${CONTROL}/control.cpp      ${CONTROL}/control.hpp
    ${LOCKFREE}/lockfree.hpp
    ${UDP}/udp_server.cpp       ${UDP}/udp_server.hpp       ${UDP}/json.hpp 
)

target_link_libraries(${PROJECT}
    ${DGSDK_LIB}
    Threads::Threads
    Eigen3::Eigen
)

set_target_properties(${PROJECT} PROPERTIES
    BUILD_RPATH "$ORIGIN"
)