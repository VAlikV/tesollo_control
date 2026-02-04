cmake_minimum_required(VERSION 3.10)

set(CORE DG_CORE)
set(PROJECT DG_CONTROL)
set(PYTHON_PACKAGE dg5f_python)

project(${PROJECT} VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(GCC_COVERAGE_COMPILE_FLAGS "-W -fexceptions -O2 -DNDEBUG -Dlinux")

find_library(DGSDK_LIB DGSDK NAMES libDGSDK.so -libDGSDK HINTS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pybind11 REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/include)

set(CONTROL ${CMAKE_CURRENT_SOURCE_DIR}/dg_control)
set(LOCKFREE ${CMAKE_CURRENT_SOURCE_DIR}/lockfree)
set(UDP ${CMAKE_CURRENT_SOURCE_DIR}/udp)
set(POSES ${CMAKE_CURRENT_SOURCE_DIR}/poses)

add_library(${CORE}
    ${CONTROL}/control.cpp      ${CONTROL}/control.hpp
    ${LOCKFREE}/lockfree.hpp
    ${UDP}/udp_server.cpp       ${UDP}/udp_server.hpp       ${UDP}/json.hpp 
    ${POSES}/poses.hpp
)

set_target_properties(${CORE} PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(${CORE}
    ${DGSDK_LIB}
    Threads::Threads
    Eigen3::Eigen
)

add_executable(${PROJECT}
    main.cpp   
)

pybind11_add_module(${PYTHON_PACKAGE}
    python/wrapper.cpp
)


target_link_libraries(${PROJECT}
    ${CORE}
)

target_link_libraries(${PYTHON_PACKAGE}
    PRIVATE
    ${CORE}
)

set_target_properties(${PROJECT} PROPERTIES
    BUILD_RPATH "$ORIGIN"
)

set_target_properties(${PYTHON_PACKAGE} PROPERTIES
    BUILD_RPATH "$ORIGIN"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)