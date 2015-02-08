cmake_minimum_required(VERSION 2.6)
project(optimal)

SET(CPP_COMPILE_FLAGS "-std=c++11")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${CPP_COMPILE_FLAGS}" )

include_directories( "$ENV{EIGEN3_INCLUDE_DIR}" )
include_directories(include)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

file(GLOB lib_source "src/*.cpp")
LIST(SORT lib_source)

add_library(${PROJECT_NAME} STATIC ${lib_source})

file(GLOB tests_files "test/*.cpp")
LIST(SORT tests_files)

foreach(test_file ${tests_files})
    get_filename_component(test_base ${test_file} NAME_WE)
    message(STATUS "Adding test ${test_base}")
    add_executable(${test_base} ${test_file})
    target_link_libraries(${test_base} ${PROJECT_NAME})
endforeach(test_file)

