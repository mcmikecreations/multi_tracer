# CMakeList.txt : CMake project for multi_tracer, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

set(TARGET_NAME multi_tracer)

find_package(SDL2 REQUIRED)

add_executable (${TARGET_NAME} "multi_tracer.cpp" "multi_tracer.h")
target_include_directories(${TARGET_NAME} PRIVATE ${SDL2_INCLUDE_DIRS})
target_link_libraries(${TARGET_NAME} ${SDL2_LIBRARIES})
configure_file(${SDL2_SHARED} ${CMAKE_BINARY_DIR}/${TARGET_NAME}/${SDL2_SHARED_FILENAME} COPYONLY)
