# CMakeList.txt : CMake project for multi_tracer, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

set(TARGET_NAME multi_tracer)

find_package(SDL2 REQUIRED)

add_executable (${TARGET_NAME} "multi_tracer.cpp" "multi_tracer.h" "scene.h" "random.h" "random.cpp" "scene.cpp" "fps.h" "fps.cpp" "window.h" "window.cpp")
target_include_directories(${TARGET_NAME} PRIVATE ${SDL2_INCLUDE_DIRS} ../vendor/olc)
target_link_libraries(${TARGET_NAME} ${SDL2_LIBRARIES} marl)
target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)
configure_file(${SDL2_SHARED} ${CMAKE_BINARY_DIR}/${TARGET_NAME}/${SDL2_SHARED_FILENAME} COPYONLY)
