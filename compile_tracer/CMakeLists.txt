cmake_minimum_required (VERSION 3.8)

set(TARGET_NAME compile_tracer)

add_executable (${TARGET_NAME} "ctracer.cpp")
target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)