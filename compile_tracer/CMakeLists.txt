cmake_minimum_required (VERSION 3.8)

set(TARGET_NAME compile_tracer)
set(CONST_STEPS 536870912)

add_executable (${TARGET_NAME} "ctracer.cpp")
target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)
target_compile_options(${TARGET_NAME} PRIVATE $<IF:$<CXX_COMPILER_ID:MSVC>,/constexpr:steps${CONST_STEPS},-fconstexpr-ops-limit=${CONST_STEPS}>)