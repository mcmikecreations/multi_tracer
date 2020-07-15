cmake_minimum_required (VERSION 3.8)

project ("multi_tracer")

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/vendor/modules")
if (WIN32)
	set(SDL2_DIR "${PROJECT_SOURCE_DIR}/vendor/sdl2/win")
	get_filename_component(SDL2_DLL_LOCATION_TEMP ${SDL2_LIBRARY} DIRECTORY)
	set(SDL2_SHARED_FILENAME SDL2.dll)
	set(SDL2_SHARED ${SDL2_DLL_LOCATION_TEMP}/${SDL2_SHARED_FILENAME})
endif(WIN32)

add_subdirectory(vendor/marl)

add_subdirectory("multi_tracer")
add_subdirectory("compile_tracer")