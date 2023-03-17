cmake_minimum_required(VERSION 3.20)

set(CMAKE_TOOLCHAIN_FILE "armcm4.cmake")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")

project(GD32C10x C ASM)

set(LD_FLAGS "-T${PROJECT_SOURCE_DIR}/gd32e10x.ld")

add_definitions(
	-DUSE_STDPERIPH_DRIVER
)

add_subdirectory(GD32C10x)
add_subdirectory(src)