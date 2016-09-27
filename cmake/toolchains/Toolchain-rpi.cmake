############################################################################
# Cross-compilation Toolchain File (CMAKE_TOOLCHAIN_FILE)
# for Raspberry Pi 2
#     requires a proper toolchain setup:
#     https://github.com/pixhawk/rpi_toolchain
#
# Author: Bo Liu (bo-rc@acm.org)
#
############################################################################
add_definitions(
	-D__DF_RPI
	-D__DF_LINUX
)

######### test DriverFramework for rpi ###
# used for debug
#add_definitions(-DDF_DEBUG)

if ("${RPI_TOOLCHAIN_DIR}" STREQUAL "")
	set(RPI_TOOLCHAIN_DIR /opt/rpi_toolchain)
endif()

# this one is important
set(CMAKE_SYSTEM_NAME Linux)

# this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# set cross compilers
set(CMAKE_C_COMPILER "${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin/arm-linux-gnueabihf-g++")
set(CMAKE_C_FLAGS "")
set(LINKER_FLAGS "-Wl,-gc-sections")
set(CMAKE_EXE_LINKER_FLAGS ${LINKER_FLAGS})

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
