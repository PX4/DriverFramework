############################################################################
# Cross-compilation Toolchain File (CMAKE_TOOLCHAIN_FILE)
# for OcPoC-Zynq-Mini
# 
# Requires Xilinx petalinux-v2015.4-final. Define an $ENV variable by adding
# the following line to .bashrc:
# export OCPOC_TOOLCHAIN_DIR="<path_to_petalinux>/petalinux-v2015.4-final/tools/linux-i386/arm-xilinx-linux-gnueabi/bin"
#
# Author: Dave Royer (dave@aerotenna.com)
#
############################################################################
add_definitions(
	-D__DF_OCPOC
	-D__DF_LINUX
)

set(DF_TARGET ocpoc)

# See notes above for setting the ENV variable in .bashrc
if ("$ENV{OCPOC_TOOLCHAIN_DIR}" STREQUAL "")
        message(FATAL_ERROR "$OCPOC_TOOLCHAIN_DIR not set")
else()
        set(OCPOC_TOOLCHAIN_DIR $ENV{OCPOC_TOOLCHAIN_DIR})
endif()

# this one is important
set(CMAKE_SYSTEM_NAME Generic)

# this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# set cross compilers
set(CMAKE_C_COMPILER "${OCPOC_TOOLCHAIN_DIR}/arm-xilinx-linux-gnueabi-gcc")
set(CMAKE_CXX_COMPILER "${OCPOC_TOOLCHAIN_DIR}/arm-xilinx-linux-gnueabi-g++")
set(CMAKE_C_FLAGS "")
set(LINKER_FLAGS "-Wl,-gc-sections")
set(CMAKE_EXE_LINKER_FLAGS ${LINKER_FLAGS})

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
