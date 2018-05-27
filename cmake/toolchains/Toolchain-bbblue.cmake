############################################################################
# Cross-compilation Toolchain File (CMAKE_TOOLCHAIN_FILE)
# for BeagleBone Blue
#
############################################################################
add_definitions(
	-D__DF_BBBLUE
	-D__DF_LINUX
)

#optional __DF_BBBLUE_USE_RC_BMP280_IMP
add_definitions(
	-D__DF_BBBLUE_USE_RC_BMP280_IMP
)

######### test DriverFramework for bbblue ###
# used for debug
#add_definitions(-DDF_DEBUG)

if ("${BBBLUE_TOOLCHAIN_DIR}" STREQUAL "")
	set(BBBLUE_TOOLCHAIN_DIR /opt/bbblue_toolchain)
endif()

# this one is important
set(CMAKE_SYSTEM_NAME Linux)

# this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# set cross compilers
set(CMAKE_C_COMPILER   "${BBBLUE_TOOLCHAIN_DIR}/gcc-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc")
set(CMAKE_CXX_COMPILER "${BBBLUE_TOOLCHAIN_DIR}/gcc-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++")
set(CMAKE_C_FLAGS "")
set(LINKER_FLAGS "-Wl,-gc-sections")
set(CMAKE_EXE_LINKER_FLAGS ${LINKER_FLAGS})

# search for programs in the build host directories
SET(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
SET(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
SET(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
