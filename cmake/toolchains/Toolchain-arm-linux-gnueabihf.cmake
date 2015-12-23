############################################################################
# Copyright (c) 2015 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name ATLFlight nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
############################################################################

# defines:
#
# C_COMPILER
# CMAKE_SYSTEM_NAME
# CMAKE_SYSTEM_VERSION
# LINKER_FLAGS
# CMAKE_EXE_LINKER_FLAGS
# CMAKE_FIND_ROOT_PATH
# CMAKE_FIND_ROOT_PATH_MODE_PROGRAM
# CMAKE_FIND_ROOT_PATH_MODE_LIBRARY
# CMAKE_FIND_ROOT_PATH_MODE_INCLUDE
# HEXAGON_SDK_ROOT

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR "HEXAGON_SDK_ROOT not set")
endif()

set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})

include(CMakeForceCompiler)

# this one is important
set(CMAKE_SYSTEM_NAME Generic)

#this one not so much
set(CMAKE_SYSTEM_VERSION 1)

# specify the cross compiler
find_program(C_COMPILER arm-linux-gnueabihf-gcc-5)
if(NOT C_COMPILER)
	find_program(C_COMPILER arm-linux-gnueabihf-gcc-4.9)
	if(NOT C_COMPILER)
		find_program(C_COMPILER arm-linux-gnueabihf-gcc-4.8)
		if(NOT C_COMPILER)
			find_program(C_COMPILER arm-linux-gnueabihf-gcc)
			if(NOT C_COMPILER)
				message(FATAL_ERROR "could not find arm-linux-gnueabihf-gcc compiler")
			endif()
		endif()
	endif()
endif()
cmake_force_c_compiler(${C_COMPILER} GNU)

include_directories(
	${HEXAGON_SDK_ROOT}/inc/stddef
	${HEXAGON_SDK_ROOT}/lib/common/remote/ship/UbuntuARM_Debug
	)

set(CMAKE_C_FLAGS "")
set(LINKER_FLAGS "-Wl,-gc-sections")
set(CMAKE_EXE_LINKER_FLAGS ${LINKER_FLAGS})

# where is the target environment 
set(CMAKE_FIND_ROOT_PATH  get_file_component(${C_COMPILER} PATH))

# search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# for libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
