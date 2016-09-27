############################################################################
#
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
# 3. Neither the name PX4 nor the names of its contributors may be
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
#
############################################################################

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(version_major 0)
set(version_minor 1)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "charlebm@gmail.com")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon")

if ("$ENV{ENABLE_TSAN}" STREQUAL "1")
message("Enabling Thread Sanitizer")

# Used for unit testing
set(CMAKE_C_COMPILER /usr/bin/clang)
set(CMAKE_CXX_COMPILER /usr/bin/clang++)

# warnings
set(warning_list
	-Wall
	-Werror
	-Wextra
	-Wno-sign-compare
	-Wshadow
	-Wfloat-equal
	-Wpointer-arith
	-Wmissing-declarations
	-Wno-unused-parameter
	-Werror=format-security
	-Werror=array-bounds
	-Wfatal-errors
	-Werror=unused-variable
	-Werror=reorder
	-Werror=uninitialized
	-Werror=init-self
	# cxx specific
	-Wno-missing-field-initializers
	)

string (REPLACE ";" " " _TMP_STR "${warning_list}")
set(warning_flags ${_TMP_STR})


set(DF_CXX_FLAGS "-fsanitize=thread -std=c++11 ${warning_flags} -g" CACHE STRING "DF_CXX_FLAGS")

elseif ("$ENV{ENABLE_ASAN}" STREQUAL "1")
message("Enabling Address Sanitizer")

# Used for unit testing, works with gcc 4.8 or clang
set(DF_CXX_FLAGS "-O1 -g -fsanitize=address -fno-omit-frame-pointer -std=c++11 ${warning_flags}" CACHE STRING "DF_CXX_FLAGS")

else()

	set(DF_CXX_FLAGS "-std=c++11 ${warning_Flags} -g" CACHE STRING "DF_CXX_FLAGS")

endif()

# Enable this directory's flags:
set(CMAKE_CXX_FLAGS "${DF_CXX_FLAGS} ${CMAKE_CXX_FLAGS}")

# Enable CI flags from travis:
if ("$ENV{CI}")
	add_definitions(-DCI)
endif()

add_definitions(-D__STDC_FORMAT_MACROS)

set(df_link_libs CACHE STRING "DF_LINK_LIBS")

# set DF_TARGET
include(cmake/df_common.cmake)

if (NOT "${DF_TARGET}" STREQUAL "qurt")

	list(APPEND df_link_libs pthread)

	if (NOT APPLE)
		list(APPEND df_link_libs rt)
	endif()
else()
	include_directories(dspal/include)
	add_subdirectory(os/qurt)
	list(APPEND df_link_libs df_qurt_stubs)

endif()

include_directories(
	framework/include
	os/qurt/include
	)

# Add framework library
add_subdirectory(framework/src)

# Add driver libraries
add_subdirectory(drivers)

if("${DF_ENABLE_TESTS}" STREQUAL "1")
	# Add unit test
	add_subdirectory(test)
endif()

message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
