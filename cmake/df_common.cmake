#
# Copyright (C) 2015 Mark Charlebois. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#	notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#	notice, this list of conditions and the following disclaimer in
#	the documentation and/or other materials provided with the
#	distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#	used to endorse or promote products derived from this software
#	without specific prior written permission.
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

# Set DF_TARGET from OS and add any other platform detection logic

if ("${DF_TARGET}" STREQUAL "")
	if ("${OS}" STREQUAL "")
		message(FATAL_ERROR "OS not defined")
	endif()

	if("${OS}" STREQUAL "posix")
		if (APPLE)
			set(DF_TARGET darwin)

			if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
				message(FATAL_ERROR "PX4 Firmware requires XCode 8 or newer on Mac OS. Version installed on this system: ${CMAKE_CXX_COMPILER_VERSION}")
			endif()

			EXEC_PROGRAM(uname ARGS -v  OUTPUT_VARIABLE DARWIN_VERSION)
			STRING(REGEX MATCH "[0-9]+" DARWIN_VERSION ${DARWIN_VERSION})
			# message(STATUS "DF Darwin Version: ${DARWIN_VERSION}")
			if (DARWIN_VERSION LESS 16)
				add_definitions(
					-DCLOCK_MONOTONIC=1
					-D__DF_APPLE_LEGACY
					)
			endif()
		else()
			set(DF_TARGET linux)
		endif()
	else()
		set(DF_TARGET ${OS})
	endif()
endif()

function (df_add_library df_library_name)
	set(args "${ARGN}")
	add_library (${df_library_name} ${args})

	if ("${DF_TARGET}" STREQUAL "qurt")
		set_property(TARGET ${df_library_name} PROPERTY POSITION_INDEPENDENT_CODE TRUE)
	endif()
endfunction ()
