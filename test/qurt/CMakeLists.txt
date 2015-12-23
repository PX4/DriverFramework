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

if ("${HEXAGON_TOOLS_ROOT}" STREQUAL "")
	message( FATAL_ERROR "HEXAGON_TOOLS_ROOT not set")
endif()

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message( FATAL_ERROR "HEXAGON_SDK_ROOT not set")
endif()

set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})

include_directories(
	${CMAKE_CURRENT_BINARY_DIR}
	${HEXAGON_SDK_ROOT}/inc/stddef
	${HEXAGON_SDK_ROOT}/lib/common/remote/ship/hexagon_Debug
	)

include (CMakeParseArguments)

function(BUILD_QURT_APP)
	set(options)
	set(oneValueArgs APP_NAME ARM_TOOLCHAIN)
	set(multiValueArgs APPS_SOURCES APPS_LINK_LIBS DSP_SOURCES DSP_LINK_LIBS)
	cmake_parse_arguments(BUILD_QURT_APP "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN} )

	message("APP_NAME = ${BUILD_QURT_APP_APP_NAME}")

	add_custom_command(
		OUTPUT ${BUILD_QURT_APP_APP_NAME}.h ${BUILD_QURT_APP_APP_NAME}_skel.c ${BUILD_QURT_APP_APP_NAME}_stub.c
		DEPENDS ${BUILD_QURT_APP_APP_NAME}.idl
		COMMAND "${HEXAGON_SDK_ROOT}/tools/qaic/Ubuntu14/qaic" "-mdll" "-I" "${HEXAGON_SDK_ROOT}/inc/stddef" "${CMAKE_CURRENT_SOURCE_DIR}/${BUILD_QURT_APP_APP_NAME}.idl"
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		)

	add_custom_target(generate_${BUILD_QURT_APP_APP_NAME}_stubs ALL
		DEPENDS ${BUILD_QURT_APP_APP_NAME}.h ${BUILD_QURT_APP_APP_NAME}_skel.c ${BUILD_QURT_APP_APP_NAME}_stub.c
		)

	set_source_files_properties(
		${BUILD_QURT_APP_APP_NAME}.h
		${BUILD_QURT_APP_APP_NAME}_skel.c
		${BUILD_QURT_APP_APP_NAME}_stub.c
		PROPERTIES
		GENERATED TRUE
		)

	add_library(${BUILD_QURT_APP_APP_NAME} SHARED
		${BUILD_QURT_APP_DSP_SOURCES}
		)

	add_dependencies(${BUILD_QURT_APP_APP_NAME} generate_${BUILD_QURT_APP_APP_NAME}_stubs)

	add_library(${BUILD_QURT_APP_APP_NAME}_skel SHARED
		${BUILD_QURT_APP_APP_NAME}_skel.c
		)

	file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/apps)

	set(${APP_APP_NAME}_INCLUDE_DIRS 
		-I${CMAKE_CURRENT_BINARY_DIR}
		-I${HEXAGON_SDK_ROOT}/inc/stddef
		-I${HEXAGON_SDK_ROOT}/lib/common/remote/ship/UbuntuARM_Debug
		)
	set(${APP_APP_NAME}_LINK_DIRS -L${HEXAGON_SDK_ROOT}/lib/common/remote/ship/UbuntuARM_Debug -ladsprpc)
	add_custom_command(
		OUTPUT ${BUILD_QURT_APP_APP_NAME}_app
		DEPENDS generate_${BUILD_QURT_APP_APP_NAME}_stubs
		COMMAND "arm-linux-gnueabihf-gcc" ${${APP_APP_NAME}_INCLUDE_DIRS} -o ${CMAKE_CURRENT_BINARY_DIR}/${BUILD_QURT_APP_APP_NAME}_app ${BUILD_QURT_APP_APPS_SOURCES} "${CMAKE_CURRENT_BINARY_DIR}/${BUILD_QURT_APP_APP_NAME}_stub.c" ${${APP_APP_NAME}_LINK_DIRS}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		)

	add_custom_target(build_${BUILD_QURT_APP_APP_NAME}_apps ALL
		DEPENDS ${BUILD_QURT_APP_APP_NAME}_app ${BUILD_QURT_APP_APP_NAME}_stub.c
		)

	add_dependencies(${BUILD_QURT_APP_APP_NAME}_skel generate_${BUILD_QURT_APP_APP_NAME}_stubs build_${BUILD_QURT_APP_APP_NAME}_apps)

endfunction()

build_qurt_app(APP_NAME df_testapp
	APPS_SOURCES df_testapp.cpp
	DSP_SOURCES df_testapp_dsp.cpp
	DSP_LINK_LIBS df_driver_framework ${df_driver_libs} ${df_link_libs}
	ARM_TOOLCHAIN ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake
	)

# vim: set noet fenc=utf-8 ff=unix ft=cmake :
