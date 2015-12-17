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

include_directories(
	./
	)

if("${DF_TARGET}" STREQUAL "qurt")

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

	add_custom_command(
		OUTPUT df_testapp.h df_testapp_skel.c df_testapp_stub.c
		DEPENDS qurt/df_testapp.idl
		COMMAND "${HEXAGON_SDK_ROOT}/tools/qaic/Ubuntu14/qaic" "-mdll" "-I" "${HEXAGON_SDK_ROOT}/inc/stddef" "${CMAKE_SOURCE_DIR}/test/qurt/df_testapp.idl"
		)

	add_custom_target(generate_stubs
		DEPENDS df_testapp.h df_testapp_skel.c df_testapp_stub.c
		)

	add_library(df_testapp SHARED
		qurt/main.cpp
		test.cpp
		listtest.cpp
		)

	add_dependencies(df_testapp generate_stubs)

	add_library(df_testapp_skel SHARED
		df_testapp_skel.c
		)

	add_dependencies(df_testapp_skel generate_stubs)

else()
	add_executable(df_testapp
		main.cpp
		test.cpp
		listtest.cpp
		)
endif()

target_link_libraries(df_testapp
	df_driver_framework
	${df_driver_libs}
	${df_link_libs}
	)
# vim: set noet fenc=utf-8 ff=unix ft=cmake :
