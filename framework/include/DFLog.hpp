/**********************************************************************
* Copyright (c) 2015 Mark Charlebois
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the
* disclaimer below) provided that the following conditions are met:
*
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*  * Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the
*    distribution.
*
*  * Neither the name of Dronecode Project nor the names of its
*    contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
* GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
* HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*************************************************************************/
#pragma once

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

namespace DriverFramework
{

uint64_t offsetTime(void);

}

#define DF_INFO 0

#ifdef __DF_QURT

#include <cstdarg>

extern "C" {

// declaration to make the compiler happy.  This symbol is part of the DSP static image.
	void HAP_debug(const char *msg, int level, const char *filename, int line);

};

static __inline void qurt_log_2(int level, const char *file, int line, const char *format, ...)
{
	char buf[256];
	va_list args;
	va_start(args, format);
	vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	HAP_debug(buf, level, file, line);
}

#define DF_LOG_INFO(FMT, ...) qurt_log_2(0, __FILE__, __LINE__, "%" PRIu64 " " FMT  "\n", offsetTime(), ##__VA_ARGS__)
#define DF_LOG_ERR(FMT, ...)  qurt_log_2(0, __FILE__, __LINE__, "%" PRIu64 " " FMT  "\n", offsetTime(), ##__VA_ARGS__)

#if DF_DEBUG
#define DF_LOG_DEBUG(FMT, ...)  qurt_log_2(1, __FILE__, __LINE__, "%" PRIu64 " " FMT  "\n", offsetTime(), ##__VA_ARGS__)
#else
#define DF_LOG_DEBUG(FMT, ...)
#endif

#elif defined(__DF_NUTTX)

// TODO: Substitute logging implementation here

// TODO: NuttX doesn't support the PRIuN defines
// TODO: The NuttX build doesn't link to offsetTime() yet
#define DF_LOG_INFO(FMT, ...) printf(FMT  "\n", ##__VA_ARGS__)
#define DF_LOG_ERR(FMT, ...)  printf(FMT "\n", ##__VA_ARGS__)

#if DF_DEBUG
#define DF_LOG_DEBUG(FMT, ...)  printf(FMT "\n", ##__VA_ARGS__)
#else
#define DF_LOG_DEBUG(FMT, ...)
#endif

#else

// TODO: Substitute logging implementation here
#define DF_LOG_INFO(FMT, ...) printf("%" PRIu64 " " FMT  "\n", offsetTime(), ##__VA_ARGS__)
#define DF_LOG_ERR(FMT, ...)  printf("%" PRIu64 " " FMT "\n", offsetTime(), ##__VA_ARGS__)

#if DF_DEBUG
#define DF_LOG_DEBUG(FMT, ...)  printf("%" PRIu64 " " FMT "\n", offsetTime(), ##__VA_ARGS__)
#else
#define DF_LOG_DEBUG(FMT, ...)
#endif

#endif

