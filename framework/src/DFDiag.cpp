/**********************************************************************
* Copyright (c) 2016 Mark Charlebois
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

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include "DFLog.hpp"
#include "DFDiag.hpp"
#include "DevMgr.hpp"

using namespace DriverFramework;

void DFDiag::listRegisteredDevices()
{
	unsigned int index = 0;
	const char *name;

	DF_LOG_INFO("Registered Devices:");

	while (DevMgr::getNextDeviceName(index, &name) == 0) {
		DF_LOG_INFO("   '%s'", name);
	}
}

void DFDiag::listRawDevices()
{
	int fd;
	char devname[15];

	DF_LOG_INFO("I2C devices:");

	for (unsigned int i = 0; i < 8; i++) {
#ifdef __DF_QURT
		sprintf(devname, "/dev/iic-%u", i);
#else
		sprintf(devname, "/dev/i2c-%u", i);
#endif
		fd = ::open(devname, O_RDONLY);

		if (fd != -1) {
			DF_LOG_INFO("  %s", devname);
			close(fd);

		} else if (errno == EACCES) {
			DF_LOG_INFO("  %s (WARNING access denied)", devname);
		}
	}

	DF_LOG_INFO("SPI devices:");

	for (unsigned int i = 0; i < 8; i++) {
		sprintf(devname, "/dev/spi-%u", i);
		fd = ::open(devname, O_RDONLY);

		if (fd != -1) {
			DF_LOG_INFO("  %s", devname);
			close(fd);
		}
	}
}
