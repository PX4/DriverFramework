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
#include <unistd.h>
#include "DevMgrTest.hpp"

static void setNotify(void *arg)
{
	DevObj *p = reinterpret_cast<DevObj *>(arg);

	DevMgr::updateNotify(*p);
}

bool DevMgrTest::verifyStart(TestDriver &test)
{
	// Register the driver
	int ret = test.init();

	if (ret < 0) {
		DF_LOG_INFO("init() failed (%d))", ret);
		return false;
	}

	// Start the driver
	ret = test.start();

	if (ret < 0) {
		DF_LOG_INFO("start() failed (%d)", ret);
		return false;
	}

	usleep(1000);
	return true;
}

bool DevMgrTest::verifyRegisterDriver()
{
	unsigned int index = 0;
	const char *instname = nullptr;
	bool found = false;

	char devname[strlen(TEST_DRIVER_CLASS_PATH) + 3];
	snprintf(devname, sizeof(devname), "%s%d", TEST_DRIVER_CLASS_PATH, 0);

	while (DevMgr::getNextDeviceName(index, &instname) == 0) {
		if (instname && (strcmp(instname, devname) == 0)) {
			found = true;

		} else {
			DF_LOG_INFO("Found device '%s'", instname == nullptr ? "undefined" : instname);
		}
	}

	if (!found) {
		DF_LOG_INFO("Device '%s' was not found", devname);
		return false;
	}

	return true;

}

bool DevMgrTest::verifyUpdateNotify()
{
	char devname[strlen(TEST_DRIVER_CLASS_PATH) + 3];
	snprintf(devname, sizeof(devname), "%s%d", TEST_DRIVER_CLASS_PATH, 0);

	DevHandle h;
	DevMgr::getHandle(devname, h);

	if (!h.isValid()) {
		DF_LOG_INFO("Failed to open %s (%d)", devname, h.getError());
		return false;
	}

	TestDriver *p = DevMgr::getDevObjByHandle<TestDriver>(h);

	if (p == nullptr) {
		DF_LOG_INFO("Failed to get dev obj from handle");
		return false;
	}

	// start task to write to device
	WorkHandle wh;
	WorkMgr::getWorkHandle(setNotify, p, 10, wh);

	UpdateList in_set, out_set;

	in_set.pushBack(&h);

	// Blocking read
	int ret = DevMgr::waitForUpdate(in_set, out_set, 0);

	if (ret != 0 && out_set.size() != 1) {
		DF_LOG_INFO("Failed to set handle error");
	}

	UpdateList::Index index = nullptr;
	index = out_set.next(index);
	DevHandle *ph = reinterpret_cast<DevHandle *>(out_set.get(index));

	// Verify that the handle returned was the handle passed in
	if (ph != &h) {
		DF_LOG_INFO("handle address incorrect");
		return false;
	}

	DevMgr::releaseHandle(h);

	// Release an already released handle
	DevMgr::releaseHandle(h);

	DevMgr::setDevHandleError(h, 1);

	if (h.getError() != 1) {
		DF_LOG_INFO("Failed to set handle error");
		return false;
	}

	return true;
}

void DevMgrTest::_doTests()
{
	TestDriver test;

	reportResult("Verify start()", verifyStart(test));
	reportResult("Verify registerDriver()", verifyRegisterDriver());
	reportResult("Verify updateNotify()", verifyUpdateNotify());

	test.stop();
}
