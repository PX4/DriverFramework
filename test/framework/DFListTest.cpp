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
#include "DFListTest.hpp"

bool DFListTest::verifyStructList(DFManagedList<testStr> &pl, const char *expectedOrder)
{
	bool passed = true;
	DFPointerList::Index idx = nullptr;
	idx = pl.next(idx);
	unsigned int i = 0;

	while (idx != nullptr) {
		testStr *val = pl.get(idx);

		if (val->msg[0] != expectedOrder[i]) {
			DF_LOG_INFO("Expected %c got %c", expectedOrder[i], val->msg[0]);
			passed = false;
		}

		idx = pl.next(idx);
		++i;
	}

	return passed;
}

bool DFListTest::verifyList(DFPointerList &pl, const char *expectedOrder)
{
	bool passed = true;
	DFPointerList::Index idx = nullptr;
	idx = pl.next(idx);
	unsigned int i = 0;

	while (idx != nullptr) {
		const char *val = static_cast<const char *>(pl.get(idx));

		if (val[0] != expectedOrder[i]) {
			DF_LOG_INFO("Expected %c got %c", expectedOrder[i], val[0]);
			passed = false;
		}

		idx = pl.next(idx);
		++i;
	}

	return passed;
}

bool DFListTest::verifyUIntList(DFUIntList &pl, unsigned int check_list[4])
{
	bool passed = true;
	DFUIntList::Index idx = nullptr;
	idx = pl.next(idx);
	unsigned int i = 0;

	while (idx != nullptr) {
		unsigned int val;
		bool ret = pl.get(idx, val);

		if (ret == false) {
			DF_LOG_INFO("get() failed");
			passed = false;
		}

		if (val != check_list[i]) {
			DF_LOG_INFO("Expected %c got %c", check_list[i], val);
			passed = false;
		}

		idx = pl.next(idx);
		++i;
	}

	return passed;
}

void DFListTest::managedListTest()
{
	startFeatureTest("Managed list tests");
	DFManagedList<testStr> pl;
	testStr *msg1 = new testStr(1);
	testStr *msg2 = new testStr(2);
	testStr *msg3 = new testStr(3);
	testStr *msg4 = new testStr(4);
	testStr *msg0 = new testStr(0);

	reportResult("Verify empty() true on list creation", pl.empty());

	pl.pushBack(msg1);
	pl.pushBack(msg2);
	pl.pushBack(msg3);
	pl.pushBack(msg4);

	reportResult("Verify pushBack()", verifyStructList(pl, "1234"));

	DFPointerList::Index idx = nullptr;

	idx = pl.next(idx);
	idx = pl.next(idx);
	idx = pl.next(idx);

	// idx points to node containing msg3
	idx = pl.erase(idx);

	pl.pushFront(msg0);

	reportResult("Verify next() pushFront() and erase()", verifyStructList(pl, "0124"));

	pl.clear();
	reportResult("Verify clear()", pl.empty());
}

void DFListTest::unmanagedListTest()
{
	startFeatureTest("Unmanaged list tests");
	DFPointerList pl;
	const char *msg1 = "1";
	const char *msg2 = "2";
	const char *msg3 = "3";
	const char *msg4 = "4";
	const char *msg0 = "0";

	reportResult("Verify empty() true on list creation", pl.empty());

	pl.pushBack((void *)msg1);
	pl.pushBack((void *)msg2);
	pl.pushBack((void *)msg3);
	pl.pushBack((void *)msg4);

	reportResult("Verify pushBack()", verifyList(pl, "1234"));

	DFPointerList::Index idx = nullptr;

	idx = pl.next(idx);
	idx = pl.next(idx);
	idx = pl.next(idx);

	// idx points to node containing msg3
	idx = pl.erase(idx);

	pl.pushFront((void *)msg0);

	reportResult("Verify next() pushFront() and erase()", verifyList(pl, "0124"));

	pl.clear();
	reportResult("Verify clear()", pl.empty());
}

void DFListTest::uintListTest()
{
	startFeatureTest("UInt list tests");
	DFUIntList pl;
	unsigned int val1 = 1;
	unsigned int val2 = 2;
	unsigned int val3 = 3;
	unsigned int val4 = 4;
	unsigned int val0 = 0;

	reportResult("Verify empty() true on list creation", pl.empty());

	unsigned int check_list1[] = { 1, 2, 3, 4 };
	pl.pushBack(val1);
	pl.pushBack(val2);
	pl.pushBack(val3);
	pl.pushBack(val4);

	reportResult("Verify pushBack()", verifyUIntList(pl, check_list1));

	DFUIntList::Index idx = nullptr;

	idx = pl.next(idx);
	idx = pl.next(idx);
	idx = pl.next(idx);

	// idx points to node containing msg3
	idx = pl.erase(idx);

	pl.pushFront(val0);

	unsigned int check_list2[] = { 0, 1, 2, 4 };
	reportResult("Verify next() pushFront() and erase()", verifyUIntList(pl, check_list2));

	pl.clear();
	reportResult("Verify clear()", pl.empty());
}

void DFListTest::_doTests()
{
	unmanagedListTest();
	managedListTest();
	uintListTest();
}

