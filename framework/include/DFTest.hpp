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

#include "DFLog.hpp"

namespace DriverFramework
{

class DFTestObj
{
public:
	DFTestObj(const char *name) : m_name(name) {}

	virtual ~DFTestObj() {}

	bool doTests()
	{
		testStart();
		_doTests();
		testEnd();
		return (m_failed_count == 0);
	}

protected:
	virtual void _doTests() = 0;

	void reportResult(const char *name, bool passed)
	{
		DF_LOG_INFO("TEST %s: %s", name, passed ? "PASSED" : "FAILED");

		if (passed) {
			++m_passed_count;

		} else {
			++m_failed_count;
		}
	}

	void startFeatureTest(const char *name)
	{
		DF_LOG_INFO("------- BEGIN FEATURE TEST FOR %s", name);
	}

private:
	void testStart()
	{
		DF_LOG_INFO("======= BEGIN TESTS FOR %s", m_name);
		m_passed_count = 0;
		m_failed_count = 0;
	}

	void testEnd()
	{
		DF_LOG_INFO("======= END OF TESTS FOR %s. %u of %u tests passed", m_name, m_passed_count,
			    m_passed_count + m_failed_count);
	}

	const char *m_name;

	unsigned int 	m_passed_count = 0;
	unsigned int 	m_failed_count = 0;
};
};
