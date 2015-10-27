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
#include <string>

class DriverObj
{
public:
	DriverObj(string name) : 
		m_name(name)
	{}

	~DriverObj() {}

	virtual int initialize() = 0;

	virtual int ioctl(int datatype, void *data) = 0;

	const string &getName()
	{
		return m_name;
	}
private:
	string m_name;
	unsigned int id;
};

class I2CDriverObj : public DriverObj
{
public:
	I2CDriverObj(string name, string path, int flags) : 
		DriverObj(name),
		m_path(path),
		m_flags(flags)
	{}

	~DriverObj() {}

	virtual int initialize()
	{
		m_fd = open(m_name.c_str, m_flags);

		return (m_fd >= 0);
	}

	virtual int ioctl(int datatype, void *data)
	{
		return ::ioctl(m_fd, datatype, data);
	}
	
private:
	string m_path;
	int m_flags;
	int m_fd;
};

class VirtDriverObj : public DriverObj
{
public:
	VirtDriverObj(string name, string path) : 
		DriverObj(name),
		m_path(path)
	{}

	~VirtDriverObj() {}

	virtual int ioctl(int datatype, void *data) = 0;
};

class DriverMgr
{    
	static DriverObj *getDriverObjByName(const string &name, unsigned int instance);
	static DriverObj *getDriverObjByID(unsigned long id);

private:
	DriverMgr() {}
	~DriverMgr() {}
};

