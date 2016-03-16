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

#include "DisableCopy.hpp"
#include "SyncObj.hpp"

namespace DriverFramework
{


class DFPointerList : public DisableCopy
{
public:
	class DFListNode;

	typedef DFListNode *Index;

	class DFListNode
	{
	public:
		DFListNode(void *item);
		~DFListNode();

		Index	m_next;
		void 	*m_item;
	};

	DFPointerList();

	virtual ~DFPointerList();

	unsigned int size();

	bool pushBack(void *item);

	bool pushFront(void *item);

	virtual Index erase(Index idx);

	virtual void clear();

	bool empty();

	Index next(Index &idx);

	void *get(Index idx);

	// Caller must lock before using list
	SyncObj m_sync;

private:

	Index		m_head;
	Index		m_end;
	unsigned int 	m_size;
};

template <class T>
class DFManagedList : public DFPointerList
{
public:
	DFManagedList() : DFPointerList() {}

	virtual ~DFManagedList()
	{
		Index idx = nullptr;
		idx = next(idx);

		while (idx != nullptr) {
			T *tmp = get(idx);
			delete tmp;
			idx = next(idx);
		}
	}

	T *get(Index idx)
	{
		return static_cast<T *>(DFPointerList::get(idx));
	}

	virtual Index erase(Index idx)
	{
		if (idx != nullptr) {
			T *tmp = get(idx);
			delete tmp;
			return DFPointerList::erase(idx);
		}

		return idx;
	}

	virtual void clear()
	{
		Index idx = nullptr;
		idx = next(idx);

		while (idx != nullptr) {
			T *tmp = get(idx);
			delete tmp;
			idx = next(idx);
		}

		DFPointerList::clear();
	}

	bool pushBack(T *item)
	{
		return DFPointerList::pushBack((void *)item);
	}

	bool pushFront(T *item)
	{
		return DFPointerList::pushFront((void *)item);
	}
};

class DFUIntList : public DisableCopy
{
public:
	class DFUIntListNode;

	typedef DFUIntListNode *Index;

	class DFUIntListNode
	{
	public:
		DFUIntListNode(unsigned int item);
		~DFUIntListNode();

		Index		m_next;
		unsigned int	m_item;
	};

	DFUIntList();

	~DFUIntList();

	unsigned int size();

	bool pushBack(unsigned int item);

	bool pushFront(unsigned int item);

	Index erase(Index idx);

	void clear();

	bool empty();

	Index next(Index &idx);

	bool get(Index idx, unsigned int &val);

	// Caller must lock before using list
	SyncObj m_sync;

private:

	Index		m_head;
	Index		m_end;
	unsigned int 	m_size;
};

};
