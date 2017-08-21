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
#include "DFList.hpp"

using namespace DriverFramework;

DFPointerList::~DFPointerList()
{
	m_sync.lock();
	Index next_del = m_head;

	while (next_del != nullptr) {
		Index curr = next_del;
		next_del = curr->m_next;
		delete curr;
	}

	m_head = nullptr;
	m_sync.unlock();
}

unsigned int DFPointerList::size()
{
	m_sync.lock();
	unsigned int rv = m_size;
	m_sync.unlock();
	return rv;
}

bool DFPointerList::pushBack(void *item)
{
	m_sync.lock();
	Index t = new DFListNode(item);

	if (t == nullptr) {
		m_sync.unlock();
		return false;
	}

	if (m_head == nullptr) {
		m_head = t;
		m_end = t;

	} else {
		m_end->m_next = t;
		m_end = t;
	}

	++m_size;
	m_sync.unlock();
	return true;
}

bool DFPointerList::pushFront(void *item)
{
	m_sync.lock();
	Index t = new DFListNode(item);

	if (t == nullptr) {
		m_sync.unlock();
		return false;
	}

	if (m_head == nullptr) {
		m_head = t;
		m_end = t;

	} else {
		t->m_next = m_head;
		m_head = t;
	}

	++m_size;
	m_sync.unlock();
	return true;
}

DFPointerList::Index DFPointerList::erase(Index idx)
{
	m_sync.lock();

	if (idx != nullptr && idx == m_head) {
		Index next_erase = m_head->m_next;
		delete m_head;
		m_head = next_erase;

		if (next_erase == nullptr) {
			m_end = nullptr;
		}

		--m_size;
		m_sync.unlock();
		return m_head;

	} else {
		for (Index curr = m_head; curr->m_next != nullptr;) {
			Index next_erase = curr->m_next;

			if (idx == next_erase) {
				curr->m_next = next_erase->m_next;

				if (next_erase == m_end) {
					m_end = curr;
				}

				delete next_erase;
				--m_size;
				m_sync.unlock();
				return curr->m_next;
			}

			curr = next_erase;
		}
	}

	m_sync.unlock();
	return nullptr;
}

void DFPointerList::clear()
{
	m_sync.lock();

	while (m_head != nullptr) {
		Index tmp = m_head;
		m_head = m_head->m_next;
		delete tmp;
	}

	m_sync.unlock();
}

bool DFPointerList::empty()
{
	m_sync.lock();
	bool is_empty = (m_head == nullptr);
	m_sync.unlock();
	return is_empty;
}

DFPointerList::Index DFPointerList::next(Index &idx)
{
	m_sync.lock();

	if (idx == nullptr) {
		idx = m_head;

	} else {
		idx = idx->m_next;
	}

	m_sync.unlock();
	return idx;

}

void *DFPointerList::get(Index idx)
{
	if (idx) {
		return idx->m_item;
	}

	return nullptr;
}

DFUIntList::~DFUIntList()
{
	m_sync.lock();
	Index next_del = m_head;

	while (next_del != nullptr) {
		Index curr = next_del;
		next_del = curr->m_next;
		delete curr;
	}

	m_head = nullptr;
	m_sync.unlock();
}

unsigned int DFUIntList::size()
{
	return m_size;
}

bool DFUIntList::pushBack(unsigned int item)
{
	m_sync.lock();
	Index t = new DFUIntListNode(item);

	if (t == nullptr) {
		m_sync.unlock();
		return false;
	}

	if (m_head == nullptr) {
		m_head = t;
		m_end = t;

	} else {
		m_end->m_next = t;
		m_end = t;
	}

	++m_size;
	m_sync.unlock();
	return true;
}

bool DFUIntList::pushFront(unsigned int item)
{
	m_sync.lock();
	Index t = new DFUIntListNode(item);

	if (t == nullptr) {
		m_sync.unlock();
		return false;
	}

	if (m_head == nullptr) {
		m_head = t;
		m_end = t;

	} else {
		t->m_next = m_head;
		m_head = t;
	}

	++m_size;
	m_sync.unlock();
	return true;
}

DFUIntList::Index DFUIntList::erase(Index idx)
{
	m_sync.lock();

	if (idx != nullptr && idx == m_head) {
		Index t = m_head->m_next;
		delete (m_head);
		m_head = t;

		if (t == nullptr) {
			m_end = nullptr;
		}

		--m_size;
		m_sync.unlock();
		return m_head;

	} else {
		for (Index p = m_head; p->m_next != nullptr;) {
			Index t = p->m_next;

			if (idx == t) {
				p->m_next = t->m_next;

				if (t == m_end) {
					m_end = p;
				}

				delete (t);
				--m_size;
				m_sync.unlock();
				return p->m_next;
			}

			p = t;
		}
	}

	m_sync.unlock();
	return nullptr;
}

void DFUIntList::clear()
{
	while (m_head != nullptr) {
		erase(m_head);
	}
}

bool DFUIntList::empty()
{
	m_sync.lock();
	bool is_empty = (m_head == nullptr);
	m_sync.unlock();
	return is_empty;
}

DFUIntList::Index DFUIntList::next(Index &idx)
{
	m_sync.lock();

	if (idx == nullptr) {
		idx = m_head;

	} else {
		idx = idx->m_next;
	}

	m_sync.unlock();
	return idx;
}

bool DFUIntList::get(Index idx, unsigned int &val)
{
	if (idx) {
		val = idx->m_item;
		return true;
	}

	return false;
}

