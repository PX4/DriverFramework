/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include <unistd.h>
#include "DriverFramework.hpp"

using namespace DriverFramework;

extern "C" {

	void block_indefinite(void)
	{
		for (;;) {
			volatile int x = 0;
			++x;
		}
	}

	unsigned int sleep(unsigned int seconds)
	{
		return usleep(seconds * 1000000);
	}

	void _Read_uleb(void)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void _Parse_fde_instr(void)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void _Parse_csd(void)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void _Locksyslock(int x)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void _Unlocksyslock(int x)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void _Valbytes(void)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void _Get_eh_data(void)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void _Parse_lsda(void)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void __cxa_guard_release(void)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void _Read_enc_ptr(void)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void _Read_sleb(void)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void __cxa_guard_acquire(void)
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

	void __cxa_pure_virtual()
	{
		DF_LOG_ERR("Error: Calling unresolved symbol stub[%s]", __FUNCTION__);
		block_indefinite();
	}

};
