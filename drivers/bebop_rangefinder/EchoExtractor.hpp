/****************************************************************************
 *
 *   Copyright (C) 2017 PX4 Development Team. All rights reserved.
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

#pragma once

#include <stdint.h>

namespace DriverFramework
{

class EchoExtractor
{
public:
	EchoExtractor(uint16_t signal_length) ;
	~EchoExtractor();

	// Returns the index where the echo pulse was detected in the signal
	// This is the differnce between the beginning of send and the beginning
	// of the reflected echo
	int16_t get_echo_index(const uint16_t *signal);

private:
	// Returns the index where the send pulse ends
	int16_t _find_end_of_send(const uint16_t *signal);
	// Comoute the derivative of the signal
	int16_t _differentiate(const uint16_t *signal);
	// Simple mean filter
	int16_t _filter_read_buffer(const uint16_t *signal);

	uint16_t m_start_of_send; // index where send pulse starts
	uint16_t m_end_of_send;   // index where send pulse ends

	uint16_t m_signal_length; // total buffer length
	int16_t *m_derivative;    // memory for the signal's derivative
	uint16_t *m_filtered_buffer; // memory for the filtered signal
};
} // namespace DriverFramework
