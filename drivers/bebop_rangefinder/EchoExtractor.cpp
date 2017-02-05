#include "EchoExtractor.hpp"
#include <stdlib.h>
#include <string.h>
#include "DriverFramework.hpp"

// Threshold to detect the beginning or our send pulse
#define BEBOP_RANGEFINDER_REQUEST_THRESH 3276

// The mic records the transmitted signal at the beginning of our received signal.
// This value defines, when it should have ended and is used to detect if we are bellow
// block distance.
#define BEBOP_RANGEFINDER_SEND_PULSE_LEN 420

// This value defines the end of the transitted signal and defines the noise level. A valid
// received signal has to be above this value to be accepted
#define BEBOP_RANGEFINDER_NOISE_LEVEL_THRESH 100

using namespace DriverFramework;

EchoExtractor::EchoExtractor(uint16_t signal_length)
	: m_signal_length(signal_length), m_send_length(0), m_maximum_signal_value(0)
{
	m_filtered_buffer = static_cast<uint16_t *>(malloc(m_signal_length * 2));
	memset(m_filtered_buffer, 0, m_signal_length * 2);
}

EchoExtractor::~EchoExtractor()
{
	free(m_filtered_buffer);
}

int16_t EchoExtractor::_find_end_of_send(const uint16_t *signal)
{
	bool peak = false;
	uint16_t start_index = 0;
	uint16_t end_index = 0;

	for (unsigned int i = 0; i < m_signal_length; ++i) {
		uint16_t value = signal[i];

		if (!peak && value > BEBOP_RANGEFINDER_REQUEST_THRESH) {
			start_index = i;
			peak = true;

		} else if (peak && value < (BEBOP_RANGEFINDER_NOISE_LEVEL_THRESH)) {
			end_index = i;
			break;
		}
	}

	m_send_length = end_index - start_index;

	if (m_send_length > BEBOP_RANGEFINDER_SEND_PULSE_LEN) {
		DF_LOG_DEBUG("Bellow block distance");
		return -1;

	} else {
		return end_index;
	}
}

int16_t EchoExtractor::_filter_read_buffer(const uint16_t *signal)
{

	memset(m_filtered_buffer, 0, m_signal_length);
	m_maximum_signal_value = 0;

	int16_t eos = _find_end_of_send(signal); // get index where the send pulse ends (end-of-send = eos)

	if (eos < 0) {
		return -1;
	}

	// Perform a rolling mean filter with size 3

	// Set the values at the left edge
	m_filtered_buffer[0] = signal[eos];
	m_filtered_buffer[1] = signal[eos + 1];

	uint16_t running_sum = m_filtered_buffer[0] + m_filtered_buffer[1]
			       + (signal[eos + 2]);

	// Mean filter the read signal, but exclude the pulse recorded during send
	for (unsigned int i = 2; i < m_signal_length - eos - 1; ++i) {
		m_filtered_buffer[i] = running_sum / 3;
		running_sum = (running_sum + (signal[eos + i + 1])
			       - (signal[eos + i - 2]));

		// Capture the maximum value in the signal
		if (m_filtered_buffer[i] > m_maximum_signal_value) {
			m_maximum_signal_value = m_filtered_buffer[i];
		}
	}

	if (m_maximum_signal_value < BEBOP_RANGEFINDER_NOISE_LEVEL_THRESH) {
		DF_LOG_DEBUG("No peak found");
		return -1;
	}

	return 0;
}

int16_t EchoExtractor::get_echo_index(const uint16_t *signal)
{
	if (_filter_read_buffer(signal) < 0) {
		return -1;
	}

	// threshold is 4/5 * m_maximum_signal_value
	uint16_t threshold = m_maximum_signal_value - (m_maximum_signal_value / 5);
	bool peak = false;
	bool max_peak_found = false;
	uint16_t start_index = 0;

	// search the filtered signal for the rising edge of the peak, which also
	// includes the maximum value (strongest reflection)
	for (unsigned int i = 0; i < m_signal_length; ++i) {
		if (!peak && m_filtered_buffer[i] > (threshold)) {
			peak = true;
			start_index = i;

		} else if (peak && m_filtered_buffer[i] < threshold) {
			peak = false;

			if (max_peak_found) {
				// return the index of the rising edge and add the length that we cut
				// at the beginning of the signal to exclude the send peak
				return start_index + m_send_length;
			}

		} else if (peak && m_filtered_buffer[i] >= m_maximum_signal_value) {
			max_peak_found = true;
		}
	}

	DF_LOG_DEBUG("No peak");
	return -1;
}

