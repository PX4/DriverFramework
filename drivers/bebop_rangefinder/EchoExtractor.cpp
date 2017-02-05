#include "EchoExtractor.hpp"
#include <stdlib.h>
#include <string.h>
#include "DriverFramework.hpp"

using namespace DriverFramework;

// Threshold for the derivative to detect rising, falling edges
#define DT_THRESH 100

// Add a small margin after the detected end of send and start searching
// for the echo pulse afterwards
#define EOS_MARGIN 50

EchoExtractor::EchoExtractor(uint16_t signal_length)
	: m_signal_length(signal_length)
{
	// Allocate memory for the signal processing
	m_derivative = static_cast<int16_t *>(malloc(m_signal_length * 2));
	memset(m_derivative, 0, m_signal_length * 2);
	m_filtered_buffer = static_cast<uint16_t *>(malloc(m_signal_length * 2));
	memset(m_filtered_buffer, 0, m_signal_length * 2);
}

EchoExtractor::~EchoExtractor()
{
	free(m_derivative);
	free(m_filtered_buffer);
}

int16_t EchoExtractor::_find_end_of_send(const uint16_t *signal)
{
	bool peak = false;
	bool rising = false;
	uint16_t start = 0;
	uint16_t end = 0;

	for (size_t i = 0; i < m_signal_length; ++i) {
		if (!peak && m_derivative[i] > DT_THRESH) {
			// detect rising edge in the signal
			rising = true;

		} else if (rising && m_derivative[i] < DT_THRESH) {
			// detect peak or plateau
			peak = true;
			rising = false;
			start = i;

		} else if (peak && m_derivative[i] < -DT_THRESH) {
			// detect falling edge of the peak
			end = i;
			break;
		}
	}

	m_start_of_send = start;
	m_end_of_send = end;

	return 0;
}

int16_t EchoExtractor::_differentiate(const uint16_t *signal)
{

	memset(m_derivative, 0, m_signal_length * 2);

	for (size_t i = 3; i < m_signal_length - 3; ++i) {
		m_derivative[i] = - static_cast<int16_t>(signal[i - 3])
				  - static_cast<int16_t>(signal[i - 2])
				  - static_cast<int16_t>(signal[i - 1])
				  + static_cast<int16_t>(signal[i + 1])
				  + static_cast<int16_t>(signal[i + 2])
				  + static_cast<int16_t>(signal[i + 3]);
	}

	return 0;
}

int16_t EchoExtractor::get_echo_index(const uint16_t *signal)
{
	// Preprocess the signal
	_filter_read_buffer(signal);
	_differentiate(m_filtered_buffer);
	_find_end_of_send(m_filtered_buffer);

	// start searching for the echo pulse after this index
	int16_t start_for_signal = m_end_of_send + EOS_MARGIN;

	uint16_t peak_index = 0;
	uint16_t peak_value = 0;
	uint16_t start_current_peak = 0;
	uint16_t max_current_peak = 0;
	bool peak = false;
	bool rising = false;

	for (size_t i = start_for_signal; i < m_signal_length; ++i) {
		if (!peak && m_derivative[i] > DT_THRESH) {
			// rising edge in the signal
			rising = true;

		} else if (rising && m_derivative[i] < DT_THRESH) {
			// detect peak or plateau
			peak = true;
			rising = false;
			start_current_peak = i;
			max_current_peak = signal[i];

		} else if (peak && m_derivative[i] < -DT_THRESH) {
			// detect falling edge after the peak

			// capture new peak if its value was higher than the previous peaks
			if (max_current_peak > peak_value) {
				peak_index = start_current_peak;
				peak_value = max_current_peak;
			}

			max_current_peak = 0;
			peak = false;

		} else if (peak) {
			// find the maximum value within this peak
			if (signal[i] > max_current_peak) {
				max_current_peak = signal[i];
			}
		}
	}

	return peak_index - m_start_of_send;
}

int16_t EchoExtractor::_filter_read_buffer(const uint16_t *signal)
{

	memset(m_filtered_buffer, 0, m_signal_length * 2);

	// Perform a rolling mean filter with size 3

	// Set the values at the left edge
	m_filtered_buffer[0] = signal[0];

	uint16_t running_sum = m_filtered_buffer[0] + m_filtered_buffer[1]
			       + (signal[2]);

	// Mean filter the read signal
	for (unsigned int i = 2; i < m_signal_length - 1; ++i) {
		m_filtered_buffer[i - 1] = running_sum / 3;
		running_sum = (running_sum + (signal[i + 1])
			       - (signal[i - 2]));

	}

	return 0;
}
