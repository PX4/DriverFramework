/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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

#include "I2CDevObj.hpp"

namespace DriverFramework
{

#define BEBOP_BUS_DEVICE_PATH "/dev/i2c-1"
#define BEBOP_BUS_CLASS_PATH "/dev/i2c-1"

// update frequency is 100 Hz
#define BEBOP_BUS_UPDATE_INTERVAL_US 10000
#define BEBOP_BUS_TRANSFER_TIMEOUT_IN_USECS 500

#define BEBOP_BUS_BUS_FREQUENCY_IN_KHZ 400

#define DRV_DF_DEVTYPE_BEBOP_BUS 0x50

#define BEBOP_BUS_SLAVE_ADDRESS 0x08

/// package with published data
struct bebop_state_data {
	float battery_voltage_v;
	uint16_t rpm[4];
} __attribute__((packed));

/// read observations from the Bebop
struct bebop_bus_observation {
	uint16_t rpm_front_left;
	uint16_t rpm_front_right;
	uint16_t rpm_back_right;
	uint16_t rpm_back_left;
	uint16_t battery_voltage_mv;
	uint8_t  status;
	uint8_t  error;
	uint8_t  motors_in_fault;
	uint8_t  temperatur_c;
	uint8_t  checksum;
} __attribute__((packed));

/// read information from the Bebop
struct bebop_bus_info {
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t type;
	uint8_t n_motors_controlled;
	uint16_t n_flights;
	uint16_t last_flight_time;
	uint32_t total_flight_time;
	uint8_t last_error;
} __attribute__((packed));

/// send esc speeds to the Bebop
struct bebop_bus_esc_speeds {
	uint16_t rpm_front_left;
	uint16_t rpm_front_right;
	uint16_t rpm_back_right;
	uint16_t rpm_back_left;
	uint8_t  enable_security;
	uint8_t  checksum;
} __attribute__((packed));

class BebopBus : public DriverFramework::I2CDevObj
{
public:
	BebopBus(const char *device_path)
		: I2CDevObj("BebopBus", device_path, BEBOP_BUS_CLASS_PATH, BEBOP_BUS_UPDATE_INTERVAL_US),
		  _speed_setpoint{}
	{
	}


	virtual int start();

	virtual int stop();

protected:

	/// Enum for the BLDC states, read in observations
	enum BebopBusBLDCStatus : uint8_t {
		INIT = 0x00,
		IDLE = 0x01,
		RAMPING = 0x02,
		SPINNING_1 = 0x03,
		SPINNING_2 = 0x04,
		STOPPING = 0x05,
		CRITICAL = 0x06,
	};

	/// Enum to specify the sound played on the Bebop
	enum BebopBusSound : int8_t {
		NONE = 0,
		SHORT = 1,
		BOOT = 2,
		MELODY = 3,
		REPEAT = -1,
	};

	/// Enum to specify the LED signal
	enum BebopBusGPIO : uint8_t {
		RESET = 0x01,
		RED = 0x02,
		GREEN = 0x04,
	};

	/// Callback to publish the observation data
	virtual int _publish(struct bebop_state_data &data);
	/// Periodically called with update interval
	virtual void _measure();

	/// Get the information (e.g. version, type, flight times) from the Bebop
	int _get_info(struct bebop_bus_info *info);

	/// Get measuremets with the current state (e.g. battery voltage, errors, BLDC states)
	int _get_observation_data(struct bebop_bus_observation *obs);

	/// Send the start motor signal to the Bebop
	int _start_motors();

	/// Send the stop motor signal to the Bebop
	int _stop_motors();

	/// Clear the error cache of the Bebop
	int _clear_errors();

	/// Play the specified sound on the Bebop
	int _play_sound(BebopBusSound sound);

	/// Switch the LEDs to the specified GPIO mode
	int _toggle_gpio(BebopBusGPIO mode);

	/// Set the speeds of the BLDC (speeds are in range 0.0 - 1.0)
	int _set_esc_speed(const float speeds[4]);

	// Get a string for the given status
	const char *strstatus(uint8_t status);

	// Get desired motor speeds in rpm.
	void _get_esc_speed_setpoint(uint16_t speeds_rpm[4]);

private:

	uint16_t _speed_setpoint[4];

	/// Scale the scale 0.0-1.0 to MIN-MAX rpm of the Bebop
	uint16_t _scale_to_rpm(float scale);

	/// The Bebop's checksum
	uint8_t _checksum(uint8_t initial, uint8_t *data, uint16_t packet_size);

	static uint32_t swap32(uint32_t val) { return (val >> 24) | ((val >> 8) & 0x0000FF00) | ((val << 8) & 0x00FF0000) | (val << 24); }
};

}; // namespace DriverFramework
