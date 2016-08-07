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

#include <string.h>

#include "DriverFramework.hpp"
#include "BebopBus.hpp"

#define BEBOP_REG_SET_ESC_SPEED 0x02
#define BEBOP_REG_GET_OBS 0x20
#define BEBOP_REG_START_BLDC 0x40
#define BEBOP_REG_STOP_BLDC 0x60
#define BEBOP_REG_TOGGLE_GPIO 0x4D
#define BEBOP_REG_CLEAR_ERROR 0x80
#define BEBOP_REG_PLAY_SOUND 0x82
#define BEBOP_REG_GET_INFO 0xA0

#define BEBOP_BUS_ERROR_NO 0x00
#define BEBOP_BUS_ERROR_EEPROM 0x01
#define BEBOP_BUS_ERROR_STALLED 0x02
#define BEBOP_BUS_ERROR_PROPELLER_SECURETY 0x03
#define BEBOP_BUS_ERROR_COMMUNICATION_LOST 0x04
#define BEBOP_BUS_ERROR_LOW_BATTERY 0x09
#define BEBOP_BUS_ERROR_LIPO_CELLS 0x0A
#define BEBOP_BUS_ERROR_BLDC 0x0B

// Rotation direction
// front left: clockwise
// front right: counterclockwise
// back right: clockwise
// back left: counterclockwise
#define BEBOP_BLDC_RLRL 0xA 
// Rotation direction
// front left: counterclockwise
// front right: clockwise
// back right: counterclockwise
// back left: clockwise
#define BEBOP_BLDC_LRLR 0x5 // 

using namespace DriverFramework;

int BebopBus::start()
{
	int result = I2CDevObj::start();

	if (result < 0) {
		DF_LOG_ERR("DevObj start failed");
		DF_LOG_ERR("Unable to open the device path: %s", m_dev_path);
		return result;
	}

	result = _setSlaveConfig(BEBOP_BUS_SLAVE_ADDRESS,
				 BEBOP_BUS_BUS_FREQUENCY_IN_KHZ,
				 BEBOP_BUS_TRANSFER_TIMEOUT_IN_USECS);

	if (result < 0) {
		DF_LOG_ERR("Could not set slave config");
	}

	/* Try to talk to the sensor. */
	struct bebop_bus_info info;
	result = _get_info(&info);

	DF_LOG_INFO("Software Version: %d.%d", info.version_major, info.version_minor);
	DF_LOG_INFO("Software Type: %d", info.type);
	DF_LOG_INFO("Number of controlled motors: %d", info.n_motors_controlled);
	DF_LOG_INFO("Number of flights: %d", swap16(info.n_flights));
	DF_LOG_INFO("Last flight time: %d", swap16(info.last_flight_time));
	DF_LOG_INFO("Total flight time: %d", swap32(info.total_flight_time));
	DF_LOG_INFO("Last Error: %d\n", info.last_error);

	if (result < 0) {
		DF_LOG_ERR("Unable to communicate with the sensor");
		return -1;
	}

	result = _clear_errors();
	if (result < 0)
	{
		DF_LOG_ERR("Unable to clear errors");
		return -1;
	}

  //result = _bebop_bus_init();

	if (result != 0) {
		DF_LOG_ERR("error: Bebop bus initialization failed, thread not started");
		goto exit;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("DevObj start failed");
		return result;
	}

exit:
	return result;
}
	
uint8_t BebopBus::_checksum(uint8_t *data, uint16_t packet_size)
{
  uint8_t checksum = data[0];
  for(size_t i = 0; i < packet_size; i++){
       checksum = checksum ^ data[i];
  }
  return checksum;
}

int BebopBus::_get_info(struct bebop_bus_info *info)
{
	memset(info, 0, sizeof(bebop_bus_info));

	return _readReg(BEBOP_REG_GET_INFO, (uint8_t *)info, sizeof(bebop_bus_info));
}

int BebopBus::_get_observation_data(struct bebop_bus_observation *obs)
{
	memset(obs, 0, sizeof(bebop_bus_observation));

	return _readReg(BEBOP_REG_GET_OBS, (uint8_t *)obs, sizeof(bebop_bus_observation));
}

int BebopBus::_start_motors()
{

	uint8_t bits = BEBOP_BLDC_LRLR;;

	if (_writeReg(BEBOP_REG_START_BLDC, &bits, 1) != 0) {
		DF_LOG_ERR("Unable to start BLDCs");
		return -1;
	}
  return 0;
}
	
int BebopBus::_stop_motors()
{
	if (_writeReg(BEBOP_REG_STOP_BLDC, 0, 1) != 0) {
		DF_LOG_ERR("Unable to stop BLDCs");
		return -1;
	}
  return 0;
}
	
int BebopBus::_clear_errors()
{

	uint8_t bits = 0x01;

	if (_writeReg(BEBOP_REG_CLEAR_ERROR, &bits, 1) != 0) {
		DF_LOG_ERR("Unable to clear errors");
		return -1;
	}
  return 0;
}

int BebopBus::_play_sound(BebopBusSound sound)
{
	uint8_t bits = sound;

	if (_writeReg(BEBOP_REG_PLAY_SOUND, &bits, 1) != 0) {
		DF_LOG_ERR("Unable to play sound");
		return -1;
	}
  return 0;
}

int BebopBus::_toggle_gpio(BebopBusGPIO mode)
{
	uint8_t bits = mode;

	if (_writeReg(BEBOP_REG_TOGGLE_GPIO, &bits, 1) != 0) {
		DF_LOG_ERR("Unable to toggle gpio");
		return -1;
	}
  return 0;
}

int BebopBus::_set_esc_speed(struct bebop_bus_esc_speeds *speeds)
{
	// TODO Add implementation
	return -1;
}

int BebopBus::stop()
{
	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("DevObj stop failed");
		return result;
	}

	return 0;
}

void BebopBus::_update()
{
	// TODO Add implementation
}

void BebopBus::_measure()
{
	// TODO Add implementation
}

void BebopBus::_publish(struct bebop_state_data &data)
{
	// TODO Add implementation
}

int BebopBus::_bebop_bus_init()
{
	// TODO Add implementation
	return -1;
}
