/****************************************************************************
 *
 *   Copyright (C) 2016 Bharath Ramaswamy. All rights reserved.
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

/***************************************************
 * modified by Christoph Tobler <christoph@px4.io>
 ***************************************************/

#pragma once

#include "LtcSensor.hpp"

namespace DriverFramework
{

// From the schematic
#define LTC2946_DEVICE_PATH "/dev/iic-3"

// TODO: What should the correct value be?
#define DRV_DF_DEVTYPE_LTC2946 0x49

#define LTC_MODE_ESC_VBATT 0
#define LTC_MODE_APM_VBATT 1
#define LTC_MODE_APM_5V    2

#define R_SENSE_ESC_VBATT	0.001
#define R_SENSE_APM_VBATT	0.0005
#define R_SENSE_APM_5V		0.005

// I2C address to measure battery supply voltage and current
#define LTC2946_I2C_ADDRESS_VBATT 0x6a //0b1101010  // 0x6a  0xD4  ADR1-NC ADR0-NC , page 25 of manual
// I2C address to measure 5V supply voltage and current
#define LTC2946_I2C_ADDRESS_5V 0x6b //0b1101011     // 0x6b  0xD6  ADR1-NC ADR0-L

// *** Set this to a LtcModeType value ***
#define LTC_MODE LTC_MODE_APM_5V
//#define LTC_MODE LTC_MODE_ESC_VBATT
//#define LTC_MODE LTC_MODE_APM_VBATT

#define LTC2946_BUS_FREQUENCY_IN_KHZ 400
#define LTC2946_TRANSFER_TIMEOUT_IN_USECS 9000

#define LTC2946_I2C_ADDRESS LTC2946_I2C_ADDRESS_5V

class LTC2946 : public LtcSensor
{
public:
	LTC2946(const char *device_path) :
		LtcSensor(device_path, LTC2946_MEASURE_INTERVAL_US)
	{
		m_id.dev_id_s.devtype = DRV_DF_DEVTYPE_LTC2946;
		m_id.dev_id_s.address = LTC2946_I2C_ADDRESS;
	}

	// @return 0 on success, -errno on failure
	virtual int start();

	// @return 0 on success, -errno on failure
	virtual int stop();

	static void printValues(struct ltc2946_sensor_data &data);
	static int getSensorData(DevHandle &h, struct ltc2946_sensor_data &out_data,
				 bool is_new_data_required);

protected:
	virtual void _measure();
	virtual int _publish(const struct ltc2946_sensor_data &data);

	struct ltc2946_sensor_data m_sensor_data;
	SyncObj m_synchronize;

private:
	// @returns 0 on success, -errno on failure
	int ltc2946_init();
	int configure();
};

#define LTC2946_I2C_MASS_WRITE      0xCC
#define LTC2946_I2C_ALERT_RESPONSE  0x19

// Registers
#define LTC2946_CTRLA_REG                           0x00
#define LTC2946_CTRLB_REG                           0x01
#define LTC2946_ALERT1_REG                          0x02
#define LTC2946_STATUS1_REG                         0x03
#define LTC2946_FAULT1_REG                          0x04

#define LTC2946_POWER_MSB2_REG                      0x05
#define LTC2946_POWER_MSB1_REG                      0x06
#define LTC2946_POWER_LSB_REG                       0x07
#define LTC2946_MAX_POWER_MSB2_REG                  0x08
#define LTC2946_MAX_POWER_MSB1_REG                  0x09
#define LTC2946_MAX_POWER_LSB_REG                   0x0A
#define LTC2946_MIN_POWER_MSB2_REG                  0x0B
#define LTC2946_MIN_POWER_MSB1_REG                  0x0C
#define LTC2946_MIN_POWER_LSB_REG                   0x0D
#define LTC2946_MAX_POWER_THRESHOLD_MSB2_REG        0x0E
#define LTC2946_MAX_POWER_THRESHOLD_MSB1_REG        0x0F
#define LTC2946_MAX_POWER_THRESHOLD_LSB_REG         0x10
#define LTC2946_MIN_POWER_THRESHOLD_MSB2_REG        0x11
#define LTC2946_MIN_POWER_THRESHOLD_MSB1_REG        0x12
#define LTC2946_MIN_POWER_THRESHOLD_LSB_REG         0x13

#define LTC2946_DELTA_SENSE_MSB_REG                 0x14
#define LTC2946_DELTA_SENSE_LSB_REG                 0x15
#define LTC2946_MAX_DELTA_SENSE_MSB_REG             0x16
#define LTC2946_MAX_DELTA_SENSE_LSB_REG             0x17
#define LTC2946_MIN_DELTA_SENSE_MSB_REG             0x18
#define LTC2946_MIN_DELTA_SENSE_LSB_REG             0x19
#define LTC2946_MAX_DELTA_SENSE_THRESHOLD_MSB_REG   0x1A
#define LTC2946_MAX_DELTA_SENSE_THRESHOLD_LSB_REG   0x1B
#define LTC2946_MIN_DELTA_SENSE_THRESHOLD_MSB_REG   0x1C
#define LTC2946_MIN_DELTA_SENSE_THRESHOLD_LSB_REG   0x1D

#define LTC2946_VIN_MSB_REG                         0x1E
#define LTC2946_VIN_LSB_REG                         0x1F
#define LTC2946_MAX_VIN_MSB_REG                     0x20
#define LTC2946_MAX_VIN_LSB_REG                     0x21
#define LTC2946_MIN_VIN_MSB_REG                     0x22
#define LTC2946_MIN_VIN_LSB_REG                     0x23
#define LTC2946_MAX_VIN_THRESHOLD_MSB_REG           0x24
#define LTC2946_MAX_VIN_THRESHOLD_LSB_REG           0x25
#define LTC2946_MIN_VIN_THRESHOLD_MSB_REG           0x26
#define LTC2946_MIN_VIN_THRESHOLD_LSB_REG           0x27

#define LTC2946_ADIN_MSB_REG                        0x28
#define LTC2946_ADIN_LSB_REG_REG                    0x29
#define LTC2946_MAX_ADIN_MSB_REG                    0x2A
#define LTC2946_MAX_ADIN_LSB_REG                    0x2B
#define LTC2946_MIN_ADIN_MSB_REG                    0x2C
#define LTC2946_MIN_ADIN_LSB_REG                    0x2D
#define LTC2946_MAX_ADIN_THRESHOLD_MSB_REG          0x2E
#define LTC2946_MAX_ADIN_THRESHOLD_LSB_REG          0x2F
#define LTC2946_MIN_ADIN_THRESHOLD_MSB_REG          0x30
#define LTC2946_MIN_ADIN_THRESHOLD_LSB_REG          0x31

#define LTC2946_ALERT2_REG                          0x32
#define LTC2946_GPIO_CFG_REG                        0x33

#define LTC2946_TIME_COUNTER_MSB3_REG               0x34
#define LTC2946_TIME_COUNTER_MSB2_REG               0x35
#define LTC2946_TIME_COUNTER_MSB1_REG               0x36
#define LTC2946_TIME_COUNTER_LSB_REG                0x37

#define LTC2946_CHARGE_MSB3_REG                     0x38
#define LTC2946_CHARGE_MSB2_REG                     0x39
#define LTC2946_CHARGE_MSB1_REG                     0x3A
#define LTC2946_CHARGE_LSB_REG                      0x3B

#define LTC2946_ENERGY_MSB3_REG                     0x3C
#define LTC2946_ENERGY_MSB2_REG                     0x3D
#define LTC2946_ENERGY_MSB1_REG                     0x3E
#define LTC2946_ENERGY_LSB_REG                      0x3F

#define LTC2946_STATUS2_REG                         0x40
#define LTC2946_FAULT2_REG                          0x41
#define LTC2946_GPIO3_CTRL_REG                      0x42
#define LTC2946_CLK_DIV_REG                         0x43
// Voltage Selection Command
#define LTC2946_DELTA_SENSE            				0x00
#define LTC2946_VDD                    				0x08
#define LTC2946_ADIN                   				0x10
#define LTC2946_SENSE_PLUS             				0x18
// Command Codes
#define LTC2946_ADIN_INTVCC                     0x80
#define LTC2946_ADIN_GND                        0x00

#define LTC2946_OFFSET_CAL_LAST                 0x60
#define LTC2946_OFFSET_CAL_128                  0x40
#define LTC2946_OFFSET_CAL_16                   0x20
#define LTC2946_OFFSET_CAL_EVERY                0x00

#define LTC2946_CHANNEL_CONFIG_SNAPSHOT         0x07
#define LTC2946_CHANNEL_CONFIG_V_C              0x06
#define LTC2946_CHANNEL_CONFIG_A_V_C_1          0x05
#define LTC2946_CHANNEL_CONFIG_A_V_C_2          0x04
#define LTC2946_CHANNEL_CONFIG_A_V_C_3          0x03
#define LTC2946_CHANNEL_CONFIG_V_C_1            0x02
#define LTC2946_CHANNEL_CONFIG_V_C_2            0x01
#define LTC2946_CHANNEL_CONFIG_V_C_3            0x00


#define LTC2946_ENABLE_ALERT_CLEAR              0x80
#define LTC2946_ENABLE_SHUTDOWN                 0x40
#define LTC2946_ENABLE_CLEARED_ON_READ          0x20
#define LTC2946_ENABLE_STUCK_BUS_RECOVER        0x10

#define LTC2946_DISABLE_ALERT_CLEAR             0x7F
#define LTC2946_DISABLE_SHUTDOWN                0xBF
#define LTC2946_DISABLE_CLEARED_ON_READ         0xDF
#define LTC2946_DISABLE_STUCK_BUS_RECOVER       0xEF

#define LTC2946_ACC_PIN_CONTROL                 0x08
#define LTC2946_DISABLE_ACC                     0x04
#define LTC2946_ENABLE_ACC                      0x00

#define LTC2946_RESET_ALL                       0x03
#define LTC2946_RESET_ACC                       0x02
#define LTC2946_ENABLE_AUTO_RESET               0x01
#define LTC2946_DISABLE_AUTO_RESET              0x00


#define LTC2946_MAX_POWER_MSB2_RESET            0x00
#define LTC2946_MIN_POWER_MSB2_RESET            0xFF
#define LTC2946_MAX_DELTA_SENSE_MSB_RESET       0x00
#define LTC2946_MIN_DELTA_SENSE_MSB_RESET       0xFF
#define LTC2946_MAX_VIN_MSB_RESET               0x00
#define LTC2946_MIN_VIN_MSB_RESET               0xFF
#define LTC2946_MAX_ADIN_MSB_RESET              0x00
#define LTC2946_MIN_ADIN_MSB_RESET              0xFF

#define LTC2946_ENABLE_MAX_POWER_ALERT          0x80
#define LTC2946_ENABLE_MIN_POWER_ALERT          0x40
#define LTC2946_DISABLE_MAX_POWER_ALERT         0x7F
#define LTC2946_DISABLE_MIN_POWER_ALERT         0xBF

#define LTC2946_ENABLE_MAX_I_SENSE_ALERT        0x20
#define LTC2946_ENABLE_MIN_I_SENSE_ALERT        0x10
#define LTC2946_DISABLE_MAX_I_SENSE_ALERT       0xDF
#define LTC2946_DISABLE_MIN_I_SENSE_ALERT       0xEF

#define LTC2946_ENABLE_MAX_VIN_ALERT            0x08
#define LTC2946_ENABLE_MIN_VIN_ALERT            0x04
#define LTC2946_DISABLE_MAX_VIN_ALERT           0xF7
#define LTC2946_DISABLE_MIN_VIN_ALERT           0xFB

#define LTC2946_ENABLE_MAX_ADIN_ALERT           0x02
#define LTC2946_ENABLE_MIN_ADIN_ALERT           0x01
#define LTC2946_DISABLE_MAX_ADIN_ALERT          0xFD
#define LTC2946_DISABLE_MIN_ADIN_ALERT          0xFE

#define LTC2946_ENABLE_ADC_DONE_ALERT           0x80
#define LTC2946_DISABLE_ADC_DONE_ALERT          0x7F

#define LTC2946_ENABLE_GPIO_1_ALERT             0x40
#define LTC2946_DISABLE_GPIO_1_ALERT            0xBF

#define LTC2946_ENABLE_GPIO_2_ALERT             0x20
#define LTC2946_DISABLE_GPIO_2_ALERT            0xDF

#define LTC2946_ENABLE_STUCK_BUS_WAKE_ALERT     0x08
#define LTC2946_DISABLE_STUCK_BUS_WAKE_ALERT    0xF7

#define LTC2946_ENABLE_ENERGY_OVERFLOW_ALERT    0x04
#define LTC2946_DISABLE_ENERGY_OVERFLOW_ALERT   0xFB

#define LTC2946_ENABLE_CHARGE_OVERFLOW_ALERT    0x02
#define LTC2946_DISABLE_CHARGE_OVERFLOW_ALERT   0xFD

#define LTC2946_ENABLE_COUNTER_OVERFLOW_ALERT   0x01
#define LTC2946_DISABLE_COUNTER_OVERFLOW_ALERT  0xFE

#define LTC2946_GPIO1_IN_ACTIVE_HIGH            0xC0
#define LTC2946_GPIO1_IN_ACTIVE_LOW             0x80
#define LTC2946_GPIO1_OUT_HIGH_Z                0x40
#define LTC2946_GPIO1_OUT_LOW                   0x00

#define LTC2946_GPIO2_IN_ACTIVE_HIGH            0x30
#define LTC2946_GPIO2_IN_ACTIVE_LOW             0x20
#define LTC2946_GPIO2_OUT_HIGH_Z                0x10
#define LTC2946_GPIO2_OUT_LOW                   0x12
#define LTC2946_GPIO2_IN_ACC                    0x00


#define LTC2946_GPIO3_IN_ACTIVE_HIGH            0x0C
#define LTC2946_GPIO3_IN_ACTIVE_LOW             0x08
#define LTC2946_GPIO3_OUT_REG_42                0x04
#define LTC2946_GPIO3_OUT_ALERT                 0x00
#define LTC2946_GPIO3_OUT_LOW                   0x40
#define LTC2946_GPIO3_OUT_HIGH_Z                0x00
#define LTC2946_GPIO_ALERT_CLEAR                0x00
// Register Mask Command
#define LTC2946_CTRLA_ADIN_MASK                0x7F
#define LTC2946_CTRLA_OFFSET_MASK              0x9F
#define LTC2946_CTRLA_VOLTAGE_SEL_MASK         0xE7
#define LTC2946_CTRLA_CHANNEL_CONFIG_MASK      0xF8
#define LTC2946_CTRLB_ACC_MASK                 0xF3
#define LTC2946_CTRLB_RESET_MASK               0xFC
#define LTC2946_GPIOCFG_GPIO1_MASK             0x3F
#define LTC2946_GPIOCFG_GPIO2_MASK             0xCF
#define LTC2946_GPIOCFG_GPIO3_MASK             0xF3
#define LTC2946_GPIOCFG_GPIO2_OUT_MASK         0xFD
#define LTC2946_GPIO3_CTRL_GPIO3_MASK          0xBF
}; // namespace DriverFramework
