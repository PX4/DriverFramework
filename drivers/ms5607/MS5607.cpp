/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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

#include "MS5607.hpp"

using namespace DriverFramework;

#define POW2(_x) ((_x) * (_x))

int32_t MS5607::convertTemperature(int32_t adc_T)
{
	// Conversion from the datasheet
	/* temperature offset (in ADC units) */
	int32_t dT = adc_T - ((int32_t)m_sensor_calibration.c5_reference_temp << 8);

	int64_t sens = ((int64_t)m_sensor_calibration.c1_pressure_sens << 16)
		       + (((int64_t)m_sensor_calibration.c3_temp_coeff_pres_sens * dT) >> 7);

	int64_t off = ((int64_t)m_sensor_calibration.c2_pressure_offset << 17)
		      + (((int64_t)m_sensor_calibration.c4_temp_coeff_pres_offset * dT) >> 6);

	/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
	int32_t temp =  2000 + (int32_t)(((int64_t)dT * m_sensor_calibration.c6_temp_coeff_temp) >> 23);

	/* temperature compensation */
	if (temp < 2000) {
		int32_t t2 = POW2(dT) >> 31;

		int64_t f = POW2((int64_t)temp - 2000);
		int64_t off2 = 61 * f >> 4;
		int64_t sens2 = 2 * f;

		if (temp < -1500) {
			int64_t f2 = POW2(temp + 1500);
			off2 += 15 * f2;
			sens2 += 8 * f2;
		}

		temp -= t2;
		sens -= sens2;
		off -= off2;
	}

	m_raw_sensor_convertion.temperature_cc = temp;
	m_raw_sensor_convertion.sens = sens;
	m_raw_sensor_convertion.off = off;
	return temp;
}

int MS5607::_publish(struct baro_sensor_data &data)
{
	// TBD
	return -1;
}
