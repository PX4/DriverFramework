/****************************************************************************
 *
 *   Copyright (C) 2017 Nicolae Rosia. All rights reserved.
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

#include "ImuSensor.hpp"

namespace DriverFramework
{

void printImuValues(struct imu_sensor_data &data)
{
	DF_LOG_INFO("IMU: accel: [%.2f, %.2f, %.2f] m/s^2",
		    (double)data.accel_m_s2_x,
		    (double)data.accel_m_s2_y,
		    (double)data.accel_m_s2_z);
	DF_LOG_INFO("     gyro:  [%.2f, %.2f, %.2f] rad/s",
		    (double)data.gyro_rad_s_x,
		    (double)data.gyro_rad_s_y,
		    (double)data.gyro_rad_s_z);

	DF_LOG_INFO("     mag:  [%.6f, %.6f, %.6f] ga",
		    (double)data.mag_ga_x,
		    (double)data.mag_ga_y,
		    (double)data.mag_ga_z);

	DF_LOG_INFO("     temp:  %.2f C",
		    (double)data.temp_c);
}

}
