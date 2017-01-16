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

#include "MT9V117.hpp"
#include "MT9V117_patches.hpp"

#if !(defined(__APPLE__) && defined(__MACH__))
// v4l2-subdev.h is not available on OSX
#include <linux/v4l2-subdev.h>
#endif

#include <string.h>

#define MT9V117_BUS_FREQUENCY_IN_KHZ 400
#define MT9V117_TRANSFER_TIMEOUT_IN_USEC 500

#define MT9V117_REG_ID 0x00
#define MT9V117_REG_RESET 0x001A
#define MT9V117_REG_ACCESS_CTL 0x0982
#define MT9V117_PHYSICAL_ADDRESS_ACCESS 0x098A
#define MT9V117_LOGICAL_ADDRESS_ACCESS 0x098E
#define MT9V117_REG_PAD_SLEW 0x0030
#define MT9V117_REG_CMD 0x0040
#define MT9V117_REG_AE_TRACK_JUMP_DIVISOR 0xa812
#define MT9V117_REG_CAM_AET_SKIP_FRAMES 0xc868

#define MT9V117_WHOAMI 0x2282
#define MT9V117_RESET 0x0001
#define HOST_CMD_OK 0x8000
#define HOST_CMD_0 0x0001
#define HOST_CMD_1 0x0002
#define HOST_CMD_2 0x0004
#define MT9V117_AE_RULE_AVERAGE 0x00
#define MT9V117_AE_RULE_WEIGHTED 0x01

#define MT9V117_VAR_AE_RULE 0x9
#define MT9V117_VAR_AE_TRACK 0xA
#define MT9V117_VAR_AWB 0xB
#define MT9V117_VAR_CAM_CTRL 0x12
#define MT9V117_VAR_LOW_LIGHT 0xF
#define MT9V117_VAR_SYSMGR 0x17
#define MT9V117_VAR_PATCHLDR 0x18

#define MT9V117_OUTPUT_FORMAT_RGB_565       0x0
#define MT9V117_OUTPUT_FORMAT_RGB_555       0x1000
#define MT9V117_OUTPUT_FORMAT_RGB_444X      0x2000
#define MT9V117_OUTPUT_FORMAT_RGB_X444      0x3000
#define MT9V117_OUTPUT_FORMAT_BAYER_10      0x0
#define MT9V117_OUTPUT_FORMAT_YUV           0x0
#define MT9V117_OUTPUT_FORMAT_RGB           0x100
#define MT9V117_OUTPUT_FORMAT_BAYER         0x200
#define MT9V117_OUTPUT_FORMAT_BT656_ENABLE  0x08
#define MT9V117_OUTPUT_FORMAT_MONO_ENABLE   0x04
#define MT9V117_OUTPUT_FORMAT_SWAP_BYTES    0x02
#define MT9V117_OUTPUT_FORMAT_SWAP_RED_BLUE 0x01

using namespace DriverFramework;

int MT9V117::start()
{
	int result = init();

	if (result != 0) {
		DF_LOG_ERR("Init error");
		return -1;
	}

	// Start PWM signal which is the clock of the image sensor
	// Period = 23ns -> Freq = 43333333Hz
	_sensor_clock.set_frequency(43333333UL);
	_sensor_clock.enable();

	// Reset the image sensor via GPIO
	_sensor_reset.set_value(false);
	_sensor_reset.set_value(true);

	usleep(15000); // 15ms

	result = I2CDevObj::start();

	if (result != 0) {
		DF_LOG_ERR("Error: Could not start DevObj");
		return result;
	}

	result = _setSlaveConfig(MT9V117_SLAVE_ADDRESS,
				 MT9V117_BUS_FREQUENCY_IN_KHZ,
				 MT9V117_TRANSFER_TIMEOUT_IN_USEC);

	if (result != 0) {
		DF_LOG_ERR("I2C slave configuration failed.");
		return result;
	}

	result = mt9v117_init();

	if (result != 0) {
		DF_LOG_ERR("Error: Image sensor initialization failed");
		return result;
	}

	result = DevObj::start();

	if (result != 0) {
		DF_LOG_ERR("Error: Could not start DevObj: %d", result);
		return result;
	}

	set_format();

	return 0;
}

int MT9V117::stop()
{
	_sensor_clock.disable();
	_sensor_reset.set_value(false);

	int result = DevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("Error: Could not stop DevObj");
		return result;
	}

	result = I2CDevObj::stop();

	if (result != 0) {
		DF_LOG_ERR("Error: Could not stop I2CDevObj");
		return result;
	}

	return 0;
}

void MT9V117::_measure()
{
	// NoOp
}

void MT9V117::_publish()
{
	// NoOp
}

int MT9V117::mt9v117_init()
{
	if (probe() != 0) {
		DF_LOG_ERR("MT9V117 image sensor not detected");
		return -1;
	}

	if (soft_reset() != 0) {
		DF_LOG_ERR("MT9V117 image sensor soft reset failed");
		return -1;
	}

	if (write_patch() != 0) {
		DF_LOG_ERR("MT9V117 image sensor write patch failed");
		return -1;
	}

	if (write_settings() != 0) {
		DF_LOG_ERR("MT9V117 image sensor write settings failed");
		return -1;
	}

	if (configure_sensor() != 0) {
		DF_LOG_ERR("MT9V117 image sensor configuration failed");
		return -1;
	}

	return 0;
}

int MT9V117::probe()
{
	uint16_t id = 0;

	int result = read16(MT9V117_REG_ID, &id);

	if (result != 0) {
		DF_LOG_ERR("Unable to read whoami reg: %d", result);
		return -1;
	}

	if (id != MT9V117_WHOAMI) {
		DF_LOG_ERR("Wrong WHOAMI %x (expected %x)", id, MT9V117_WHOAMI);
		return -1;
	}

	return 0;
}

int MT9V117::soft_reset()
{
	uint16_t tx = MT9V117_RESET;
	int result = write16(MT9V117_REG_RESET, tx);

	if (result != 0) {
		DF_LOG_ERR("Unable to soft reset");
		return -1;
	}

	result = write16(MT9V117_REG_RESET, 0);

	if (result != 0) {
		DF_LOG_ERR("Unable to restart after soft reset");
		return -1;
	}

	usleep(50000); // 50ms
	return 0;
}

int inline MT9V117::write8(uint16_t add, uint8_t val)
{
	uint8_t tx = val;
	return _writeReg16(swap16(add), (uint16_t *) &tx, 1);
}

int inline MT9V117::write16(uint16_t add, uint16_t val)
{
	uint16_t buf = swap16(val);
	return _writeReg16(swap16(add), &buf, 2);
}

int inline MT9V117::write32(uint16_t add, uint32_t val)
{
	uint32_t buf = swap32(val);
	return _writeReg16(swap16(add), (uint16_t *)&buf, 4);
}

int MT9V117::read16(uint16_t add, uint16_t *val)
{
	uint16_t read_val;
	int result = _readReg16(swap16(add), &read_val, 2);

	*val = swap16(read_val);

	return result;
}

int MT9V117::write_patch()
{
	int result = 0;

	// Errata item 2
	result += write16(0x301a, 0x10d0);
	result += write16(0x31c0, 0x1404);
	result += write16(0x3ed8, 0x879c);
	result += write16(0x3042, 0x20e1);
	result += write16(0x30d4, 0x8020);
	result += write16(0x30c0, 0x0026);
	result += write16(0x301a, 0x10d4);

	// Errata item 6
	result += write16(to_reg(MT9V117_VAR_AE_TRACK, 0x0002), 0x00d3);
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x0078), 0x00a0);
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x0076), 0x0140);

	// Errata item 8
	result += write16(to_reg(MT9V117_VAR_LOW_LIGHT, 0x0004), 0x00fc);
	result += write16(to_reg(MT9V117_VAR_LOW_LIGHT, 0x0038), 0x007f);
	result += write16(to_reg(MT9V117_VAR_LOW_LIGHT, 0x003a), 0x007f);
	result += write16(to_reg(MT9V117_VAR_LOW_LIGHT, 0x003c), 0x007f);
	result += write16(to_reg(MT9V117_VAR_LOW_LIGHT, 0x0004), 0x00f4);

	// Patch 0403:
	// Critical sensor optimization
	result += write16(MT9V117_REG_ACCESS_CTL, 0x0001);
	result += write16(MT9V117_PHYSICAL_ADDRESS_ACCESS, 0x7000);

	// write patch lines
	for (size_t i = 0; i < MT9V117_PATCH_LINE_NUM; ++i) {
		int bytes_written = ::write(m_fd, (uint8_t *) MT9V117_patch[i].data, MT9V117_patch[i].size);

		if (bytes_written != MT9V117_patch[i].size) {
			DF_LOG_INFO("Patch line transmission failed");
			result += -1;
		}
	}

	result += write16(MT9V117_LOGICAL_ADDRESS_ACCESS, 0x0000);

	result += write16(to_reg(MT9V117_VAR_PATCHLDR, 0x00), 0x05d8);
	result += write16(to_reg(MT9V117_VAR_PATCHLDR, 0x02), 0x0403);
	result += write32(to_reg(MT9V117_VAR_PATCHLDR, 0x04), 0x00430104);

	result += write16(MT9V117_REG_CMD, HOST_CMD_OK | HOST_CMD_0);

	result += check_config_change(HOST_CMD_0);
	return result;
}

int MT9V117::check_config_change(uint16_t new_state)
{
	uint16_t retries = 10;
	uint16_t state = 0;
	uint16_t i = 0;

	for (i = 0; i < retries; ++i) {
		int result = read16(MT9V117_REG_CMD, &state);

		if (result == 0 && (state & new_state) == 0) {
			break;
		}

		usleep(15000); // 15ms
	}

	if (i >= retries) {
		DF_LOG_ERR("Timeout: Config change failed");
		return -1;
	}

	if ((state & HOST_CMD_OK) == 0) {
		DF_LOG_ERR("Config change failed");
		return -2;
	}

	return 0;
}

int MT9V117::write_settings()
{
	int result = 0;
	result += write32(to_reg(MT9V117_VAR_AWB, 0x40), 50000);
	result += write16(to_reg(MT9V117_VAR_AE_RULE, 0x04), MT9V117_AE_RULE_AVERAGE);

	uint16_t pad_slew;

	if (read16(MT9V117_REG_PAD_SLEW, &pad_slew) == 0) {
		uint16_t new_pad_slew = pad_slew | 0x0600 | 0x0001;
		result += write16(MT9V117_REG_PAD_SLEW, new_pad_slew);

	} else {
		DF_LOG_ERR("Read error");
		result += -1;
	}

	return result;
}

int MT9V117::configure_sensor()
{
	int result = 0;

	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x02), 16);  // X address startoffset
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x06), 663); // X address end
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x00), 8);   // Y address start
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x04), 501); // Y address end
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x14), 243); // cpipe last row
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x0E), 283); // frame length line
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x28), 4);   // read mode: y skip enabled
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x1A), 1);   // max FDZONE 60
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x1E), 1);   // target FDZONE 60

	result += write8(MT9V117_REG_AE_TRACK_JUMP_DIVISOR, 0x03);
	result += write8(MT9V117_REG_CAM_AET_SKIP_FRAMES, 0x02);

	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x54), 320); // frame width
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x56), 240); // frame height

	// Set gain metric for 90 fps
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x116), 0x03e8); // low light start gain metric
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x118), 0x1770); // low light stop gain metric

	// set crop window
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x48), 0); // crop window X offset
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x4A), 0); // crop window Y offset
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x4C), 640); // crop window width
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x4E), 240); // crop window height

	// Enable auto-stats mode
	result += write8(to_reg(MT9V117_VAR_CAM_CTRL, 0x50), 3); // crop mode
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0xF0), 319); // AWB HG window X end
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0xF2), 239); // AWB HG window Y end
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0xF4), 2); // AE initial window X start
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0xF6), 2); // AE initial window Y start
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0xF8), 65); // AE initial window X end
	result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0xFA), 49); // AE initial window Y end

	uint16_t itu656;

	if (read16(to_reg(MT9V117_VAR_CAM_CTRL, 0x58), &itu656) == 0) {
		uint16_t new_itu656 = itu656 | MT9V117_OUTPUT_FORMAT_BT656_ENABLE;
		result += write16(to_reg(MT9V117_VAR_CAM_CTRL, 0x58), new_itu656); // output format

	} else {
		result += -1;
	}

	result += write8(to_reg(MT9V117_VAR_SYSMGR, 0x00), 0x28); // next state

	result += write16(MT9V117_REG_CMD, HOST_CMD_OK | HOST_CMD_1);

	result += check_config_change(HOST_CMD_1);
	return result;
}

int MT9V117::set_format()
{
#if !(defined(__APPLE__) && defined(__MACH__))
// v4l2-subdev.h is not available on OSX
	struct v4l2_subdev_format fmt;
	int ret, fd;

	char device_path[] = "/dev/v4l-subdev0";

	fd = open(device_path, O_RDWR | O_CLOEXEC);

	if (fd < 0) {
		DF_LOG_ERR("Error open");
		return -1;
	}

	memset(&fmt, 0, sizeof(fmt));
	fmt.pad = 0;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.format.width = 320;
	fmt.format.height = 240;
	fmt.format.code = V4L2_MBUS_FMT_UYVY8_2X8;

	ret = ioctl(fd, VIDIOC_SUBDEV_S_FMT, &fmt);
	close(fd);

	if (ret < 0) {
		return -1;
	}

	return 0;
#else
	return -1;
#endif
}
