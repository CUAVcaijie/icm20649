/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * OF USE, FIFO_DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
#include "ICM20649.hpp"

#include <px4_platform/board_dma_alloc.h>

using namespace time_literals;

static constexpr int16_t combine(uint8_t msb, uint8_t lsb)
{
	return (msb << 8u) | lsb;
}

ICM20649::ICM20649(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		   spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio) :
	SPI(MODULE_NAME, nullptr, bus, device, spi_mode, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus),

	_px4_accel(get_device_id(), ORB_PRIO_HIGH, rotation),
	_px4_gyro(get_device_id(), ORB_PRIO_HIGH, rotation)
{
	set_device_type(DRV_IMU_DEVTYPE_ICM20649);

	_px4_accel.set_device_type(DRV_IMU_DEVTYPE_ICM20649);
	_px4_gyro.set_device_type(DRV_IMU_DEVTYPE_ICM20649);

	ConfigureSampleRate(_px4_gyro.get_max_rate_hz());
}

ICM20649::~ICM20649()
{

	if (_dma_FIFO_DATA_buffer != nullptr) {
		board_dma_free(_dma_FIFO_DATA_buffer, SIZE);
	}

	perf_free(_transfer_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
	perf_free(_fifo_empty_perf);
	perf_free(_fifo_overflow_perf);
	perf_free(_fifo_reset_perf);
	perf_free(_drdy_interval_perf);
}

int ICM20649::init()
{
	if (SPI::init() != PX4_OK) {
		PX4_ERR("SPI::init failed");
		return false;
	}

	// allocate DMA capable buffer
	_dma_FIFO_DATA_buffer = (uint8_t *)board_dma_alloc(SIZE);

	if (_dma_FIFO_DATA_buffer == nullptr) {
		PX4_ERR("DMA alloc failed");
		printf("%s:DMA alloc failed\r\n", __FUNCTION__);
		return false;
	}

	_state.store(STATE::WAIT_FOR_CONFIGURE);

	printf("%s:init su %d \r\n", __FUNCTION__, true);
	ScheduleClear();
	ScheduleNow();
	configure_status = true;
	return 0;
}

bool ICM20649::Reset()
{
	ScheduleClear();
	ScheduleNow();
	return true;
}

void ICM20649::exit_and_cleanup()
{
	FIFO_DATAReadyInterruptDisable();
	I2CSPIDriverBase::exit_and_cleanup();
}

void ICM20649::print_status()
{
	PX4_INFO("FIFO empty interval: %d us (%.3f Hz)", _fifo_empty_interval_us,
		 static_cast<double>(1000000 / _fifo_empty_interval_us));

	perf_print_counter(_transfer_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
	perf_print_counter(_fifo_empty_perf);
	perf_print_counter(_fifo_overflow_perf);
	perf_print_counter(_fifo_reset_perf);
	perf_print_counter(_drdy_interval_perf);

	_px4_accel.print_status();
	_px4_gyro.print_status();
}

int ICM20649::probe()
{
	const uint8_t whoami = _register_read(ICM20649_WHO_AM_I);

	if (whoami != ICM20649_WHOAMI_ICM20649) {
		PX4_WARN("unexpected WHO_AM_I 0x%02x", whoami);
		return PX4_ERROR;
	}

	uint8_t tries;

	for (tries = 0; tries < 5; tries++) {
		_last_stat_user_ctrl = _register_read(ICM20649_USER_CTRL);

		/* First disable the master I2C to avoid hanging the slaves on the
		* aulixiliar I2C bus - it will be enabled again if the AuxiliaryBus
		* is used */
		if (_last_stat_user_ctrl & BIT_USER_CTRL_I2C_MST_EN) {
			_last_stat_user_ctrl &= ~BIT_USER_CTRL_I2C_MST_EN;
			_register_write(ICM20649_USER_CTRL, _last_stat_user_ctrl);
			px4_usleep(10000);
		}

		/* reset device */
		_register_write(ICM20649_PWR_MGMT_1, BIT_PWR_MGMT_1_DEVICE_RESET);
		px4_usleep(100000);

		_last_stat_user_ctrl |= BIT_USER_CTRL_I2C_IF_DIS;
		_register_write(ICM20649_USER_CTRL, _last_stat_user_ctrl);

		// Wake up device and select Auto clock. Note that the
		// Invensense starts up in sleep mode, and it can take some time
		// for it to come out of sleep
		_register_write(ICM20649_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_AUTO);
		px4_usleep(5000);

		// check it has woken up
		if (_register_read(ICM20649_PWR_MGMT_1) == BIT_PWR_MGMT_1_CLK_AUTO) {
			break;
		}

		px4_usleep(10000);

		if (_FIFO_DATA_ready()) {
			break;
		}
	}

	if (tries == 5) {
		printf("%s:Failed to boot Invensense 5 times\r\n", __FUNCTION__);
		return PX4_ERROR;
	}

	_register_write(ICM20649_PWR_MGMT_2, 0x00);

	px4_usleep(1000);
	// always use FIFO
	_fifo_reset();

	// setup on-sensor filtering and scaling
	_set_filter_and_scaling();
	px4_usleep(1000);
	ConfigureAccel();
	ConfigureGyro();

	// set sample rate to 1.125KHz
	_register_write(ICM20649_GYRO_SMPLRT_DIV, 0);

	px4_usleep(1);
	// configure interrupt to fire when new FIFO_DATA arrives
	_register_write(ICM20649_INT_ENABLE_1, 0x01);
	px4_usleep(1);
	return PX4_OK;
}

void ICM20649::RunImpl()
{
	switch (_state.load()) {
	case STATE::WAIT_FOR_CONFIGURE:

		if (configure_status) {
			_state.store(STATE::FIFO_READ);
			ScheduleOnInterval(_fifo_empty_interval_us, _fifo_empty_interval_us);

		} else {
			ScheduleDelayed(10_ms);
		}

		break;

	case STATE::FIFO_READ: {
			uint8_t samples = 0;
			hrt_abstime timestamp_sample = 0;

			if ((hrt_elapsed_time(&timestamp_sample) > (_fifo_empty_interval_us / 2))) {

				// use the time now roughly corresponding with the last sample we'll pull from the FIFO
				timestamp_sample = hrt_absolute_time();
				const uint16_t fifo_count = FIFOReadCount();

				if (fifo_count == 0) {
					//failure = true;
					perf_count(_fifo_empty_perf);
				}

				samples = (fifo_count / sizeof(FIFO_DATA) / 2) * 2; // round down to nearest 2
			}

			if (samples >= 2) {
				// require at least 2 samples (we want at least 1 new accel sample per transfer)
				if (!FIFORead(timestamp_sample, samples)) {
					_px4_accel.increase_error_count();
					_px4_gyro.increase_error_count();
				}
			}

			if (samples > FIFO_MAX_SAMPLES) {
				// not technically an overflow, but more samples than we expected or can publish
				perf_count(_fifo_overflow_perf);
				_fifo_reset();
			}

			if (hrt_elapsed_time(&_temperature_update_timestamp) > 1_s) {
				UpdateTemperature();
				_temperature_update_timestamp = timestamp_sample;
			}

			break;
		}
		break;
	}
}

void ICM20649::ConfigureAccel()
{
	const uint8_t ACCEL_FS_SEL = _register_read(ICM20649_ACCEL_CONFIG) & (Bit2 | Bit1); // 2:1 ACCEL_FS_SEL[1:0]

	switch (ACCEL_FS_SEL) {
	case ACCEL_FS_SEL_4G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 8192);
		_px4_accel.set_range(4 * CONSTANTS_ONE_G);
		break;

	case ACCEL_FS_SEL_8G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 4096);
		_px4_accel.set_range(8 * CONSTANTS_ONE_G);
		break;

	case ACCEL_FS_SEL_16G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 2048);
		_px4_accel.set_range(16 * CONSTANTS_ONE_G);
		break;

	case ACCEL_FS_SEL_32G:
		_px4_accel.set_scale(CONSTANTS_ONE_G / 1024);
		_px4_accel.set_range(32 * CONSTANTS_ONE_G);
		break;
	}
}

void ICM20649::ConfigureGyro()
{
	const uint8_t GYRO_FS_SEL = _register_read(ICM20649_GYRO_CONFIG_1) & (Bit2 | Bit1); // 2:1 GYRO_FS_SEL[1:0]

	switch (GYRO_FS_SEL) {
	case GYRO_FS_SEL_500_DPS:
		_px4_gyro.set_scale(math::radians(1.0f / 65.5f));
		_px4_gyro.set_range(math::radians(500.f));
		break;

	case GYRO_FS_SEL_1000_DPS:
		_px4_gyro.set_scale(math::radians(1.0f / 32.8f));
		_px4_gyro.set_range(math::radians(1000.f));
		break;

	case GYRO_FS_SEL_2000_DPS:
		_px4_gyro.set_scale(math::radians(1.0f / 16.4f));
		_px4_gyro.set_range(math::radians(2000.0f));
		break;

	case GYRO_FS_SEL_4000_DPS:
		_px4_gyro.set_scale(math::radians(1.0f / 8.2f));
		_px4_gyro.set_range(math::radians(4000.0f));
		break;
	}
}

void ICM20649::ConfigureSampleRate(int sample_rate)
{
	if (sample_rate == 0) {
		sample_rate = 1000; // default to 1 kHz
	}

	static constexpr float sample_min_interval = (1000.f / 4.5f);

	_fifo_empty_interval_us = math::max(((1000000 / sample_rate) / sample_min_interval) * sample_min_interval,
					    sample_min_interval); // round down to nearest sample_min_interval
	_fifo_gyro_samples = math::min(_fifo_empty_interval_us / (1000000 / GYRO_RATE), FIFO_MAX_SAMPLES);

	// recompute FIFO empty interval (us) with actual gyro sample limit
	_fifo_empty_interval_us = _fifo_gyro_samples * (1000000 / GYRO_RATE);

	_fifo_accel_samples = math::min(_fifo_empty_interval_us / (1000000 / ACCEL_RATE), FIFO_MAX_SAMPLES);

	_px4_accel.set_update_rate(1000000 / _fifo_empty_interval_us);
	_px4_gyro.set_update_rate(1000000 / _fifo_empty_interval_us);
}

int ICM20649::FIFO_DATAReadyInterruptCallback(int irq, void *context, void *arg)
{
	static_cast<ICM20649 *>(arg)->FIFO_DATAReady();
	return 0;
}

void ICM20649::FIFO_DATAReady()
{
	if (_FIFO_DATA_ready_count.fetch_add(1) >= (_fifo_gyro_samples - 1)) {
		_FIFO_DATA_ready_count.store(0);
		_fifo_watermark_interrupt_timestamp = hrt_absolute_time();
		_fifo_read_samples.store(_fifo_gyro_samples);
		ScheduleNow();
	}

	perf_count(_drdy_interval_perf);
}

bool ICM20649::FIFO_DATAReadyInterruptConfigure()
{
	int ret_setevent = -1;

	// Setup FIFO_DATA ready on rising edge
	// TODO: cleanup horrible DRDY define mess


	return (ret_setevent == 0);
}

bool ICM20649::FIFO_DATAReadyInterruptDisable()
{
	int ret_setevent = -1;

	// Disable FIFO_DATA ready callback
	// TODO: cleanup horrible DRDY define mess


	return (ret_setevent == 0);
}

uint16_t ICM20649::FIFOReadCount()
{
	uint8_t cmd[3] = {0};
	_block_read(ICM20649_FIFO_COUNTH, cmd, 2);

	return combine(cmd[1], cmd[2]);
}

bool ICM20649::FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples)
{
	TransferBuffer *report = (TransferBuffer *)_dma_FIFO_DATA_buffer;

	if (samples > 32) { samples = 32; }

	const size_t transfer_size = math::min(samples * sizeof(FIFO_DATA) + 1, SIZE);
	memset(report, 0, transfer_size);

	// report->cmd_bank_select = static_cast<uint8_t>(Register::BANK_0::REG_BANK_SEL);
	// report->bank = REG_BANK_SEL_BIT::USER_BANK_0;
	report->cmd = static_cast<uint8_t>(0x72) | 0x80;

	perf_begin(_transfer_perf);

	uint8_t cmd[2] {
		(uint8_t)0x7F,
		0x00,
	};
	transfer(cmd, cmd, 2);

	if (transfer(_dma_FIFO_DATA_buffer, _dma_FIFO_DATA_buffer, transfer_size) != PX4_OK) {
		perf_end(_transfer_perf);
		perf_count(_bad_transfer_perf);
		return false;
	}

	perf_end(_transfer_perf);

	//return false;
	ProcessGyro(timestamp_sample, report, samples);
	return ProcessAccel(timestamp_sample, report, samples);
}

bool ICM20649::fifo_accel_equal(const FIFO_DATA &f0, const FIFO_DATA &f1)
{
	return (memcmp(&f0.ACCEL_XOUT_H, &f1.ACCEL_XOUT_H, 6) == 0);
}

bool ICM20649::ProcessAccel(const hrt_abstime &timestamp_sample, const TransferBuffer *const report, uint8_t samples)
{
	PX4Accelerometer::FIFOSample accel;

	accel.timestamp_sample = timestamp_sample;
	accel.dt = _fifo_empty_interval_us / _fifo_accel_samples;

	bool bad_FIFO_DATA = false;

	// accel FIFO_DATA is doubled in FIFO, but might be shifted
	int accel_first_sample = 1;

	if (samples >= 3) {
		if (fifo_accel_equal(report->f[0], report->f[1])) {
			// [A0, A1, A2, A3]
			//  A0==A1, A2==A3
			accel_first_sample = 1;

		} else if (fifo_accel_equal(report->f[1], report->f[2])) {
			// [A0, A1, A2, A3]
			//  A0, A1==A2, A3
			accel_first_sample = 0;

		} else {
			perf_count(_bad_transfer_perf);
			bad_FIFO_DATA = true;
		}
	}

	int accel_samples = 0;

	if (accel_first_sample == 1) {
		accel_samples = 0;
	}

	for (int i = accel_first_sample; i < samples; i = i + 2) {
		const FIFO_DATA &fifo_sample = report->f[i];
		int16_t accel_y = combine(fifo_sample.ACCEL_XOUT_H, fifo_sample.ACCEL_XOUT_L);
		int16_t accel_x = combine(fifo_sample.ACCEL_YOUT_H, fifo_sample.ACCEL_YOUT_L);
		int16_t accel_z = -combine(fifo_sample.ACCEL_ZOUT_H, fifo_sample.ACCEL_ZOUT_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[accel_samples] = accel_x;
		accel.y[accel_samples] = (accel_y == INT16_MIN) ? INT16_MAX : -accel_y;
		accel.z[accel_samples] = (accel_z == INT16_MIN) ? INT16_MAX : -accel_z;
		accel_samples++;
	}

	accel.samples = accel_samples;
	_px4_accel.updateFIFO(accel);

	return !bad_FIFO_DATA;
}

void ICM20649::ProcessGyro(const hrt_abstime &timestamp_sample, const TransferBuffer *const report, uint8_t samples)
{
	PX4Gyroscope::FIFOSample gyro;

	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = samples;
	gyro.dt = _fifo_empty_interval_us / _fifo_gyro_samples;

	for (int i = 0; i < samples; i++) {
		const FIFO_DATA &fifo_sample = report->f[i];

		const int16_t gyro_y  = combine(fifo_sample.GYRO_XOUT_H, fifo_sample.GYRO_XOUT_L);
		const int16_t gyro_x = combine(fifo_sample.GYRO_YOUT_H, fifo_sample.GYRO_YOUT_L);
		const int16_t gyro_z = -combine(fifo_sample.GYRO_ZOUT_H, fifo_sample.GYRO_ZOUT_L);

		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro_x;
		gyro.y[i] = (gyro_y == INT16_MIN) ? INT16_MAX : -gyro_y;
		gyro.z[i] = (gyro_z == INT16_MIN) ? INT16_MAX : -gyro_z;
	}

	_px4_gyro.updateFIFO(gyro);
}

void ICM20649::UpdateTemperature()
{
	// read current temperature
	uint8_t temperature_buf[3] {};
	_block_read(ICM20649_TEMP_OUT_H, temperature_buf, 2);

	const int16_t TEMP_OUT = combine(temperature_buf[1], temperature_buf[2]);
	const float TEMP_degC = ((TEMP_OUT - ROOM_TEMPERATURE_OFFSET) / TEMPERATURE_SENSITIVITY) + ROOM_TEMPERATURE_OFFSET;

	if (PX4_ISFINITE(TEMP_degC)) {
		_px4_accel.set_temperature(TEMP_degC);
		_px4_gyro.set_temperature(TEMP_degC);
	}
}

void ICM20649::_set_filter_and_scaling(void)
{
	uint8_t gyro_config = BITS_GYRO_FS_2000DPS_20649;
	uint8_t accel_config = BITS_ACCEL_FS_30G_20649;

	// assume 1.125kHz sampling to start

	_register_write(ICM20649_I2C_SLV4_CTRL, 0x1F);

	gyro_config |= BIT_GYRO_NODLPF_9KHZ;
	accel_config |= BIT_ACCEL_NODLPF_4_5KHZ;

	for (uint8_t i = 0; i < 5; i++) {
		_register_write(ICM20649_GYRO_CONFIG_1, gyro_config);
		_register_write(ICM20649_ACCEL_CONFIG, accel_config);
		_register_write(ICM20649_FIFO_MODE, 0xF);
		px4_usleep(5 * 1000);

		if (_register_read(ICM20649_GYRO_CONFIG_1) == gyro_config &&
		    _register_read(ICM20649_ACCEL_CONFIG) == accel_config) {
			return;
		}
	}

}

void ICM20649::_fifo_reset()
{
	uint8_t user_ctrl = _last_stat_user_ctrl;
	user_ctrl &= ~(BIT_USER_CTRL_FIFO_EN);

	perf_count(_fifo_reset_perf);

	_register_write(ICM20649_FIFO_EN_2, 0);
	_register_write(ICM20649_USER_CTRL, user_ctrl);
	_register_write(ICM20649_FIFO_RST, 0x0F);
	_register_write(ICM20649_FIFO_RST, 0x00);
	_register_write(ICM20649_USER_CTRL, user_ctrl | BIT_USER_CTRL_FIFO_EN);
	_register_write(ICM20649_FIFO_EN_2, BIT_XG_FIFO_EN | BIT_YG_FIFO_EN |
			BIT_ZG_FIFO_EN | BIT_ACCEL_FIFO_EN);

	_last_stat_user_ctrl = user_ctrl | BIT_USER_CTRL_FIFO_EN;

	_FIFO_DATA_ready_count.store(0);
	_fifo_watermark_interrupt_timestamp = 0;
	_fifo_read_samples.store(0);
}

int ICM20649::_FIFO_DATA_ready()
{
	uint8_t status = _register_read(ICM20649_INT_STATUS_1);
	return status != 0;
}

int ICM20649::_block_read(uint16_t reg, uint8_t *buf, uint32_t size)
{
	_select_bank(GET_BANK(reg));

	buf[0] = reg | DIR_READ;
	buf[1] = 0;
	buf[2] = 0;

	transfer(buf, buf, size + 1);
	return 1;
}

uint8_t ICM20649::_register_read(uint16_t reg)
{
	uint8_t val = 0;
	_select_bank(GET_BANK(reg));
	uint8_t cmd[2] {
		uint8_t(GET_REG(reg) | 0x80),
		val
	};
	transfer(cmd, cmd, 2);

	return cmd[1];
}

void ICM20649::_register_write(uint16_t reg, uint8_t val)
{
	_select_bank(GET_BANK(reg));
	uint8_t cmd[2] {
		uint8_t(GET_REG(reg)),
		val,
	};
	transfer(cmd, cmd, 2);
}

void ICM20649::_select_bank(uint8_t bank)
{
	if (_current_bank != bank) {

		uint8_t cmd[2] {
			ICM20649_BANK_SEL,
			uint8_t(bank << 4),
		};
		transfer(cmd, cmd, 2);
		_current_bank = bank;
	}
}
