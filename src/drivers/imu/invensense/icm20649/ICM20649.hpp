/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

/**
 * @file ICM20649.hpp
 *
 * Driver for the Invensense ICM20649 connected via SPI.
 *
 */

#pragma once

#include "InvenSense_ICM20649_registers.hpp"

#include <drivers/drv_hrt.h>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/device/spi.h>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/ecl/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <cstdint>

class ICM20649 : public device::SPI, public I2CSPIDriver<ICM20649>
{
public:
	ICM20649(I2CSPIBusOption bus_option, int bus, uint32_t device, enum Rotation rotation, int bus_frequency,
		 spi_mode_e spi_mode, spi_drdy_gpio_t drdy_gpio);
	~ICM20649() override;

	static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
					     int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

	void Start();
	bool Reset();
protected:
	void custom_method(const BusCLIArguments &cli) override;
	void exit_and_cleanup() override;
private:
	static constexpr size_t SIZE = 512;

	// FIFO_FIFO_DATA layout when FIFO_EN has ACCEL_FIFO_EN and GYRO_{Z, Y, X}_FIFO_EN set
	struct FIFO_DATA {
		uint8_t ACCEL_XOUT_H;
		uint8_t ACCEL_XOUT_L;
		uint8_t ACCEL_YOUT_H;
		uint8_t ACCEL_YOUT_L;
		uint8_t ACCEL_ZOUT_H;
		uint8_t ACCEL_ZOUT_L;
		uint8_t GYRO_XOUT_H;
		uint8_t GYRO_XOUT_L;
		uint8_t GYRO_YOUT_H;
		uint8_t GYRO_YOUT_L;
		uint8_t GYRO_ZOUT_H;
		uint8_t GYRO_ZOUT_L;
	};

	static constexpr uint32_t GYRO_RATE{9000};  // 9 kHz gyro
	static constexpr uint32_t ACCEL_RATE{4500}; // 4.5 kHz accel
	static constexpr uint32_t FIFO_MAX_SAMPLES{ math::min(SIZE / sizeof(FIFO_DATA) + 1, sizeof(PX4Gyroscope::FIFOSample::x) / sizeof(PX4Gyroscope::FIFOSample::x[0]))};


	// Transfer FIFO_DATA
	struct TransferBuffer {
		uint8_t cmd;
		FIFO_DATA f[FIFO_MAX_SAMPLES];
	};
	// ensure no struct padding
	static_assert(sizeof(TransferBuffer) == (sizeof(uint8_t) + FIFO_MAX_SAMPLES *sizeof(FIFO_DATA)));

	int probe() override;
	bool fifo_accel_equal(const FIFO_DATA &f0, const FIFO_DATA &f1);

	void ConfigureAccel();
	void ConfigureGyro();
	void ConfigureSampleRate(int sample_rate);

	static int FIFO_DATAReadyInterruptCallback(int irq, void *context, void *arg);
	void FIFO_DATAReady();
	bool FIFO_DATAReadyInterruptConfigure();
	bool FIFO_DATAReadyInterruptDisable();

	uint16_t FIFOReadCount();
	bool FIFORead(const hrt_abstime &timestamp_sample, uint16_t samples);


	bool ProcessAccel(const hrt_abstime &timestamp_sample, const TransferBuffer *const buffer, uint8_t samples);
	void ProcessGyro(const hrt_abstime &timestamp_sample, const TransferBuffer *const buffer, uint8_t samples);
	void UpdateTemperature();

	void _set_filter_and_scaling(void);
	void _fifo_reset();
	int _FIFO_DATA_ready();
	int _block_read(uint16_t reg, uint8_t *buf, uint32_t size);
	uint8_t _register_read(uint16_t reg);
	void _register_write(uint16_t reg, uint8_t val);
	void _select_bank(uint8_t bank);
	uint8_t _current_bank {0xff};
	uint8_t _last_stat_user_ctrl = 0;
	uint8_t configure_status {false};

	uint8_t *_dma_FIFO_DATA_buffer{nullptr};

	PX4Accelerometer _px4_accel;
	PX4Gyroscope _px4_gyro;

	perf_counter_t _transfer_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": transfer")};
	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _fifo_empty_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO empty")};
	perf_counter_t _fifo_overflow_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO overflow")};
	perf_counter_t _fifo_reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": FIFO reset")};
	perf_counter_t _drdy_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": DRDY interval")};

	hrt_abstime _last_config_check_timestamp{0};
	hrt_abstime _fifo_watermark_interrupt_timestamp{0};
	hrt_abstime _temperature_update_timestamp{0};

	px4::atomic<uint8_t> _FIFO_DATA_ready_count{0};
	px4::atomic<uint8_t> _fifo_read_samples{0};


	enum class STATE : uint8_t {
		WAIT_FOR_CONFIGURE,
		FIFO_READ,
	};

	px4::atomic<STATE> _state{STATE::WAIT_FOR_CONFIGURE};

	uint16_t _fifo_empty_interval_us{1000}; // 1000 us / 1000 Hz transfer interval
	uint8_t _fifo_gyro_samples{static_cast<uint8_t>(_fifo_empty_interval_us / (1000000 / GYRO_RATE))};
	uint8_t _fifo_accel_samples{static_cast<uint8_t>(_fifo_empty_interval_us / (1000000 / ACCEL_RATE))};
};
