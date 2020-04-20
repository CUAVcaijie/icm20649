 /****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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

/**
 * @file rgb.hpp
 *
 * @author Andreas Jochum <Andreas@NicaDrone.com>
 */

#pragma once

#include <uavcan/uavcan.hpp>
#include <uavcan/equipment/indication/LightsCommand.hpp>
#include <perf/perf_counter.h>
#include <lib/led/led.h>

/**
 * @brief The RGB class
 */

class UavcanUavcanRgb
{
public:
	UavcanUavcanRgb(uavcan::INode &node);
	~UavcanUavcanRgb();

	/*
	* setup periodic updater
	*/
	int init();

private:
	/*
	 * Max update rate to avoid exessive bus traffic
	 */
	static constexpr unsigned			MAX_RATE_HZ = 100;	///< XXX make this configurable


	void periodic_update(const uavcan::TimerEvent &);

	typedef uavcan::MethodBinder<UavcanUavcanRgb *, void (UavcanUavcanRgb::*)(const uavcan::TimerEvent &)>
	TimerCbBinder;
	LedController		_led_controller;

	pthread_mutex_t					_node_mutex;
	/*
	 * libuavcan related things
	 */
	uavcan::INode							&_node;
	uavcan::Publisher<uavcan::equipment::indication::LightsCommand> _rgb_pub;
	uavcan::TimerEventForwarder<TimerCbBinder>			_timer;

};
