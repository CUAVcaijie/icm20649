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
 * @file hardpoint.cpp
 *
 * @author Andreas Jochum <Andreas@NicaDrone.com>
 */

#include "safety_state.hpp"
#include <systemlib/err.h>
#include <px4_platform_common/time.h>
#include <uORB/Subscription.hpp>


UavcanUavcanSafetyState::UavcanUavcanSafetyState(uavcan::INode &node) :
	_node(node),
	_safety_state_pub(node),
	_timer(node)
{

}

UavcanUavcanSafetyState::~UavcanUavcanSafetyState()
{

}

int
UavcanUavcanSafetyState::init()
{
	/*
	 * Setup timer and call back function for periodic updates
	 */

	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &UavcanUavcanSafetyState::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	}
	return 0;
}

void
UavcanUavcanSafetyState::periodic_update(const uavcan::TimerEvent &)
{
	if (_actuator_armed_sub.updated()) {
		_actuator_armed_sub.copy(&_actuator_armed);

		com::hex::equipment::indication::SafetyStateCommand cmd;
		if(_actuator_armed.prearmed == true) {
			cmd.status = 255;
		}
		else {
			cmd.status = 0;
		}
		(void)_safety_state_pub.broadcast(cmd);
	}
}
