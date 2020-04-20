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

#include "beep.hpp"
#include <systemlib/err.h>
#include <px4_platform_common/time.h>
#include <uORB/Subscription.hpp>


UavcanUavcanBeep::UavcanUavcanBeep(uavcan::INode &node) :
	_node(node),
	_beep_pub(node),
	_timer(node)
{

}

UavcanUavcanBeep::~UavcanUavcanBeep()
{

}

int
UavcanUavcanBeep::init()
{
	/*
	 * Setup timer and call back function for periodic updates
	 */

	if (!_timer.isRunning()) {
		_timer.setCallback(TimerCbBinder(this, &UavcanUavcanBeep::periodic_update));
		_timer.startPeriodic(uavcan::MonotonicDuration::fromMSec(1000 / MAX_RATE_HZ));
	}

	return 0;
}

void
UavcanUavcanBeep::periodic_update(const uavcan::TimerEvent &)
{
	if (_tune_control_sub.updated()) {
		_tune_control_sub.copy(&_tune);

		if (_tune.timestamp > 0) {
			_play_tone = _tunes.set_control(_tune) == 0;
		}
	}

	unsigned int frequency = 0;
	unsigned int duration = 0;

	static hrt_abstime time = hrt_absolute_time();
	static unsigned int tmp = 0;

	if (hrt_absolute_time() - time <= tmp) { return; }

	time = hrt_absolute_time();

	if (_silence_length > 0) {
		duration = _silence_length;
		_silence_length = 0;

	} else if (_play_tone) {
		int parse_ret_val = _tunes.get_next_note(frequency, duration, _silence_length);

		if (parse_ret_val > 0) {
			// Continue playing.
			_play_tone = true;

			// A frequency of 0 corresponds to stop_note();
			if (frequency > 0) {
				// Start playing the note.
				uavcan::equipment::indication::BeepCommand cmd;
				cmd.frequency = frequency;
				cmd.duration = duration / 1000000.f;            // We don't want to incapacitate ESC for longer time that this
				(void)_beep_pub.broadcast(cmd);
			}

		} else {
			_play_tone = false;
		}

	} else {
		duration = _tunes.get_maximum_update_interval();
	}

	tmp = duration;
}
