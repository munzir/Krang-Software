/* -*- mode: C; c-basic-offset: 2  -*- */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */


/** Author: Tobias Kunz
 */

#include <argp.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <unistd.h>
#include <algorithm>
#include <cmath>
#include <iostream>

#include <ach.h>
#include <somatic.h>
#include <somatic/util.h>
#include <somatic.pb-c.h>
#include <somatic/msg/joystick.h>
#include <somatic/msg/motor.h>

#include <jachd.h>
#include <pciod.h>

using namespace std;

/* ---------- */
/* ARGP Junk  */
/* ---------- */
static int opt_verbosity = 0;
static const char *opt_joystick_chan = JOYSTICK_CHANNEL_NAME;
static const char *opt_cmd_chan = PCIOD_CMD_CHANNEL_NAME;
static const char *opt_state_chan = PCIOD_STATE_CHANNEL_NAME;

static struct argp_option options[] = {
		{ "verbose", 'v', 0, 0, "Causes verbose output", 0 },
		{ "commands",'c', "channel", 0, "ach channel name for sending powercube commands", 0  },
		{ "states", 's', "channel", 0, "ach channel name for receiving pcio state messages", 0  },
		{ "joystick", 'j', "channel", 0, "ach channel name for receiving joystick messages", 0  },
		{ 0, 0, 0, 0, 0, 0 }
};

/// argp parsing function
static int parse_opt( int key, char *arg, struct argp_state *state);
/// argp program version
const char *argp_program_version = "torso v0.1";
/// argp program arguments documention
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "Basic torso control";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };
static int parse_opt( int key, char *arg, struct argp_state *state) {
	(void) state; // ignore unused parameter
	switch(key) {
	case 'v':
		opt_verbosity++;
		break;
	case 'c':
		opt_cmd_chan = strdup(arg);
		break;
	case 's':
		opt_state_chan = strdup(arg);
		break;
	case 'j':
		opt_joystick_chan = strdup(arg);
		break;
	case 0:
		break;
	}
	return 0;
}

/* --------- */
/* CONTROLLER*/
/* --------- */
void torso_control(double& current, double& position_setpoint, double& velocity_setpoint, const double& position, const double& velocity, const double& torso_direction, const double& dt) {

	static bool control_init = false;
	if(!control_init) {
		position_setpoint = position;
		control_init = true;
	}

	double acceleration_setpoint = 0.0;

	if (torso_direction > 0.9 && position < 2.5) {
		if(velocity_setpoint < 0.3)
			acceleration_setpoint = 0.5;
		else
			acceleration_setpoint = 0.0;
		velocity_setpoint = min(0.3, velocity_setpoint + acceleration_setpoint * dt);

	}
	else if (torso_direction < -0.9 && position > -0.458) {
		if(velocity_setpoint > -0.3)
			acceleration_setpoint = -0.5;
		else
			acceleration_setpoint = 0.0;
		velocity_setpoint = max(-0.3, velocity_setpoint + acceleration_setpoint * dt);
	}
	else if(velocity_setpoint > 0.0) {
		acceleration_setpoint = -3.0;
		velocity_setpoint = max(0.0, velocity_setpoint + acceleration_setpoint * dt);
	}
	else if(velocity_setpoint < 0.0) {
		acceleration_setpoint = 3.0;
		velocity_setpoint = min(0.0, velocity_setpoint + acceleration_setpoint * dt);
	}

	position_setpoint += velocity_setpoint * dt;

	current = - 100.0 * (position - position_setpoint) - 0.0 * (velocity - velocity_setpoint);
	current = max(-3.5, min(3.5, current));

}

/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {

	argp_parse (&argp, argc, argv, 0, NULL, NULL);

	// install signal handler
	somatic_sighandler_simple_install();

	// Open ACH channels for PCIO state and commands
	ach_channel_t pcio_state_chan;
	ach_channel_t pcio_cmd_chan;
	int r  = ach_open( &pcio_state_chan, opt_state_chan, NULL );
	r |= ach_open( &pcio_cmd_chan, opt_cmd_chan,   NULL );
	ach_flush(&pcio_cmd_chan);
	aa_hard_assert(r == ACH_OK,
			"Ach failure %s on opening PCIO channel (%s, line %d)\n",
			ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	// Open ACH channels for Joystick
	ach_channel_t js_chan;
	r  = ach_open( &js_chan,  opt_joystick_chan , NULL );
	aa_hard_assert(r == ACH_OK,
			"Ach failure %s on opening Joystick channel (%s, line %d)\n",
			ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	if (opt_verbosity) {
		fprintf(stderr, "\n* js_tele *\n");
		fprintf(stderr, "Verbosity:    %d\n", opt_verbosity);
		fprintf(stderr, "command channel:      %s\n", opt_cmd_chan);
		fprintf(stderr, "state channel:      %s\n", opt_state_chan);
		fprintf(stderr, "joystick channel:      %s\n", opt_joystick_chan);
		fprintf(stderr, "-------\n");
	}

	/*
	 * LOOP
	 */
	double dt = 0.0;
	struct timespec t_now, t_prev;
	t_prev = aa_tm_now();
	bool halted = true;
	double position = 0.0;
	double velocity = 0.0;
	double torso_direction = 0.0;
	bool motion_enabled = false;			// For safety

	while (!somatic_sig_received) {

		t_now = aa_tm_now();										// get current time
		dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	// get time difference

		/*
		 * Read joystick channel
		 */
		Somatic__Joystick *js_msg =
				SOMATIC_GET_LAST_UNPACK( r, somatic__joystick, &protobuf_c_system_allocator, JOYSTICK_CHANNEL_SIZE, &js_chan );

		aa_hard_assert(r == ACH_OK || r == ACH_STALE_FRAMES,
				"Ach failure %s on joystick data receive (%s, line %d)\n",
				ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

		if(r == ACH_OK) {

			// In order to drive the torso, Button No.6 must also be pressed down.
			motion_enabled = js_msg->buttons->data[5];
			if (motion_enabled)
				torso_direction	= js_msg->axes->data[5];

			somatic__joystick__free_unpacked( js_msg, &protobuf_c_system_allocator );
		}

		/*
		 * Read pcio state channel
		 */
		struct timespec abstime = aa_tm_future( aa_tm_sec2timespec( 1.0 / 30.0 ));

		Somatic__MotorState *pcio_state =
				SOMATIC_WAIT_LAST_UNPACK( r, somatic__motor_state, &protobuf_c_system_allocator, 1024, &pcio_state_chan, &abstime );
		aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT,
				"Ach wait failure %s on pcio data receive (%s, line %d)\n",
				ach_result_to_string(static_cast<ach_status_t>(r)),
				__FILE__, __LINE__);

		if (r == ACH_OK) {
			position = pcio_state->position->data[0] - 0.4889; // Kasemsit: We set torso zero position to be at the joint limit. The value 0.4889 [rad] is where all three robot's axes aligned.
			velocity = pcio_state->velocity->data[0];
			somatic__motor_state__free_unpacked(pcio_state, &protobuf_c_system_allocator);

			//printf("%f\t%f\n", position, velocity);
		}

		/*
		 * Controller
		 */
		double current, position_setpoint, velocity_setpoint;
		torso_control(current, position_setpoint, velocity_setpoint, position, velocity, torso_direction, dt);

		/*
		 * Send motor command
		 */

		size_t n_modules = 2;
		double cmd[] = {current, -current};

		if((fabs(velocity_setpoint) < 1e-6) && fabs(position - position_setpoint) < 0.05) {
			somatic_generate_motorcmd(&pcio_cmd_chan, cmd, n_modules, SOMATIC__MOTOR_PARAM__MOTOR_HALT);
			position_setpoint = position;
			halted = true;
		}
		else {
			if(halted) {
				somatic_generate_motorcmd(&pcio_cmd_chan, cmd, n_modules, SOMATIC__MOTOR_PARAM__MOTOR_RESET);
				halted = false;
			}
			else {
				somatic_generate_motorcmd(&pcio_cmd_chan, cmd, n_modules, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT);
			}
		}

		// update previous measured time
		t_prev = t_now;
	}

	ach_close(&pcio_cmd_chan);
	ach_close(&pcio_state_chan);
	ach_close(&js_chan);

	return 0;
}
