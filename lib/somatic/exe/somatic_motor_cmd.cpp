/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright notice, this list of
 *       conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright notice, this list of
 *       conditions and the following disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived from this software without
 *       specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA TECH RESEARCH
 * CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @date Aug 06, 2013
 * @author Saul Reynolds-Haertle
 * @file somatic_motor_cmd.cpp
 * @brief Sends motor commands to the given motor group.
 */

#include <somatic.h>
#include <somatic/daemon.h>
#include <somatic.pb-c.h>
#include <somatic/motor.h>
#include <ach.h>
#include <unistd.h>

#include <iomanip>

#include <argp.h>

/* ******************************************************************************************** */
// Options and variables

somatic_d_t daemon_cx;

static char cmd_chan_name[1024];
static char state_chan_name[1024];
static Somatic__MotorParam cmd_type = SOMATIC__MOTOR_PARAM__MOTOR_POSITION;

static double cmd_values[32];
static int motor_size;

/* ******************************************************************************************** */
/// The main thread
int main(int argc, char* argv[]) {

	somatic_motor_t motor;
	std::cout << argc << std::endl;

	for(int i = 1; i < argc; i++) {
		if (argc != 3 || (strcmp("--help", argv[i]) == 0)) {
			std::cout
				<< "Usage: " << argv[0] << " CHANNELBASE CMDTYPE" << std::endl
				<< "Reads motor commands from standard input and sends them to the specified" << std::endl
				<< "somatic motor daemon. Determines the motor size from the input and the" << std::endl
				<< "ach channels and command type from arguments." << std::endl
				<< "" << std::endl
				<< "  CHANNELBASE         Used to determine ach channel names. Uses" << std::endl
				<< "                      CHANNELBASE-cmd as the motor's command channel and" << std::endl
				<< "                      CHANNELBASE-state as the motor's state channel." << std::endl
				<< "  CMDTYPE             One of pos, vel, cur, halt, or reset. Functions" << std::endl
				<< "                      should be pretty clear." << std::endl
				<< "" << std::endl
				<< "Examples:" << std::endl
				<< "  echo 0 0 0 128 | " << argv[0] << "rgripper pos" << std::endl
				<< "  Assuming the normal channels are running, sets the right robotiq" << std::endl
				<< "  hand to be fully open in roughly the basic grasp position." << std::endl
				<< "" << std::endl
				<< "  echo 0 0 0 0 0 0 0 | " << argv[0] << "rlwa vel" << std::endl
				<< "  Sends a zero-velocity message to seven modeles owned by the daemon on" << std::endl
				<< "  ach channels rlwa-cmd and rlwa-state, probably the right arm." << std::endl
				<< "" << std::endl
				<< "  echo 0 0 0 0 0 0 0 | " << argv[0] << "rlwa halt" << std::endl
				<< "  Assuming normal channels and daemons, halts the right arm." << std::endl;
			exit(EXIT_FAILURE);
		}
	}

	// parse options
	memset(cmd_chan_name, 0, 1024);
	memset(state_chan_name, 0, 1024);
	sprintf(cmd_chan_name, "%s-cmd", argv[1]);
	sprintf(state_chan_name, "%s-state", argv[1]);

	if (strcmp("pos", argv[2]) == 0) cmd_type = SOMATIC__MOTOR_PARAM__MOTOR_POSITION;
	else if (strcmp("vel", argv[2]) == 0) cmd_type = SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY;
	else if (strcmp("cur", argv[2]) == 0) cmd_type = SOMATIC__MOTOR_PARAM__MOTOR_CURRENT;
	else if (strcmp("halt", argv[2]) == 0) cmd_type = SOMATIC__MOTOR_PARAM__MOTOR_HALT;
	else if (strcmp("reset", argv[2]) == 0) cmd_type = SOMATIC__MOTOR_PARAM__MOTOR_RESET;
	else {
		fprintf(stderr, "Must specify a valid command type: 'pos', 'vel', 'cur', 'halt', or 'reset'\n");
		exit(EXIT_FAILURE);
	}

	// init daemon
	somatic_d_opts_t daemon_opt;
	memset(&daemon_opt, 0, sizeof(daemon_opt)); // zero initialize
	daemon_opt.ident = "somatic_motor_cmd";
	somatic_d_init(&daemon_cx, &daemon_opt);

	// ==================================================================================
	// Process the input 

	// Read values out of standard input
	motor_size = 0;
	double tempd;
	int tempi;
	while(std::cin.good()) {
		std::cin.clear();
		std::cin >> tempd;
		if (std::cin.good()) {
			cmd_values[motor_size++] = tempd;
			continue;
		}

		std::cin.clear();
		std::cin >> std::hex >> tempi;
		if (std::cin.good()) {
			cmd_values[motor_size++] = (double)tempi;
			continue;
		}
	}
	if (!std::cin.eof()) {
		fprintf(stderr, "failed\n");
		exit(EXIT_FAILURE);
	}

	// print out what we plan to send
	std::cout << "Sending ";
	switch(cmd_type) {
		case SOMATIC__MOTOR_PARAM__MOTOR_POSITION: std::cout << "position "; break;
		case SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY: std::cout << "velocity "; break;
		case SOMATIC__MOTOR_PARAM__MOTOR_CURRENT: std::cout << "current "; break;
		case SOMATIC__MOTOR_PARAM__MOTOR_HALT: std::cout << "halt "; break;
		case SOMATIC__MOTOR_PARAM__MOTOR_RESET: std::cout << "reset "; break;
	}
	std::cout << "message to " << motor_size << " motors using channels "
			  << cmd_chan_name << " and " << state_chan_name << std::endl;
	for(int i = 0; i < motor_size; i++) 
		std::cout << std::setw(12) << std::right << std::fixed << cmd_values[i];
	std::cout << std::endl;

	// ==================================================================================
	// Initialize the motor group

	// Initialize the motor and reset it
	somatic_motor_init(&daemon_cx, &motor, motor_size, cmd_chan_name, state_chan_name);
	somatic_motor_reset(&daemon_cx, &motor);
	
	// Set the joint limits
	double** motor_minimum_values[] = { &motor.pos_valid_min, &motor.vel_valid_min,
									   &motor.pos_limit_min, &motor.vel_limit_min };
	double** motor_maximum_values[] = { &motor.pos_valid_max, &motor.vel_valid_max,
									   &motor.pos_limit_max, &motor.vel_limit_max };
	for(size_t i = 0; i < 4; i++) aa_fset(*motor_minimum_values[i], -1024.1, motor_size);
	for(size_t i = 0; i < 4; i++) aa_fset(*motor_maximum_values[i], 1024.1, motor_size);

	// Inform slogd of the group starting
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
		SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);

	// ==================================================================================
	// Send the requested command to the group

	// Update the motor
	somatic_motor_update(&daemon_cx, &motor);
	usleep(1e5);

	// Construct the command and send it
	somatic_motor_cmd(&daemon_cx, &motor, cmd_type, cmd_values, motor_size, NULL);
	
	// ==================================================================================
	// Clean up

	// Send stopping event
	somatic_d_event(&daemon_cx, SOMATIC__EVENT__PRIORITIES__NOTICE, 
		SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);

	// Close the motor
	somatic_motor_destroy(&daemon_cx, &motor);

	// Destroy daemon resources
	somatic_d_destroy(&daemon_cx);
	exit(0);
}
