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

/**
 * @file krang-waist.cpp
 * @author Tobias Kunz, Munzir Zafar
 * @date Jun 13, 2013
 * @briefs This file is the main code for waist control driver. Originally
 * written by Tobias Kunz for allowing the joystick to control the 
 * waist modules on krang. We need the following additional functionality:
 * 1. Instead of joystick allow another channel to communicate to daemon
 * 2. Allow the waist modules to come to current mode when desired
 */


// FIXME: some kind of error handling

#include <argp.h>
#include <pthread.h>
#include <somatic.h>
#include <somatic/daemon.h>
#include <unistd.h>

somatic_d_t context;
somatic_d_opts_t context_options;
ach_channel_t pcio_state_chan;
ach_channel_t waistd_chan;
ach_channel_t pcio_cmd_chan;
size_t n_modules = 2;
Somatic__MotorCmd *cmd_msg = somatic_motor_cmd_alloc(n_modules);

// Keyboard adjusted globals
double KP = 0.0;
double KD = 10.0;
double GRAVITY_COMP = 1.027;
double CURRENT_LIMIT = 1.0;
double VELOCITY_LIMIT = 0.5;

/* ---------- */
/* ARGP Junk  */
/* ---------- */
bool opt_verbosity = false;
static const char *opt_waistd_chan = "waistd-cmd";
static const char *opt_cmd_chan = "";
static const char *opt_state_chan = "";

static struct argp_option options[] = {
	{ "verbose", 'v', 0, 0, "Causes verbose output", 0 },
	{ "commands",'c', "channel", 0, "ach channel name for sending powercube commands", 0  },
	{ "states", 's', "channel", 0, "ach channel name for receiving pcio state messages", 0  },
	{ "waistd-cmd", 'w', "waistd-cmd", 0, "ach channel name for receiving commands from a highr level program", 0  },
	{ "daemonize", 'd', NULL, 0, "fork off daemon process"},
	{ "ident", 'I', "IDENT", 0, "identifier for this daemon"},
	{ 0, 0, 0, 0, 0, 0 }
};

/* ********************************************************************************************* */
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
		opt_verbosity=true;
		break;
	case 'c':
		opt_cmd_chan = strdup(arg);
		break;
	case 's':
		opt_state_chan = strdup(arg);
		break;
	case 'w':
		opt_waistd_chan = strdup(arg);
		break;
	case 0:
		break;
	}
	
	somatic_d_argp_parse(key, arg, &context_options);
	return 0;
}

/* ********************************************************************************************* */
/// NOTE: Only use in debug mode but never as a daemon - eats up the cpu!
void *kbhit(void *) {
	// If the input is a or 0, accept it, and change the state
	char input;
	while(true){ 
		input=std::cin.get(); 
		switch(input) {
			case '1': KP += 0.1; break;
			case '2': KP -= 0.1; break;
			case '3': KD += 0.3; break;
			case '4': KD -= 0.3; break;
			case '5': GRAVITY_COMP += 0.001; break;
			case '6': GRAVITY_COMP -= 0.001; break;
			case '7': CURRENT_LIMIT += 0.1; break;
			case '8': CURRENT_LIMIT -= 0.1; break;
			case '9': VELOCITY_LIMIT += 0.1; break;
			case '0': VELOCITY_LIMIT -= 0.1; break;
		}
	}
}


/************************************************************************************************/
// Controller
void waist_control(double& current, double& position_setpoint, double& velocity_setpoint, 
	const double& position, const double& velocity, const double& torso_direction, 
	const double& dt) {
/*
	// Parameters
	static double CURRENT_LIMIT = 1.0;  //<< Current will be limited to -2-limit, -2+limit
	static double VELOCITY_LIMIT = 0.5;
*/
	// ============================================================================================
	// On first iteration decide initial set_point values for position and veclocity
	static bool control_init = false;
	if(!control_init) {
		position_setpoint = position;
		velocity_setpoint = 0.0;
		control_init = true;
	}
	double acceleration_setpoint = 0.0;

	// =============================================================================================
	// Determine velocity_setpoint based on joystick input "torso_direction"

	// "Move Up Command" and position limit not reached
	if (torso_direction > 0.9) {
		// Accelerate only if speed is less than 0.5
		if(velocity_setpoint < VELOCITY_LIMIT)
			acceleration_setpoint = 0.5;
		else
			acceleration_setpoint = 0.0;
		velocity_setpoint = std::min(VELOCITY_LIMIT, velocity_setpoint + acceleration_setpoint * dt);
	}

	// "Move-down Commnd" and position limit not reached 
	else if (torso_direction < -0.9) {
		// Accelerate only if speed is less than 0.3
		if(velocity_setpoint > -VELOCITY_LIMIT)
			acceleration_setpoint = -0.5;
		else
			acceleration_setpoint = 0.0;
		velocity_setpoint = std::max(-VELOCITY_LIMIT, velocity_setpoint + acceleration_setpoint * dt);
	}

	// No command or position limit reached
	// Decelerate until speed is zero
	else if(velocity_setpoint > 0.0) {
		acceleration_setpoint = -3.0;
		velocity_setpoint = std::max(0.0, velocity_setpoint + acceleration_setpoint * dt);
	}
	else if(velocity_setpoint < 0.0) {
		acceleration_setpoint = 3.0;
		velocity_setpoint = std::min(0.0, velocity_setpoint + acceleration_setpoint * dt);
	}
	
	// =============================================================================================
	// Determine position_setpoint based on velocity_setpoint 
	position_setpoint += velocity_setpoint * dt;

	// =============================================================================================
	// Determine current command to the motor based on position and velocity set_points
	current = - KP * (position - position_setpoint) - KD * (velocity - velocity_setpoint);
	current = std::max(-CURRENT_LIMIT, std::min(CURRENT_LIMIT, current));
	// printf("\two-gcomp: %8.3lf\t", current);
	current -= GRAVITY_COMP;
	// printf("after-gcomp: %7.3lf\n", current);
}

/* ******************************************************************************************** */
void init () {

	// Initialize the daemon
	context_options.ident = "krang-waist";
	context_options.sched_rt = SOMATIC_D_SCHED_UI;
	somatic_d_init(&context, &context_options); 

	// install signal handler
	somatic_sighandler_simple_install();

	// Open ACH channels for PCIO state and commands
	int r  = ach_open( &pcio_state_chan, opt_state_chan, NULL );
	r |= ach_open( &pcio_cmd_chan, opt_cmd_chan,   NULL );
	ach_flush(&pcio_cmd_chan);
	aa_hard_assert(r == ACH_OK,
			"Ach failure %s on opening PCIO channel (%s, line %d)\n",
			ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
	double vals[]={0.0, 0.0};
	somatic_motor_cmd_set( cmd_msg, SOMATIC__MOTOR_PARAM__MOTOR_RESET, vals, n_modules, NULL );
  SOMATIC_PACK_SEND( &pcio_cmd_chan, somatic__motor_cmd, cmd_msg );
	usleep(1e5);
	
	// Create a thread for polling the keyboard for a hit
	pthread_t kbhitThread;
//	pthread_create(&kbhitThread, NULL, &kbhit, NULL);
	
	// Open ACH channels for receiving commands from higher level program
	r  = ach_open( &waistd_chan,  opt_waistd_chan , NULL );
	aa_hard_assert(r == ACH_OK,
			"Ach failure %s on opening waistd channel (%s, line %d)\n",
			ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);

	if (opt_verbosity) {
		fprintf(stderr, "\n* js_tele *\n");
		fprintf(stderr, "Verbosity:    %d\n", opt_verbosity);
		fprintf(stderr, "command channel:      %s\n", opt_cmd_chan);
		fprintf(stderr, "state channel:      %s\n", opt_state_chan);
		fprintf(stderr, "waist daemon command channel:      %s\n", opt_waistd_chan);
		fprintf(stderr, "-------\n");
	}
}

/* ******************************************************************************************** */
void run () {

	double dt = 0.0;
	struct timespec t_now, t_prev;
	t_prev = aa_tm_now();
	bool halted = true;
	double position = 0.0;
	double velocity = 0.0;
	double torso_direction = 0.0;
	bool current_mode = false;			// For safety
	bool real_current_mode = false;			// For safety
	double real_current = 0.0;

	double position_setpoint;
	double velocity_setpoint;
	bool reset = false;

	// Send a starting event
	somatic_d_event(&context, SOMATIC__EVENT__PRIORITIES__NOTICE,
	                SOMATIC__EVENT__CODES__PROC_RUNNING, NULL, NULL);
	
	int r;
	while (!somatic_sig_received) {
		
		t_now = aa_tm_now();										// get current time
		dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	// get time difference

		// ===========================================================================================
		// Print the control parameters
		// printf("KP = %5.2lf, KD = %5.2lf, GCOMP = %6.3lf, CLIM = %5.2lf, VLIM = %5.2lf\n",
		//	KP, KD, GRAVITY_COMP, CURRENT_LIMIT, VELOCITY_LIMIT);

		// ===========================================================================================
		// Read commands from a high-level program

		// Wait to receive a new mssage on the waistd-ach channel for 1/10 seconds
		struct timespec currTime;
		clock_gettime( CLOCK_MONOTONIC, &currTime);
		struct timespec abstime = aa_tm_add(aa_tm_sec2timespec(1.0/10.0), currTime);
		Somatic__WaistCmd *waistd_msg =
				SOMATIC_WAIT_LAST_UNPACK( r, somatic__waist_cmd, //&protobuf_c_system_allocator,
NULL, 4096, &waistd_chan, &abstime );
		aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT || r == ACH_MISSED_FRAME,
				"Ach failure %s on waistd data receive (%s, line %d)\n",
				ach_result_to_string(static_cast<ach_status_t>(r)), __FILE__, __LINE__);
		
		// If message was received, set the torso_direction and current_mode according to the command
		bool debug=0; static int debug_count=0;
		if(r == ACH_OK || r==ACH_MISSED_FRAME) { //debug &= (debug_count++%1000)==0;
			switch(waistd_msg->mode) {
				case SOMATIC__WAIST_MODE__MOVE_FWD:
					if(debug) printf("MOVE_FWD\n");
					real_current_mode = current_mode = false;
					torso_direction = 1.0;
				break;
				case SOMATIC__WAIST_MODE__MOVE_REV:
					if(debug) printf("MOVE_REV\n");
					real_current_mode = current_mode = false;
					torso_direction = -1.0;
				break;
				case SOMATIC__WAIST_MODE__STOP:
					if(debug) printf("STOP\n");
					real_current_mode = current_mode = false;
					torso_direction = 0.0;
				break;
				case SOMATIC__WAIST_MODE__CURRENT_MODE:
					if(debug) printf("CURRENT_MODE\n");
					current_mode = true;
					real_current_mode = false;
				break;
				case SOMATIC__WAIST_MODE__REAL_CURRENT_MODE:
					current_mode = false;
					real_current_mode = true;
					real_current = waistd_msg->data->data[0];
					if(real_current < 0.0) real_current = fmax(real_current, -14.5);	
					else real_current = fmin(real_current, 14.5);		
					if(debug) printf("REAL_CURRENT_MODE: %lf\n", real_current);
				break;
				default:
					if(debug) printf("whatever man! ... \n");
			}	
			somatic__waist_cmd__free_unpacked( waistd_msg, 
//&protobuf_c_system_allocator
NULL );
		}
		// If message was not received, stop the waist motors  
		else if (r == ACH_TIMEOUT) { if(debug) printf("ACH_TIMEOUT\n"); current_mode = false; torso_direction=0.0; } 

	
	  // ============================================================================================
		// Read pcio state channel

		// Get the time to stop waiting by
		clock_gettime( CLOCK_MONOTONIC, &currTime);
		abstime = aa_tm_add(aa_tm_sec2timespec(1.0/30.0), currTime);

		// Get the message
		Somatic__MotorState *pcio_state =
				SOMATIC_WAIT_LAST_UNPACK( r, somatic__motor_state, 
//&protobuf_c_system_allocator, 
NULL, 1024, &pcio_state_chan, &abstime );
		aa_hard_assert(r == ACH_OK || r == ACH_TIMEOUT || r == ACH_MISSED_FRAME,
				"Ach wait failure %s on pcio data receive (%s, line %d)\n",
				ach_result_to_string(static_cast<ach_status_t>(r)),
				__FILE__, __LINE__);

		if (r == ACH_OK) {
			if(!reset)  // We do this because the powercube module reports wrong position after the module
									// is reset for one cycle.
				position = pcio_state->position->data[0]; // Kasemsit: We set torso zero position 
																													 // to be at the joint limit. The value 
																													 // 0.4889 [rad] is where all three robot's 
																													 // axes aligned.
			velocity = pcio_state->velocity->data[0];
			bool sdebug=1;static int sdebug_count=0; sdebug&=sdebug_count++%100==0;
			if(sdebug) std::cout << "position : " << position << std::endl;
			somatic__motor_state__free_unpacked(pcio_state, 
//&protobuf_c_system_allocator
NULL);

			//printf("%f\t%f\n", position, velocity);
		}
		reset = false;

		// ==============================================================================================
		// Controller
	 
		double current, cmd[2];
		// If CURRENT_MODE is not commanded, determine the current control commnd to motors based on 
		// commanded torso_direction
		if((current_mode == false) && (real_current_mode == false)){
			waist_control(current, position_setpoint, velocity_setpoint, position, velocity, torso_direction, dt);
			cmd[0]=current; cmd[1]=-current;
		}
		// If CURRENT_MODE is commanded, make the current control command equal to zero
		else if(real_current_mode) {
			cmd[0]=real_current; cmd[1]=-real_current;
		}
		else if(current_mode) {
			cmd[0]=0.0; cmd[1]=0.0;
		}
		if(debug) printf("cmd: <%lf, %lf>\n", cmd[0], cmd[1]);
		
		// ==============================================================================================
		// Send Motor Commands
    // FIXME: check error codes -ntd

		// If CURRENT_MODE is not commanded and set_point speed is supposed to be zero, halt the motors 
		if((current_mode==false && real_current_mode == false) &&(fabs(velocity_setpoint) < 0.001)) {
      somatic_motor_cmd_set( cmd_msg, SOMATIC__MOTOR_PARAM__MOTOR_HALT, cmd, n_modules, NULL );
      SOMATIC_PACK_SEND( &pcio_cmd_chan, somatic__motor_cmd, cmd_msg );
			position_setpoint = position;
			halted = true;
		}
		// When CURRENT_MODE is commanded or speed is supposed to be non-zero
		else {
			// If motor is halted release it by sending the RESET command
			if(halted) {
      	somatic_motor_cmd_set( cmd_msg, SOMATIC__MOTOR_PARAM__MOTOR_RESET, cmd, n_modules, NULL );
       	SOMATIC_PACK_SEND( &pcio_cmd_chan, somatic__motor_cmd, cmd_msg );
				halted = false;
				reset = true;
			}
			// If motor is already released, send the desired current command to the motor
			else
			{
       	somatic_motor_cmd_set( cmd_msg, SOMATIC__MOTOR_PARAM__MOTOR_CURRENT, cmd, n_modules, NULL );
       	SOMATIC_PACK_SEND( &pcio_cmd_chan, somatic__motor_cmd, cmd_msg );
			}
		}

		// update previous measured time
		usleep(1e4);
		t_prev = t_now;
	}

	// Send a stopping event
	somatic_d_event(&context, SOMATIC__EVENT__PRIORITIES__NOTICE,
	                SOMATIC__EVENT__CODES__PROC_STOPPING, NULL, NULL);
}

/* ******************************************************************************************** */
void destroy () {
	ach_close(&pcio_cmd_chan);
	ach_close(&pcio_state_chan);
	ach_close(&waistd_chan);
	somatic_d_destroy(&context);
}

/*************************************************************************************************/
// Main Function

int main( int argc, char **argv ) {
	argp_parse (&argp, argc, argv, 0, NULL, NULL);
	init();
	run();
	destroy();
	return 0;
}
