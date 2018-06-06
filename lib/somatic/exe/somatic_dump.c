/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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
 * @file somatic_dump.cpp
 * @author Neil Dantam, Can Erdogan
 * @date June 28, 2013
 * @brief This executable prints out the data in the given channel with the proper format.
 */

#include <amino.h>
#include <argp.h>
#include "somatic.h"
#include <unistd.h>

/* ******************************************************************************************** */
// Channel vairables
ach_channel_t sd_chan;
uint8_t *sd_achbuf;
size_t sd_n_achbuf = 1024;
size_t sd_frame_size;
size_t sd_indent = 0;

/* ******************************************************************************************** */
// Option Vars
const char *opt_chan_name = NULL;	///< The ach channel to get data from
const char *opt_msg_type = NULL; 	///< Message type to be used if the message does not have a type

/* ******************************************************************************************** */
static struct argp_option options[] = {
	{ "verbose", 'v', NULL, 0, "Causes verbose output" },
	{ "chan", 'c', "ach channel", 0, "ach channel to send data to" },
	{ "protobuf", 'p', "protobuf-msg", 0, "name of protobuf message [imu]" }, 
	{ NULL, 0, NULL, 0, NULL }
};

/* ******************************************************************************************** */
static int parse_opt( int key, char *arg, struct argp_state *state) {
	(void) state; // ignore unused parameter
	switch(key) {
		case 'v': somatic_opt_verbosity++; break;
		case 'c': opt_chan_name = strdup( arg ); break;
		case 'p': opt_msg_type = strdup( arg ); break;
		case 0: break;
	}
	return 0;
}

/// argp program version
const char *argp_program_version = "somatic_dump 0.0";

/// argp program doc line
static char doc[] = "dump somatic msg to stdout";

/// argp object
static struct argp argp = {options, parse_opt, NULL, doc, NULL, NULL, NULL };

/* ******************************************************************************************** */
/// Reads the data from the ach channel
void read_ach() {
	ach_status_t r = ach_get( &sd_chan, sd_achbuf, sd_n_achbuf, &sd_frame_size, NULL, ACH_O_WAIT);
	if( ACH_OVERFLOW == r ) {
		sd_n_achbuf = AA_MAX( sd_frame_size, 2*sd_n_achbuf );
		free( sd_achbuf );
		sd_achbuf = AA_NEW_AR(uint8_t,  sd_n_achbuf );
		read_ach();
		return;
	}
	aa_hard_assert( ACH_OK == r || ACH_MISSED_FRAME == r, "Error reading frame: %s\n",
		ach_result_to_string( r ) );
}

/* ******************************************************************************************** */
/// Initializes the buffer to read messages into, creates the ach channel and sets the interrupt
/// handler
void init() {

	// Set up the buffer
	sd_achbuf = AA_NEW_AR(uint8_t,  sd_n_achbuf );

	// Open the given channel 
	int r = ach_open( &sd_chan, opt_chan_name, NULL );
	aa_hard_assert( ACH_OK == r, "Couldn't open channel %s\n", opt_chan_name );
	r = ach_flush( &sd_chan );
	aa_hard_assert( ACH_OK == r, "Couldn't flush channel\n");

	// Set the interrupt handler
	somatic_sighandler_simple_install();
}

/* ******************************************************************************************** */
void indent() {
	char buf[2*sd_indent+1];
	memset( buf, ' ', 2*sd_indent );
	buf[2*sd_indent] = '\0';
	printf("%s", buf);

}

/* ******************************************************************************************** */
/// Prints a double vector
void dump_vector (Somatic__Vector *pb, const char *fmt) {
	for( size_t i = 0; pb && pb->data && i < pb->n_data; i++ ) {
		printf(fmt, pb->data[i]);
	}
}

/* ******************************************************************************************** */
/// Prints an integer vector
void dump_ivector (Somatic__Ivector *pb, const char *fmt) {
	for( size_t i = 0; i < pb->n_data; i++ ) {
		printf(fmt, pb->data[i]);
	}
}

/* ******************************************************************************************** */
/// Prints a transform
void dump_transform( Somatic__Transform *pb ) {

	// Print the translation
	indent();
	printf("[Transform] ");
	if( pb->translation ) {
		printf("\t%6.3f\t%6.3f\t%6.3f", pb->translation->data[0], pb->translation->data[1],
			pb->translation->data[2]);
	} 
	else printf("\t\t\t");

	// Print the rotation
	printf("\t|");
	if( pb->rotation ) {
		printf("\t%6.3f\t%6.3f\t%6.3f\t%6.3f",
			   pb->rotation->data[0],
			   pb->rotation->data[1],
			   pb->rotation->data[2],
			   pb->rotation->data[3] );
	}
	printf("\n");
}

/* ******************************************************************************************** */
/// Prints the time specification
void dump_timespec( Somatic__Timespec *pb, const char *name ) {
	indent();
	printf("[%s : Timespec]\t%lus\t%09dns\n",
		   name, pb->sec, pb->has_nsec ? pb->nsec : 0 );
}

/* ******************************************************************************************** */
/// Prints the metadata
void dump_metadata( Somatic__Metadata *pb ) {

	// Print the time and the duration 
	indent();
	printf("[Metadata] \n");
	sd_indent++;
	if(pb->time) dump_timespec(pb->time, "time");
	if(pb->until) dump_timespec(pb->until, "until");

	// Print the type if one exists
	if(pb->has_type) {
		indent();
		const char *c = "unknown";
		switch(pb->type) {
			case SOMATIC__MSG_TYPE__FORCE_MOMENT: c = "ForceMoment"; break;
			case SOMATIC__MSG_TYPE__MOTOR_CMD: c = "MotorCmd"; break;
			case SOMATIC__MSG_TYPE__MOTOR_STATE: c = "MotorState"; break;
			case SOMATIC__MSG_TYPE__TRANSFORM: c = "Transform"; break;
			case SOMATIC__MSG_TYPE__MULTI_TRANSFORM: c = "MultiTransform"; break;
			case SOMATIC__MSG_TYPE__POINT_CLOUD: c = "PointCloud"; break;
			case SOMATIC__MSG_TYPE__JOYSTICK: c = "Joystick"; break;
			case SOMATIC__MSG_TYPE__LIBERTY: c = "Liberty"; break;
			case SOMATIC__MSG_TYPE__TOUCH: c = "Touch"; break;
			case SOMATIC__MSG_TYPE__MICROPHONE: c = "Microphone"; break;
			case SOMATIC__MSG_TYPE__BATTERY: c = "Battery"; break;
			case SOMATIC__MSG_TYPE__WAIST_CMD: c = "WaistCmd"; break;
			case SOMATIC__MSG_TYPE__VISUALIZE_DATA: c = "VisData"; break;
		}
		printf("[type] %s\n", c);
	}
	sd_indent--;
}

/* ******************************************************************************************** */
/// Prints multiple transforms
void dump_multi_transform( Somatic__MultiTransform *pb ) {

	// Print each transform
	indent();
	printf("[MultiTransform]\n");
	sd_indent++;
	if( pb->tf ) {
		for( size_t i = 0; i < pb->n_tf; i++ ) dump_transform(pb->tf[i]);
	}

	// Print the metadata
	if( pb->meta ) dump_metadata( pb->meta );
	sd_indent--;
}

/* ******************************************************************************************** */
/// Prints the force/torque data
void dump_force_moment( Somatic__ForceMoment *pb ) {

	// Print the title and the force values
	indent();
	printf("[ForceMoment]\n");
	sd_indent++;
	indent();
	printf("[force]");
	if(pb->force) dump_vector(pb->force, "\t%6.3f");

	// Print the moments 
	printf("\n");
	indent();
	printf("[moment]");
	if(pb->moment) dump_vector(pb->moment, "\t%6.3f");
	printf("\n");

	// Print the metadata
	if(pb->meta) dump_metadata( pb->meta );
	sd_indent--;

}

/* ******************************************************************************************** */
/// Prints the joystick data
void dump_joystick( Somatic__Joystick *pb ) {

	// Print the title and the axes values
	indent();
	printf("[Joystick]\n");
	sd_indent++;
	indent();
	printf("[axes]");
	dump_vector(pb->axes, "\t%6.3f");

	// Print the buttons
	printf("\n");
	indent();
	printf("[buttons]\t");
	dump_ivector(pb->buttons, "%d:");
	printf("\n");

	// Print the metadata
	if(pb->meta) dump_metadata( pb->meta );
	sd_indent--;
}

/* ******************************************************************************************** */
/// Prints the liberty data
void dump_liberty( Somatic__Liberty *pb ) {
  indent();
  printf("[Liberty]\n");
  dump_vector(pb->sensor1, "\t%6.3f"); printf("\n");
  dump_vector(pb->sensor2, "\t%6.3f"); printf("\n");
  dump_vector(pb->sensor3, "\t%6.3f"); printf("\n");
  dump_vector(pb->sensor4, "\t%6.3f"); printf("\n");
  dump_vector(pb->sensor5, "\t%6.3f"); printf("\n");
  dump_vector(pb->sensor6, "\t%6.3f"); printf("\n");
  dump_vector(pb->sensor7, "\t%6.3f"); printf("\n");
  dump_vector(pb->sensor8, "\t%6.3f"); printf("\n");
  
  // Print the metadata
  if(pb->meta) dump_metadata( pb->meta );
  //sd_indent--;
}

/* ******************************************************************************************** */
/// Print battery data
void dump_battery( Somatic__Battery *pb ) {

	// Print the title and the voltage values
	indent();
	printf("[Battery]\n");
	sd_indent++;
	indent();
	printf("[volt]");
	dump_vector(pb->voltage, "\t%5.3f");

	// Print the temperatures
	printf("\n");
	indent();
	printf("[temp]");
	dump_vector(pb->temp, "\t%5.2f");
	printf("\n");

	// Print the metadata
	if(pb->meta) dump_metadata( pb->meta );
	sd_indent--;
}

/* ******************************************************************************************** */
/// Print the motor state with position, velocity and current values
void dump_motor_state( Somatic__MotorState *pb ) {

	// Print the title and the position values
	indent();
	printf("[MotorState]\n");
	sd_indent++;
	if( pb->position ) {
		indent();
		printf("[position]");
		dump_vector(pb->position, "\t%6.3f");
		printf("\n");
	}

	// Print the velocities
	if( pb->velocity ) {
		indent();
		printf("[velocity]");
		dump_vector(pb->velocity, "\t%6.3f");
		printf("\n");
	}

	// Print the currents
	if( pb->current ) {
		indent();
		printf("[current]");
		dump_vector(pb->current, "\t%6.3f");
		printf("\n");
	}

	// Print the motor status
	if( pb->has_status ) {
		indent();
		printf("[status]");
		if( pb->status == SOMATIC__MOTOR_STATUS__MOTOR_OK ) printf("\tMOTOR_OK\n");
		else if( pb->status == SOMATIC__MOTOR_STATUS__MOTOR_FAIL ) printf("\tMOTOR_FAIL\n");
		else if( pb->status == SOMATIC__MOTOR_STATUS__MOTOR_COMM_FAIL ) printf("\tMOTOR_COMM_FAIL\n");
		else if( pb->status == SOMATIC__MOTOR_STATUS__MOTOR_HW_FAIL ) printf("\tMOTOR_HW_FAIL\n");
		else printf("\tUNKNOWN\n");
	}

	// Print the metadata
	if( pb->meta ) dump_metadata( pb->meta );
	sd_indent--;
}
 
/* ******************************************************************************************** */
/// Print the motor command
void dump_motor_cmd( Somatic__MotorCmd *pb ) {

	// Print the title and the parameter name
	indent();
	printf("[MotorCmd]\n");
	sd_indent++;
	indent();
	switch ( pb->param ) {
		case SOMATIC__MOTOR_PARAM__MOTOR_CURRENT: printf("[param]\tCURRENT\n"); break;
		case SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY: printf("[param]\tVELOCITY\n"); break;
		case SOMATIC__MOTOR_PARAM__MOTOR_POSITION: printf("[param]\tPOSITION\n"); break;
		case SOMATIC__MOTOR_PARAM__MOTOR_HALT: printf("[param]\tHALT\n"); break;
		case SOMATIC__MOTOR_PARAM__MOTOR_RESET: printf("[param]\tRESET\n"); break;
		case SOMATIC__MOTOR_PARAM__MOTOR_DIGITAL_OUT: printf("[param]\tDIGITAL OUT\n"); break;
	}

	// Print the given values
	if ( pb->values ) {
		indent();
		printf("[values]");
		dump_vector(pb->values, "\t%6.3f");
		printf("\n");
	}
	sd_indent--;
}

/* ******************************************************************************************** */
/// Dumps the waist command values which are 4 enumerations: go fwd, go rev, stop, current
void dump_waist_cmd (Somatic__WaistCmd* pb) {

	// Print the title and the parameter name
	indent();
	printf("[WaistCmd]\n");
	sd_indent++;
	indent();
	switch ( pb->mode ) {
		case SOMATIC__WAIST_MODE__MOVE_FWD: printf("[mode]\tMove forward\n"); break;
		case SOMATIC__WAIST_MODE__MOVE_REV: printf("[mode]\tMove reverse\n"); break;
		case SOMATIC__WAIST_MODE__STOP: printf("[mode]\tStop\n"); break;
		case SOMATIC__WAIST_MODE__CURRENT_MODE: printf("[mode]\tCurrent mode\n"); break;
		default: printf("[mode]\tUNKNOWN!\n");
	}

	// Print the metadata
	if(pb->meta) dump_metadata( pb->meta );
	sd_indent--;
}

/* ******************************************************************************************** */
/// Dumps the imu value after processing it with ssdmu library
void dump_imu (Somatic__Vector* vec) {

	// Compute the pitch from the value (as done in ssdmu_pitch function)
	static const double csr = -.7853981634;
	double newX = vec->data[0]*cos(csr) - vec->data[1]*sin(csr);
	double imu = atan2(newX, vec->data[2]);

	// Compute the pitch velocity from the value (as done in ssdmu_d_pitch function)
	double imu_d = vec->data[3] * sin(csr) + vec->data[4] * cos(csr);

	// Print it
	indent();
	printf("[Imu]\n");
	sd_indent++;
	printf("\tpos: %6.3lf (rad) \t %6.3lf (deg)\n", imu, (imu / M_PI) * 180.0);
	printf("\tvel: %6.3lf (rad) \t %6.3lf (deg)\n", imu_d, (imu_d / M_PI) * 180.0);
	sd_indent--;
}

/* ******************************************************************************************** */
/// Dumps data about location of a cinder block wrt robot frame
void dump_cinder(Somatic__Cinder* vd) {
	indent();
	printf("[Cinder]\n");
	sd_indent++;
	indent();
	printf("[Hole]:\t");
	dump_vector(vd->hole, "\t%6.3f");
	printf("\n");
	indent();
	printf("[Normal]:\t");
	dump_vector(vd->normal, "\t%6.3f");
	printf("\n");
	sd_indent--;
}

/* ******************************************************************************************** */
/// Dumps visualization data as a list of vectors
void dump_visualize_data(Somatic__VisualizeData* vd) {
	indent();
	printf("[VisData]\n");
	sd_indent++;
	indent();
	printf("[Message]\t%s\n", vd->msg);
	indent();
	printf("[Bools]\t");
	dump_ivector(vd->bools, "%d:");
	printf("\n");
	indent();
	printf("[Vectors]:\t%zu\n", vd->n_vecs);
	sd_indent++;
	for(size_t vec = 0; vec < vd->n_vecs; vec++) {
		indent();
		printf("[Vec %zu]", vec);
		dump_vector(vd->vecs[vec], "\t%6.3f");
		printf("\n");
	}
	sd_indent--;
	sd_indent--;
}

/* ******************************************************************************************** */
/// Calls the dump function for a given message type
#define UNPACK_DUMP( type, alloc, buf, size ) \
	dump_ ## type( somatic__ ## type ## __unpack( alloc, size, buf ) );

/* ******************************************************************************************** */
/// The main thread that gets a message, casts it to the correct type and prints it
void run() {

	somatic_pbregalloc_t alloc;
	somatic_pbregalloc_init(&alloc, 4096);

	while( !somatic_sig_received ) {

		// Read the message into a buffer
		read_ach();

		// Get the base message from the buffer
		Somatic__BaseMsg *base =
			somatic__base_msg__unpack( &alloc, sd_frame_size, sd_achbuf);

		// Convert the message to the proper type if it has one
		if( base->meta && base->meta->has_type ) {
			switch ( base->meta->type ) {
				case SOMATIC__MSG_TYPE__FORCE_MOMENT:
					UNPACK_DUMP( force_moment, &alloc, sd_achbuf, sd_frame_size ); break;
				case SOMATIC__MSG_TYPE__MULTI_TRANSFORM:
					UNPACK_DUMP( multi_transform, &alloc, sd_achbuf, sd_frame_size ); break;
				case SOMATIC__MSG_TYPE__JOYSTICK:
					UNPACK_DUMP( joystick, &alloc, sd_achbuf, sd_frame_size ); break;
				case SOMATIC__MSG_TYPE__LIBERTY:
					UNPACK_DUMP( liberty, &alloc, sd_achbuf, sd_frame_size ); break;
				case SOMATIC__MSG_TYPE__MOTOR_CMD:
					UNPACK_DUMP( motor_cmd, &alloc, sd_achbuf, sd_frame_size ); break;
				case SOMATIC__MSG_TYPE__MOTOR_STATE:
					UNPACK_DUMP( motor_state, &alloc, sd_achbuf, sd_frame_size ); break;
				case SOMATIC__MSG_TYPE__BATTERY:
					UNPACK_DUMP( battery, &alloc, sd_achbuf, sd_frame_size ); break;
				case SOMATIC__MSG_TYPE__WAIST_CMD:
					UNPACK_DUMP( waist_cmd, &alloc, sd_achbuf, sd_frame_size ); break;
				case SOMATIC__MSG_TYPE__CINDER:
					UNPACK_DUMP( cinder, &alloc, sd_achbuf, sd_frame_size ); break;
				case SOMATIC__MSG_TYPE__VISUALIZE_DATA:
					UNPACK_DUMP( visualize_data, &alloc, sd_achbuf, sd_frame_size ); break;
				default: printf("Unknown Message: %d\n",base->meta->type);
			}
		} 

		// If not, if a type is provided use it (such as vector for imu values)
		else if(opt_msg_type != NULL) {

			// If it is a vector, dump it as a vector
			if(strcmp(opt_msg_type, "imu") == 0.0) {
				dump_imu(somatic__vector__unpack(&alloc, sd_frame_size, sd_achbuf));
			}
		
			// If it is a vector, dump it as a vector
			if(strcmp(opt_msg_type, "vec") == 0.0) {
				Somatic__Vector* vec = somatic__vector__unpack(&alloc, sd_frame_size, sd_achbuf);
				printf("[%s]:\t", opt_chan_name);
				dump_vector(vec, "\t%6.3f");
				printf("\n");
			}
		
		}

		// If no information is available, just give up
		else 
			printf("Unknown Message, no type info\n");
		
		// Free the message memory
		assert( 0 == sd_indent );
		somatic_pbregalloc_release(&alloc);

		// Sleep a bit
		usleep(1e4);
	}

	// Free the daemon memory (?)
	somatic_pbregalloc_destroy(&alloc);
}

/* ******************************************************************************************** */
/// Closes the ach channel
void destroy() {
	ach_close(&sd_chan);
}

/* ******************************************************************************************** */
/// The main function
int main( int argc, char **argv ) {

	// Parse options
	argp_parse (&argp, argc, argv, 0, NULL, NULL);
	somatic_verbprintf_prefix="somatic_dump";
	aa_hard_assert( NULL != opt_chan_name, "Must set channel name\n");

	// Initialize the ach channel and the buffer
	init();

	// Print 
	somatic_verbprintf( 1, "Channel %s\n", opt_chan_name );
	somatic_verbprintf( 1, "Protobuf %s\n", opt_msg_type );

	// Continuously check for new messages
	run();

	// Clean up
	destroy();
	return 0;
}
/* ******************************************************************************************** */
