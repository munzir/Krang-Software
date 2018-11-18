/* -*- mode: C; c-basic-offset: 2  -*- */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */


/** Author: martin
 * 		    kasemsit
 */

// Needed by ach to set the clock type, as opposed to CLOCK_REALTIME
// which can be discontinuous due to NTP daemon, leap seconds and etc.
#define CLOCK_MONOTONIC

#include <argp.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>

#include <ach.h>
#include <somatic.h>
#include <somatic/util.h>
#include <somatic.pb-c.h>

#include "include/imud.h"

/* ----------- */
/* GLOBAL VARS */
/* ----------- */


/* ---------- */
/* ARGP Junk  */
/* ---------- */
/* Option Vars */
static int opt_verbosity = 0;
static const char *opt_ach_chan = IMU_CHANNEL_NAME;

/* ---------- */
/* ARGP Junk  */
/* ---------- */
static struct argp_option options[] = {
		{
				.name = "verbose",
				.key = 'v',
				.arg = NULL,
				.flags = 0,
				.doc = "Causes verbose output"
		},
		{
				.name = "channel",
				.key = 'c',
				.arg = "channel",
				.flags = 0,
				.doc = "ach channel to use (default \"spacenav-data\")"
		},
		{
				.name = "Create",
				.key = 'C',
				.arg = NULL,
				.flags = 0,
				.doc = "Create channel with specified name (off by default)"
		},
		{
				.name = NULL,
				.key = 0,
				.arg = NULL,
				.flags = 0,
				.doc = NULL
		}
};


/// argp parsing function
static int parse_opt( int key, char *arg, struct argp_state *state);
/// argp program version
const char *argp_program_version = "imutest v0.0.1";
/// argp program arguments documention
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "reads from the imu and pushes out ach messages";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


static int parse_opt( int key, char *arg, struct argp_state *state) {
	(void) state; // ignore unused parameter
	switch(key) {
	case 'v':
		opt_verbosity++;
		break;
	case 'c':
		opt_ach_chan = strdup( arg );
		break;
	case 0:
		break;
	}
	return 0;
}


/* --------------------- */
/* FUNCTION DECLARATIONS */
/* --------------------- */


/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {

	argp_parse (&argp, argc, argv, 0, NULL, NULL);

	// install signal handler
	somatic_sighandler_simple_install();

	// Open IMU channel
	ach_channel_t imu_chan;
	ach_status_t r  = ach_open( &imu_chan, IMU_CHANNEL_NAME , NULL );
	aa_hard_assert(r == ACH_OK,
			"Ach failure %s on opening IMU channel (%s, line %d)\n",
			ach_result_to_string(r), __FILE__, __LINE__);

	if( opt_verbosity ) {
		fprintf(stderr, "\n* IMU *\n");
		fprintf(stderr, "Verbosity:    %d\n", opt_verbosity);
		fprintf(stderr, "channel:      %s\n", opt_ach_chan);
		fprintf(stderr,"-------\n");
	}

	while (!somatic_sig_received) {

		Somatic__Vector *imu_msg =
				SOMATIC_GET_LAST_UNPACK( r, somatic__vector, 
//&protobuf_c_system_allocator, 
NULL, IMU_CHANNEL_SIZE, &imu_chan );

		aa_hard_assert(r == ACH_OK || r == ACH_STALE_FRAMES,
				"Ach failure %s on IMU data receive (%s, line %d)\n",
				ach_result_to_string(r), __FILE__, __LINE__);

		if (r == ACH_OK) {

			if( opt_verbosity ) {
				int i;
				for (i=0; i<6; i++) 		 // x,y,z,dQ,dP,dR
					printf("%f\t", imu_msg->data[i]);
				printf("\n");
			}

			somatic__vector__free_unpacked( imu_msg, 
//&protobuf_c_system_allocator
NULL );
		}
	}

	ach_close(&imu_chan);

	return 0;
}

