/**
 * @file imud.c
 * @author Martin Levihn, Kasemsit Teeyapan, Can Erdogan
 * @date Jan 12, 2012
 * @brief Writes the bms values to a log file.
 */
 
// Needed by ach to set the clock type, as opposed to CLOCK_REALTIME
// which can be discontinuous due to NTP daemon, leap seconds and etc.
#define CLOCK_MONOTONIC

#include <argp.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#include <ach.h>
#include <somatic.h>
#include <somatic/daemon.h>

#include "include/imud.h"

typedef struct {
    somatic_d_t d;
    somatic_d_opts_t d_opts;
} cx_t;

/* Option Vars */
static int opt_can_id = 0;        // CAN bus id
static int opt_verbosity = 0;
static int opt_create = 0;
static int opt_freq = 0;
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
				.doc = "ach channel to use (default \"imu-data\")"
		},
		{
				.name = "bus",
				.key = 'b',
				.arg = "CAN_bus",
				.flags = 0,
				.doc = "define a CAN bus for an IMU"
		},
		{
				.name = "Create",
				.key = 'C',
				.arg = NULL,
				.flags = 0,
				.doc = "Create channel with specified name (off by default)"
		},
		{
				.name = "frequency",
				.key = 'f',
				.arg = NULL,
				.flags = 0,
				.doc = "Displays how fast the daemon is running"
		},
                SOMATIC_D_ARGP_OPTS,
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
const char *argp_program_version = "imud-0.0.1";
/// argp program arguments documentation
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "reads from an imu and pushes out ach messages";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


static int parse_opt( int key, char *arg, struct argp_state *state) {
	cx_t *cx = (cx_t*)state->input;
	switch(key) {
	case 'b':
		opt_can_id = atoi(arg);
		break;
	case 'v':
		opt_verbosity++;
		break;
	case 'c':
		opt_ach_chan = strdup( arg );
		break;
	case 'C':
		opt_create = 1;
		break;
	case 'f':
		opt_freq++;
	case 0:
		break;
	}
        somatic_d_argp_parse( key, arg, &cx->d_opts);
	return 0;
}

/* ------------- */
/* Function Defs */
/* ------------- */

/**
 * Block, waiting for a mouse event
 */
void imu_read_to_msg(Somatic__Vector *msg, ssdmu_t *dmu)
{
	ssdmu_sample_t sample;
	int status = ssdmu_read_sample(dmu->handle, &sample);
	if (status != 0)
		return; // Didn't get good control values, loop

	msg->data[0]=sample.x;
	msg->data[1]=sample.y;
	msg->data[2]=sample.z;
	msg->data[3]=sample.dP;
	msg->data[4]=sample.dQ;
	msg->data[5]=sample.dR;

}


/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {
	static cx_t cx;
	memset(&cx,0,sizeof(cx));
	cx.d_opts.sched_rt = SOMATIC_D_SCHED_MOTOR;
	cx.d_opts.ident = "imud";

	argp_parse (&argp, argc, argv, 0, NULL, &cx);
	somatic_d_init(&cx.d, &cx.d_opts);

	// open IMU
	int status;
	ssdmu_t *dmu;
	status = ssdmu_open(opt_can_id, &dmu);
	if (status != 0) {
		perror("ssdmu_open");
		exit(-1);
	}

	// Create channel if requested
	if (opt_create == 1) {
		int i;
		{
			ach_create_attr_t attr;
			ach_create_attr_init(&attr);
			i = ach_create( (char*)opt_ach_chan, 10, IMU_CHANNEL_SIZE, &attr );
		}
		aa_hard_assert(i == ACH_OK,
				"ERROR: Ach failure %s on creatinging IMU channel (%s, line %d)\n",
				ach_result_to_string((ach_status_t)i), __FILE__, __LINE__);
	}


	int r;
	// Open the channel
	ach_channel_t imu_chan;
	somatic_d_channel_open(&cx.d, &imu_chan, IMU_CHANNEL_NAME, NULL );


	// Allocate IMU message
	Somatic__Vector *imu_msg = somatic_vector_alloc(6);

	if( opt_verbosity ) {
		fprintf(stderr, "\n* IMU *\n");
		fprintf(stderr, "Verbosity:      %d\n", opt_verbosity);
		fprintf(stderr, "imu canbus id:  %d\n", opt_can_id);
		fprintf(stderr, "channel:        %s\n", opt_ach_chan);
		fprintf(stderr, "message size:   %d\n", somatic__vector__get_packed_size(imu_msg) );
		fprintf(stderr,"-------\n");
	}
	canIoctl(dmu->handle, NTCAN_IOCTL_FLUSH_RX_FIFO, NULL);

	struct timespec t_prev, t_now;
	int count = 0;

	t_prev = aa_tm_now();
	t_now = aa_tm_now();

	while (!somatic_sig_received) {

		// Read IMU and save to message
		imu_read_to_msg(imu_msg, dmu);

		// Publish to the IMU channel
		r = SOMATIC_PACK_SEND( &imu_chan, somatic__vector, imu_msg );
		aa_hard_assert( r == ACH_OK, "Couldn't send IMU message\n");

		if( opt_verbosity ) {
			int i;
			for (i=0; i<6; i++) 		 // x,y,z,dQ,dP,dR
				printf("%f\t", imu_msg->data[i]);
			printf("\n");
		}

		if( opt_freq ) {
			if(count % 2000 == 0) {
				t_now = aa_tm_now();												// get current time
				double dt = (double)aa_tm_timespec2sec(aa_tm_sub(t_now, t_prev));	// get time difference
				double freq = count/dt;
				fprintf(stderr, "IMUD Frequency: %.2lf Hz\r", freq);
				t_prev = aa_tm_now();
				count = 0;
			}
			count++;
		}
	}

	// Cleanup:
	ach_close(&imu_chan);
	free(imu_msg->data);
	free(imu_msg);
	free(dmu);
	somatic_d_destroy(&cx.d);

	return 0;
}


