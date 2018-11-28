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

/**
 * @file amciod.c
 * @author Jon Olson (main), Kasemsit Teeyapan, Evan Seguin, Jon Scholz, Neil Dantam
 * @author Munzir Zafar => virtual output manipulation
 * @date Feb 14, 2013
 * @brief amciod is an ACH front-end daemon to communicate with two
 * AMC drives over CAN network.  It reads motor command messages from
 * an ACH channel and publishes the current states of the motors to
 * another ACH channel.
 * NOTE: The daemon reads state message in [rad, rad/s], and writes
 * command message in [Ampere].
 */

#include <argp.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#include <pthread.h>
#include <somatic.h>
#include <somatic/daemon.h>
#include <ach.h>

#include <somatic/util.h>
#include <somatic.pb-c.h>
#include <unistd.h>
#include <time.h>
#include <somatic/motor.h>

#include <math.h>

#include "amccan.h"
#include "amcdrive.h"

// Default channel constants
#define AMCIOD_CMD_CHANNEL_NAME "amc-cmd"
#define AMCIOD_STATE_CHANNEL_NAME "amc-state"

#define MAX_CURRENT 50        // Max motor current (Amps)
#define ENCODER_COUNT 4000
#define GEAR_RATIO (15.0/1.0)

#define RAD_TO_COUNT(rad) \
  ( (rad) * (ENCODER_COUNT * GEAR_RATIO ) / (2 * M_PI) )

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

/* ----------- */
/* PROTOTYPES  */
/* ----------- */

typedef struct {
    somatic_d_t d;
    somatic_d_opts_t d_opts;
    Somatic__MotorState *state_msg;
    struct timespec time;

    struct timespec wait_time;
    ach_channel_t cmd_chan;
    ach_channel_t state_chan;

    servo_vars_t *servos;
    size_t n_servos;
} cx_t;

static void
amciod_execute( cx_t *cx, servo_vars_t *servos,
                Somatic__MotorCmd *msg );

static int amciod_start(cx_t *cx, servo_vars_t *servos, size_t n);
static void amciod_run(cx_t *cx, servo_vars_t *servos);

static void *amciod_update_thfun( void *cx );

/* ---------- */
/* ARGP Junk  */
/* ---------- */

//TODO: replace these with vars from parsing args
static const char *opt_cmd_chan = AMCIOD_CMD_CHANNEL_NAME;
static const char *opt_state_chan = AMCIOD_STATE_CHANNEL_NAME;

static int opt_verbosity = 0;
static double opt_frequency = 30.0;   // refresh at 30 hz
static size_t n_buses = 0;            // number of bus
static size_t n_modules = 0;          // number of amc modules
static int32_t opt_bus_id = 0;        // amc bus id
static uint8_t opt_mod_id[2];         // amc module id
static int opt_query = 0;             // get module info
static int opt_reset = 0;             // reset drives
static uint16_t vout = 0x0000;				// represents the 16 1-bit values of the virtual outputs
static struct argp_option options[] = {
    {
        .name = "cmd-chan",
        .key = 'c',
        .arg = "amciod-cmd",
        .flags = 0,
        .doc = "ACH channel to send amcdrive commands to"
    },
    {
        .name = "state-chan",
        .key = 's',
        .arg = "amciod-state",
        .flags = 0,
        .doc = "ACH channel to listen for commands on"
    },
    {
        .name = "verbose",
        .key = 'v',
        .arg = NULL,
        .flags = 0,
        .doc = "Causes verbose output"
    },
    {
        .name = "Create",
        .key = 'C',
        .arg = NULL,
        .flags = 0,
        .doc = "Create channels with specified names (off by default)"
    },
    {
        .name = "frequency",
        .key = 'f',
        .arg = "freq",
        .flags = 0,
        .doc = "Refresh rate on state channel when no commands are received"
    },
    {
        .name = "module",
        .key = 'm',
        .arg = "module_id",
        .flags = 0,
        .doc = "Define a module ID for a motor index (e.g. 0x20)"
    },
    {
        .name = "bus",
        .key = 'b',
        .arg = "CAN_bus",
        .flags = 0,
        .doc = "Define a CAN bus for a module"
    },
    /*{
        .name = "query",
        .key = 'Q',
        .arg = NULL,
        .flags = 0,
        .doc = "get info about the module"
        },*/
    {
        .name = "reset",
        .key = 'R',
        .arg = NULL,
        .flags = 0,
        .doc = "reset drives"
    },
		{
				.name = "override",
				.key = 'z',
				.arg = NULL,
				.flags = 0,
				.doc = "Debug thing. To check whether virtual output is controlled or not"
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
const char *argp_program_version = "amciod 0.2";
/// argp program arguments documention
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "reads somatic messages and sends amcdrive motor commands";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };

/* ********************************************************************************************* */
/// Mutex for collaboration between the read and write threads for the virtual output manipulation
pthread_mutex_t handleMutex = PTHREAD_MUTEX_INITIALIZER;

/* ********************************************************************************************* */
/**
 * \brief Generate the amcdrive calls requested by the specified motor
 * command message, and update the state
 */
int main(int argc, char *argv[]) {

    static cx_t cx;
    memset(&cx,0,sizeof(cx));
    cx.d_opts.ident = "amciod";
    cx.d_opts.sched_rt = SOMATIC_D_SCHED_MOTOR;

    // ------------- INIT --------------------

    /// Parse command line args
    argp_parse(&argp, argc, argv, 0, NULL, &cx);

    /// AMC drive init
    //servo_vars_t servos[2];
    cx.n_servos = n_modules;
    cx.servos = AA_NEW0_AR(servo_vars_t,cx.n_servos);
    for( size_t i = 0; i < cx.n_servos; i ++ ) {
        cx.servos[i].gear_ratio = GEAR_RATIO;
        cx.servos[i].encoder_count = ENCODER_COUNT;
    }

    //int r = amciod_open(cx.servos);
    NTCAN_RESULT r = amcdrive_open_drives( opt_bus_id, opt_mod_id, n_modules,
                                           cx.servos);

    somatic_d_require( &cx.d, NTCAN_SUCCESS == r,
                       "AMC open failed: %i\n", canResultString(r) );

    // ------------- MODE ----------------------
    if ( opt_query ) {
    } else if ( opt_reset ) {
        printf("reset\n");
        NTCAN_RESULT status = amcdrive_reset_drives(cx.servos, n_modules);
        if( NTCAN_SUCCESS != status ) {
            fprintf(stderr, "CAN error: %s\n", canResultString(status));
            exit(EXIT_FAILURE);
        }
    } else {

        // init daemon context
        somatic_d_init(&cx.d, &cx.d_opts);

        /// Ach channels for amciod
        //ach_channel_t cmd_chan, state_chan;
        somatic_d_channel_open(&cx.d, &cx.cmd_chan, opt_cmd_chan, NULL );
        somatic_d_channel_open(&cx.d, &cx.state_chan, opt_state_chan, NULL );
        ach_flush(&cx.cmd_chan);

        /// Set the wait time based on specificed frequency
        cx.wait_time = (struct timespec){
            .tv_sec = 0,
            .tv_nsec = (long int)((1.0/opt_frequency) * 1e9)
        };

        // init message
        cx.state_msg = somatic_motor_state_alloc();
        cx.state_msg->has_status = 1;
        cx.state_msg->status = SOMATIC__MOTOR_STATUS__MOTOR_OK;
        {
            double x[n_modules];
            memset(x,0,sizeof(x[0])*n_modules);
            somatic_motor_state_set_position( cx.state_msg, x, n_modules );
            somatic_motor_state_set_velocity( cx.state_msg, x, n_modules );
            somatic_motor_state_set_current( cx.state_msg, x, n_modules );
        }

        if (opt_verbosity) {
            fprintf(stderr, "\n* AMCIOD *\n");
            fprintf(stderr, "Verbosity:    %d\n", opt_verbosity);
            fprintf(stderr, "command channel:      %s\n", opt_cmd_chan);
            fprintf(stderr, "state channel:      %s\n", opt_state_chan);
            fprintf(stderr, "n_module = %d\n", n_modules);
            fprintf(stderr, "module id = 0x%x  0x%x\n",
                    cx.servos[0].canopen_id, cx.servos[1].canopen_id);
            fprintf(stderr, "-------\n");
        }

        // start drives
        amciod_start(&cx, cx.servos, n_modules);
        // ------------- RUN --------------------
        amciod_run( &cx, cx.servos );

        // ------------ DESTROY --------------

        /// Stop motors
        for( size_t i = 0; i < n_modules; i++ ) {
            r = amcdrive_set_current(&cx.servos[i], 0.0);
            aa_hard_assert(r == NTCAN_SUCCESS, "amcdrive_set_current error: %i\n", r);
            r = amcdrive_stop_drive(&cx.servos[i]);
            aa_hard_assert(r == NTCAN_SUCCESS, "amcdrive_stop error: %i\n", r);
        }

        //

        /// Close channels
        ach_close(&cx.cmd_chan);
        ach_close(&cx.state_chan);

        somatic_d_destroy(&cx.d);
    }

    return 0;
}

/* ********************************************************************************************* */
static int parse_opt(int key, char *arg, struct argp_state *state) {
    cx_t *cx = (cx_t*)state->input;

    if (n_buses > 1) {
        fprintf(stderr, "ERROR: Accept only one -b parameters.\n");
        exit(1);
    }
    if (n_modules > 2) {
        fprintf(stderr, "ERROR: Accept only two -m parameters.\n");
        exit(1);
    }

    long int mod;
    switch (key) {

    case 'c':
        opt_cmd_chan = strdup(arg);
        break;
    case 's':
        opt_state_chan = strdup(arg);
        break;
    case 'v':
        opt_verbosity++;
				printf("verb: %d\n", opt_verbosity);
        break;
    case 'f':
        opt_frequency = atof(arg);
        break;
    case 'Q':
        opt_query++;
        break;
    case 'R':
        opt_reset++;
        break;
    case 'b':
        opt_bus_id = atoi(arg);      // accept only one bus id
        n_buses++;
        break;
    case 'm':
        mod = strtol(arg, NULL, 16); // Add an AMC module id to the list
        assert(mod < UINT8_MAX);
        opt_mod_id[n_modules++] = (uint8_t) mod;
        break;

		// Sets all the virtual outputs to 1
		case 'z':
				vout = 0xFFFF;
				break;
    }
    somatic_d_argp_parse( key, arg, &cx->d_opts );



    return 0;
}

/* ********************************************************************************************* */
static int amciod_start(cx_t *cx, servo_vars_t *servos, size_t n){
    int r;
    unsigned flags =
        REQUEST_TPDO_VELOCITY | REQUEST_TPDO_POSITION |
        ENABLE_RPDO_CURRENT | REQUEST_TPDO_STATUSWORD;

    // fetch drive vars
    for( size_t i = 0; i < n; i++ ) {
        r = amcdrive_init_drive(&servos[i], flags, 1000 );
        somatic_d_require( &cx->d, NTCAN_SUCCESS == r,
                           "couldn't init drive: %s", canResultString(r) );
        r = amcdrive_set_current(&servos[i], 0.0);
        somatic_d_require( &cx->d, NTCAN_SUCCESS == r,
                           "couldn't set current: %s", canResultString(r) );
    }

    // start drives for sure
    // sometimes it takes a couple tries to actually start these damn things
    int drives_ok = 1;
    int cnt = 0;
    do {
        // drives take a moment to settle, sleep a bit
        usleep(1e6 / 10);
        // update state
        drives_ok = 1;
        r = amcdrive_update_drives(servos, n);
        somatic_d_require( &cx->d, NTCAN_SUCCESS == r, "amcdrive_update error: %i\n", r);
        // check state
        for( size_t i = 0; drives_ok && i < n; i++ ) {
						amccan_state_t currState = amccan_decode_state(servos[i].status);
						printf("Current state: '%s'\n", amccan_state_string(currState));
            if( AMCCAN_STATE_ON_OP_EN != currState) {
                drives_ok = 0;
                // try to reset again
                r = amcdrive_start(servos[i].handle, servos[i].canopen_id);
                somatic_d_require( &cx->d, NTCAN_SUCCESS == r,
                                   "amcdrive_update error: %i\n", r);
            }
            // reset the prev drive status
            servos[i].prev_status = servos[i].status;
        }

				printf("cnt: %d\n", cnt);
        cnt++;
        somatic_d_require( &cx->d, cnt < 100,
                           "couldn't enable drives" );
    } while(!drives_ok);
    // set current
    for( size_t i = 0; i < n; i++ ) {
        r = amcdrive_set_current(&servos[i], 0.0);
        somatic_d_require( &cx->d, NTCAN_SUCCESS == r,
                           "couldn't set current: %s", canResultString(r) );
    }

		// Send the initial virtual output values
		uint8_t rcmd;
		if(	amccan_dl_virtual_out_ctrl(servos[1].handle, &rcmd, servos[1].canopen_id, vout ) != NTCAN_SUCCESS ) {
			fprintf(stderr,"Couldn't set the virtual output\n");
		}
		else { fprintf(stderr,"main: virtual output set to 0x%x\n", vout); }

    // FIXME: WTF? -ntd
    servos[1].current_sign = -1;

		// mutex for multi-threading
		servos[0].digitalOutCmdReceived=0;
		servos[1].digitalOutCmdReceived=0;

    return 0;
}

/* ********************************************************************************************* */
static void amciod_run(cx_t *cx, servo_vars_t *servos) {
    /*  Listen on the motor command channel, and issue an amcdrive command for
     *  each incoming message.
     */

    // start update thread
    pthread_t update_thread;
    int r = pthread_create( &update_thread, NULL, amciod_update_thfun, cx );
    somatic_d_require( &cx->d, 0 == r,
                       "couldn't create feedback thread: %s",
                       strerror(r) );

    // now we're running
    somatic_d_state( &cx->d, SOMATIC__EVENT__CODES__PROC_RUNNING );
    // process commands
    while (!somatic_sig_received) {
        // free memory
        aa_mem_region_release( &cx->d.memreg );

        // returns the absolute time when receive should give up waiting
				struct timespec currTime;
				clock_gettime(CLOCK_MONOTONIC, &currTime);
				struct timespec abstime = aa_tm_add(cx->wait_time, currTime);

        /// read current state from state channel
        Somatic__MotorCmd *cmd =
            SOMATIC_D_GET( &r, somatic__motor_cmd, &cx->d,
                           &cx->cmd_chan, &abstime,
                           ACH_O_WAIT | ACH_O_LAST  );

        // check ach return
        somatic_d_check( &cx->d, SOMATIC__EVENT__PRIORITIES__CRIT,
                         SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT,
                         ( (ACH_OK == r || ACH_MISSED_FRAME == r) && cmd)
                         || ACH_TIMEOUT == r,
                         "amciod-run",
                         "ach result: %s", ach_result_to_string(r) );

        // validate message
				// TODO Validation should couple the parameter types with the values
				// s.t. i can't send a current without values and get away with ivalues
        if (cmd &&
            somatic_d_check_msg(
                &cx->d,
                cmd->has_param &&
                (SOMATIC__MOTOR_PARAM__MOTOR_CURRENT == cmd->param ||
                 SOMATIC__MOTOR_PARAM__MOTOR_HALT == cmd->param ||
                 SOMATIC__MOTOR_PARAM__MOTOR_RESET == cmd->param ||
								 SOMATIC__MOTOR_PARAM__MOTOR_DIGITAL_OUT == cmd->param),
                "motor_cmd", "invalid motor param, set: %d, val: %d",
                cmd->has_param, cmd->param) &&
            somatic_d_check_msg(
                &cx->d,
                (cmd->values && cmd->values->n_data == cx->n_servos) ||
								(cmd->ivalues && cmd->ivalues->n_data == 2) ||
                SOMATIC__MOTOR_PARAM__MOTOR_HALT == cmd->param ||
                SOMATIC__MOTOR_PARAM__MOTOR_RESET == cmd->param,
                "motor_cmd", "wrong motor count: %d, wanted %d",
                cmd->values->n_data, cx->n_servos )
            ){
            // execute message
            amciod_execute(cx, servos, cmd );
        }

        if (opt_verbosity) {
            for( size_t i = 0; opt_verbosity && i < n_modules; i++ ) {
                printf("%d:\t", i);
                printf("pos: %.0f [cnt]\t",
                       servos[i].act_pos);
                printf("vel: %.3f [rad/s]\t",
                       servos[i].act_vel);
                printf("state: %s (0x%x)\n",
                       amccan_state_string(amccan_decode_state(servos[i].status)),
                       servos[i].status);
            }
        }

        if( opt_verbosity >= 2 ) {
            for( size_t i = 0; i < n_modules; i++ ) {
                printf(" -- STATUS %d -- \n", i );
                amcdrive_dump_status( stdout, servos[i].status );
            }
        }
    }
    // now we're stopping
    somatic_d_state( &cx->d, SOMATIC__EVENT__CODES__PROC_STOPPING );

    // join update thread
    pthread_join( update_thread, NULL );
}

/* ********************************************************************************************* */
/* Run drive state updates in a separate thread.  Since this can
 * happen entirely independently of drive commands, it seems
 * reasonable to use multiple thread here.  However, if we want the
 * daemon to do things like check limits before issuing commands, we'd
 * better put things back in one thread.
 *
 */
static void *amciod_update_thfun( void *_cx ) {
    cx_t *cx = (cx_t*)_cx;
    while( !somatic_sig_received ) {


        // Update amcdrive state
        int r =  amcdrive_update_drives(cx->servos, n_modules);


        somatic_hard_assert( r == NTCAN_SUCCESS, "Cannot update drive states!\n");
        for( size_t i = 0; i < n_modules; i++ ) {
            cx->state_msg->position->data[i] =  cx->servos[i].act_pos;
            cx->state_msg->velocity->data[i] =  cx->servos[i].act_vel;
            cx->state_msg->current->data[i] =  cx->servos[i].act_cur;
        }
        somatic_metadata_set_time_now( cx->state_msg->meta );

        r = SOMATIC_PACK_SEND( &cx->state_chan,
                               somatic__motor_state, cx->state_msg );

        /// FIXME: check message transmission
        somatic_d_check( &cx->d, SOMATIC__EVENT__PRIORITIES__CRIT,
                         SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT,
                         ACH_OK == r,
                         "update_state",
                         "ach result: %s", ach_result_to_string(r) );

        // check status words here
        for( size_t i = 0; i < n_modules; i ++ ) {
            int16_t status = cx->servos[i].status;
            int16_t prev_status = cx->servos[i].prev_status;
            int state = amccan_decode_state( status );
            int prev_state = amccan_decode_state( prev_status );
            if( state != prev_state ) {
                if( opt_verbosity ) printf("Status Change\n");
                somatic_d_event( &cx->d, SOMATIC__EVENT__PRIORITIES__NOTICE,
                                 SOMATIC__EVENT__CODES__UNKNOWN,
                                 NULL, "drive 0x%x: %s(0x%x) => %s(0x%x)",
                                 cx->servos[i].canopen_id,
                                 amccan_state_string(prev_state),
                                 prev_status,
                                 amccan_state_string(state),
                                 status);
            }
            // NOTE: now we need to reset the prev_status since we've //   already checked it and the next CAN message likely
            //   won't update the status word
            cx->servos[i].prev_status = status;

        }

				// If digital out command is received from a controller daemon (i.e. krang-balance-hack), send the can message to the drive.
				if( cx->servos[1].digitalOutCmdReceived == 1  ) {
						uint8_t rcmd;
						pthread_mutex_lock(&handleMutex);
						r =	amccan_dl_virtual_out_ctrl(cx->servos[1].handle, &rcmd, cx->servos[1].canopen_id, vout );
						pthread_mutex_unlock(&handleMutex);
						if( r	!= NTCAN_SUCCESS ) {
							fprintf(stderr, "Couldn't set the virtual output: %s\n",  canResultString(r));
						}
						else { fprintf(stderr, "update: virtual output set to 0x%4x\n", vout); }
						cx->servos[1].digitalOutCmdReceived = 0;
				}


    }
    return NULL;
}

/* ********************************************************************************************* */
static void
amciod_execute( cx_t *cx, servo_vars_t *servos, Somatic__MotorCmd *msg ) {
    NTCAN_RESULT status;

    // amcdrive daemon currently accepts only current command
    if (msg->param == SOMATIC__MOTOR_PARAM__MOTOR_CURRENT ) {

				// Print motor commands received from the command channel
				if (opt_verbosity > 0) {
						for (size_t i = 0; i < msg->values->n_data; ++i)
								fprintf(stdout, "%lf::", msg->values->data[i]);
						fprintf(stdout, "]\n");

						//size_t size = somatic__motor_cmd__get_packed_size(msg);
						//printf("\tmotor_cmd packed size = %d \n",size);
				}

				double motorLeftCurrent = msg->values->data[0];
				double motorRightCurrent = msg->values->data[1];

				// Current limit
				motorLeftCurrent = min(MAX_CURRENT, max(-MAX_CURRENT, motorLeftCurrent));
				motorRightCurrent = min(MAX_CURRENT, max(-MAX_CURRENT, motorRightCurrent));

				// Send current to amcdrive
				pthread_mutex_lock(&handleMutex);
				status = amcdrive_set_current(&servos[0], motorLeftCurrent);
				aa_hard_assert( status == NTCAN_SUCCESS, "Cannot set current (Left)!\n");
				status = amcdrive_set_current(&servos[1], motorRightCurrent);
				aa_hard_assert( status == NTCAN_SUCCESS, "Cannot set current (Right)!\n");
				pthread_mutex_unlock(&handleMutex);
		}

		// If digital out command is received from a controller daemon (i.e. krang-balance-hack),
		// indicate a can msg should be sent in the other thread.
		else if( msg->param == SOMATIC__MOTOR_PARAM__MOTOR_DIGITAL_OUT) {
				uint8_t rcmd; NTCAN_RESULT r;
				// TODO: See which port number is to be written and write only that port. Right now it is
				// writing on all ports
				if(msg->ivalues->data[1]==0) { vout = 0x0000;	}
				else { vout = 0xFFFF;	}
				servos[1].digitalOutCmdReceived = 1;
		}
		else {
        fprintf(stdout, "ERROR: Accept only current and digital out ctrl \
					command.\n");
        exit(0);
    }
}

