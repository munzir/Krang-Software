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
 * @author Neil Dantam
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <argp.h>
#include <ach.h>
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>

/// argp program version
const char *argp_program_version = "sjstel 0.0";
#define ARGP_DESC "Get Joystick Messages and send Motor Commands"





/*------------*/
/* PROTOTYPES */
/*------------*/

typedef struct {
    somatic_d_t d;
    somatic_d_opts_t d_opts;
    ach_channel_t js_chan;
    ach_channel_t cmd_chan;
    size_t n;
    const char *opt_js_name;
    const char *opt_cmd_name;
    Somatic__MotorCmd *cmd;
} cx_t;

/** Initialize the daemon */
static void init(cx_t *cx);
/** Main daemon run loop */
static void destroy(cx_t *cx);
/** Cleanup for exit */
static void run(cx_t *cx);
/** Update state */
static void update(cx_t *cx);

/// argp object
static int parse_opt( int key, char *arg, struct argp_state *state);
extern struct argp_option argp_options[];
extern struct argp argp;

/* ------- */
/* HELPERS */
/* ------- */

static void init(cx_t *cx) {

    somatic_d_init(&cx->d,
                   &cx->d_opts);  // init daemon variables, channels, log, etc

    // open channel
    somatic_d_channel_open( &cx->d, &cx->cmd_chan,
                            cx->opt_cmd_name, NULL );
    somatic_d_channel_open( &cx->d, &cx->js_chan,
                            cx->opt_js_name, NULL );

    // msg
    cx->cmd = somatic_motor_cmd_alloc( cx->n );
    cx->cmd->param = SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY;
    cx->cmd->has_param = 1;
}


static void update(cx_t *cx) {
    int r;
    struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) );

    // get message
    Somatic__Joystick *msg =
        SOMATIC_D_GET( &r, somatic__joystick, &cx->d,
                       &cx->js_chan, &abstimeout, ACH_O_WAIT );
    // FIXME: validate
    // send command
    if( msg ) {
        // clear
        memset(cx->cmd->values->data, 0, sizeof(double)*cx->n);
        // set values
        for( size_t i = 0; i < msg->axes->n_data && i < cx->n; i ++ ) {
            cx->cmd->values->data[i] =  msg->axes->data[i];
        }
        // send
        SOMATIC_D_PUT(somatic__motor_cmd, &cx->d, &cx->cmd_chan, cx->cmd );
        //exit(0);
    }
}

static void run(cx_t *cx) {
    somatic_d_event( &cx->d, SOMATIC__EVENT__PRIORITIES__NOTICE,
                     SOMATIC__EVENT__CODES__PROC_RUNNING,
                     NULL, NULL );
    while(!somatic_sig_received) {
        update(cx);
        // free buffers allocated during this cycle
        aa_mem_region_release( &cx->d.memreg );
    }
    somatic_d_event( &cx->d, SOMATIC__EVENT__PRIORITIES__NOTICE,
                     SOMATIC__EVENT__CODES__PROC_STOPPING,
                     NULL, NULL );
}

void destroy(cx_t *cx) {
    // close channel
    somatic_d_channel_close( &cx->d, &cx->js_chan );
    somatic_d_channel_close( &cx->d, &cx->cmd_chan );
    // end daemon
    somatic_d_destroy(&cx->d);
}

/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {
    static cx_t cx;
    memset(&cx, 0, sizeof(cx));

    /* Set some options */
    cx.d_opts.ident = "sjsteld";
    cx.d_opts.sched_rt = SOMATIC_D_SCHED_UI; // logger not realtime
    cx.opt_js_name =  "js";
    cx.opt_cmd_name = "cmd";
    cx.n = 7; // FIXME: make an command line argument

    argp_parse (&argp, argc, argv, 0, NULL, &cx);

    init(&cx);
    run(&cx);
    destroy(&cx);

    return 0;
}


int parse_opt( int key, char *arg, struct argp_state *state) {
    cx_t *cx = (cx_t*)state->input;
    char *endptr = 0;
    switch (key) {
    case 'v':
        somatic_opt_verbosity++;
        break;
    case 'j':
        cx->opt_js_name = strdup(arg);
        break;
    case 'u':
        cx->opt_cmd_name = strdup(arg);
        break;
    case 'n':
        cx->n = strtoul(arg, &endptr, 10);
        assert( cx->n > 0 ) ;
        break;
    }

    somatic_d_argp_parse( key, arg, &cx->d_opts );

    return 0;
}

/* ---------- */
/* ARGP Junk  */
/* ---------- */


struct argp_option argp_options[] = {
    {"verbose", 'v', NULL, 0, "Causes verbose output"},
    { "cmd-channel", 'u', "channel-name", 0, "motor cmd channel" },
    { "js-channel", 'j', "channel-name", 0, "joystick state channel" },
    { "num", 'n', "axes", 0, "number of motor axes" },
    SOMATIC_D_ARGP_OPTS,
    { NULL, 0, NULL, 0, NULL }
};

/// argp object
struct argp argp = { argp_options, parse_opt,
                     "args", ARGP_DESC,
                     NULL, NULL, NULL };
