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

#include <argp.h>
#include "somatic.h"
#include "somatic/daemon.h"
#include <amino.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <cblas.h>
#include <somatic.pb-c.h>
#include <somatic/util.h>
#include <ach.h>
#include <list>
#include <vector>

using namespace std;


#define COUNT 300
#define SLEEP_S 0.1

#define CHANNELS 3

/* ------- */
/* GLOBALS */
/* ------- */

static const char *opt_channel_name = "motor";
static const char *opt_quantity = "position";
static double opt_range_min = -10;
static double opt_range_max = 10;
static size_t opt_samples = 100;
static double opt_frequency = 10;

ach_channel_t global_channel;
static list<vector<double> > global_samples;
static FILE* global_gnuplot;
static uint8_t global_buf[1024*64];


/* ---------- */
/* ARGP Junk  */
/* ---------- */

// /* ------- */
// /* HELPERS */
// /* ------- */


/// argp program version
const char *argp_program_version = "somatic_motor_plot 0.0";

struct argp_option argp_options[] = {
    { "verbose", 'v', NULL, 0, "Causes verbose output" },
    { "channel", 'c', "channel-name", 0, "motor state channel" },
    { "quantity", 'x', "position|velocity|acceleration|current", 0, "quantity to plot" },
    { "samples", 'n', "count", 0, "number of samples to plot" },
    { "min", '0', "min-value", 0, "minimum range value" },
    { "max", '1', "max-value", 0, "maximum range value" },
    { "frequency", 'f', "hertz", 0, "Frequency to plot" },
    { NULL, 0, NULL, 0, NULL }
};

extern int parse_opt( int key, char *arg, struct argp_state *state);

/// argp object
struct argp argp = { argp_options, parse_opt,
                     "args", "Plots a motor quantity",
                     NULL, NULL, NULL };
static void plot_data() {
    size_t n_series = global_samples.front().size();

    for( size_t k = 0; k < n_series; k ++ ) {
        size_t i = 0;
        for( list< vector<double> >::iterator itr = global_samples.begin();
             itr != global_samples.end();
             itr++, i++ ) {
            assert( (*itr).size() == n_series );
            fprintf(global_gnuplot, "%f, %f\n", i/opt_frequency, (*itr)[k] );

        }
        fprintf(global_gnuplot, "e\n" );
    }
    fflush( global_gnuplot );
}

static void init() {
    // install signale handler
    somatic_sighandler_simple_install();

    // open channel
    {
        ach_status_t r = (ach_status_t) ach_open( &global_channel, opt_channel_name, NULL );
        aa_hard_assert(ACH_OK == r, "Error opening ach channel\n" );
        somatic_verbprintf(1, "Opened channel: %s (%s)\n",
                           opt_channel_name, ach_result_to_string(r) );
        r = (ach_status_t) ach_flush( &global_channel );
        aa_hard_assert(ACH_OK == r, "Error flushing ach channel\n" );
    }

    // open gnuplot
    global_gnuplot = popen("gnuplot -persist", "w");

    fprintf(global_gnuplot, "set title 'Channel: %s, Quantity: %s'\n",
            opt_channel_name, opt_quantity);
    fprintf(global_gnuplot, "set xlabel 'Time (s)'\n");
    fprintf(global_gnuplot, "set ylabel '%s\n", opt_quantity);
    fprintf(global_gnuplot, "set yrange [%f:%f]\n", opt_range_min, opt_range_max);


}

static void collect() {
    size_t nread;
    ach_status_t r = (ach_status_t)
        ach_get( &global_channel, global_buf, sizeof(global_buf),
                       &nread, NULL, ACH_O_WAIT | ACH_O_LAST);
    aa_hard_assert( (ACH_OK == r) || (ACH_MISSED_FRAME == r), "Failed to read: %s.\n",
                    ach_result_to_string(r) );
    Somatic__MotorState *msg =
        somatic__motor_state__unpack( &protobuf_c_system_allocator, nread, global_buf );

    Somatic__Vector *vec = NULL;
    if( 0 == strcmp("position", opt_quantity ) )
        vec = msg->position;
    else if( 0 == strcmp("velocity", opt_quantity ) )
        vec = msg->velocity;
    else if( 0 == strcmp("acceleration", opt_quantity ) )
        vec = msg->acceleration;
    else if( 0 == strcmp("current", opt_quantity ) )
        vec = msg->current;

    somatic__motor_state__free_unpacked( msg, &protobuf_c_system_allocator );

    if( global_samples.size() > opt_samples )
        global_samples.pop_front();
    global_samples.push_back( vector<double>(vec->data, vec->data + vec->n_data) );
}


static void plot() {
    static int first_time = 1;
    if( first_time ) {
        first_time = 0;

    // plot header
     fprintf(global_gnuplot, "plot '-' with lines title '0'");
     for( int i = 0; i < (int)global_samples.front().size(); i++ ) {
         fprintf(global_gnuplot, ", '-' with lines title '%d'", i);
     }
     fprintf(global_gnuplot, "\n");
    } else {
        fprintf(global_gnuplot, "replot\n");
    }
}

static void run() {
    while(!somatic_sig_received) {
        collect();
        plot();
        usleep( (int) (1e6 / opt_frequency));
    }
}

static void destroy() {
    // close channel
    {
        ach_status_t r = (ach_status_t) ach_close( &global_channel );
        if( ACH_OK != r ) {
            fprintf(stderr, "Error closing ach channel: %s\n", ach_result_to_string( r ) );
        } else {
            somatic_verbprintf(1, "Closed channel: %s (%s)\n",
                               opt_channel_name, ach_result_to_string(r) );
        }
    }
    fclose( global_gnuplot );
}

/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {
  (void) argc;
  (void) argv;
  somatic_verbprintf_prefix = "motor_plot";
  argp_parse (&argp, argc, argv, 0, NULL, NULL);
  init();
  run();
  destroy();

  return 0;
}

static void parse_error( const char *name) {
    fprintf(stderr, "Error parsing %s\n", name);
    exit(EXIT_FAILURE);
}

int parse_opt( int key, char *arg, struct argp_state *state) {
    (void)state;
    char *endptr = 0;
    switch (key) {
    case 'v':
        somatic_opt_verbosity++;
        break;
    case 'c':
        opt_channel_name = strdup(arg);
        break;
    case 'x':
        opt_quantity = strdup(arg);
        break;
    case 'n':
        opt_samples = strtol( arg, &endptr, 10 );
        if( NULL == endptr ) parse_error( "samples (-n)");
        break;
    case '0':
        opt_range_min = strtod( arg, &endptr );
        if( NULL == endptr ) parse_error( "min (-0)");
        break;
    case '1':
        opt_range_max = strtod( arg, &endptr );
        if( NULL == endptr ) parse_error( "max (-1)");
        break;
    case 'f':
        opt_frequency = strtod( arg, &endptr );
        if( NULL == endptr ) parse_error( "frequency (-f)");
        break;
    };
    return 0;
}
