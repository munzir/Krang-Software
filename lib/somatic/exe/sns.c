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
 * @date July 29, 2013
 * @author Neil Dantam, Can Erdogan
 * @file sns.cpp
 * @brief Monitors somatic daemons, killing, getting their schedule and checking if they are
 * alive.
 */

#include "somatic.h"
#include "somatic/daemon.h"
#include <somatic.pb-c.h>
#include <argp.h>
#include <ach.h>
#include <sched.h>
#include <unistd.h>
#include <fcntl.h>
#include <syslog.h>

/// argp program version
const char *argp_program_version = "sns 0.0";
#define ARGP_DESC "somatic control program"

/*------------*/
/* PROTOTYPES */
/*------------*/

typedef struct {
} cx_t;

static pid_t sns_pid(const char *ident);
static int sns_kill(const char *ident, pid_t pid);
static int sns_get_sched(const char *ident, pid_t pid);

/// argp object
static int parse_opt( int key, char *arg, struct argp_state *state);
extern struct argp_option argp_options[];
extern struct argp argp;



/* ------- */
/* GLOBALS */
/* ------- */

aa_mem_region_t memreg;


/* ------- */
/* HELPERS */
/* ------- */


static pid_t sns_pid(const char *ident) {
	const char *nam = somatic_d_pidnam( ident, &memreg );
	FILE *fp = fopen(nam, "r");
	if( NULL == fp ) {
		fprintf(stderr, "Couldn't open %s: %s\n", nam, strerror(errno));
		exit(EXIT_FAILURE);
	}
	pid_t pid = 0;
	fscanf(fp, "%d", &pid);
	fclose(fp);
	if( !pid ) {
		fprintf(stderr, "Invalid pid\n");
		exit(EXIT_FAILURE);
	}
	return pid;
}

static int sns_kill(const char *ident, pid_t pid) {

	// Kill the requested program
	if( kill(pid, SIGTERM) ) {
		fprintf(stderr, "Couldn't kill `%s': %s\n", ident, strerror(errno));
		exit(EXIT_FAILURE);
	}

	// Remove its folder in the /var/run/somatic folder
	if(strlen(ident) > 0) {
		char command [256];
		sprintf(command, "rm -rf /var/run/somatic/%s", ident);
		system(command);
	}

	return 0;
}

static int sns_get_sched(const char *ident, pid_t pid) {
	int sched = sched_getscheduler(pid);
	if( -1 == sched ) {
		fprintf(stderr, "Couldn't get scheduler for `%s': %s\n", ident, strerror(errno));
		exit(EXIT_FAILURE);
	}
	const char *schedstr;
	switch(sched) {
	case SCHED_FIFO: schedstr = "FIFO"; break;
	case SCHED_RR: schedstr = "RR"; break;
	case SCHED_OTHER: schedstr = "OTHER"; break;
	default: schedstr = "?"; break;
	}
	struct sched_param param;
	if( sched_getparam( pid, &param ) ) {
		fprintf(stderr, "Couldn't get param for `%s': %s\n", ident, strerror(errno));
		exit(EXIT_FAILURE);
	}
	printf("sched: %s\t sched_priority: %d\n", schedstr, param.sched_priority );
	return 0;
}

static int sns_alive(const char *ident) {
	const char *pidnam = somatic_d_pidnam( ident, &memreg );
	int fd = open(pidnam, O_RDWR);
	if( fd < 0 ) {
		if( ENOENT == errno ) {
			exit(-1);
		} else {
			fprintf(stderr, "couldn't open `%s': %s",
					pidnam, strerror(errno));
			exit(EXIT_FAILURE);
		}

	}
	int r = lockf( fd, F_TEST, 0 );

	if( 0 == r ) {
		/* unlocked */
		exit(-1);
	} else if( (EACCES != errno && EAGAIN != errno) ) {
		fprintf(stderr, "couldn't testing lock `%s': %s",
				pidnam, strerror(errno));
		exit(EXIT_FAILURE);
	} else  {
		/* locked */
		return 0;
	}
}

/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {
	aa_mem_region_init(&memreg, 4*1<<10);
	argp_parse (&argp, argc, argv, 0, NULL, NULL);
	//static cx_t cx;

	return 0;
}


int parse_opt( int key, char *arg, struct argp_state *state) {
	(void)state;
	switch (key) {
	case 'k':
		sns_kill(arg, sns_pid(arg));
		break;
	case 's':
		sns_get_sched(arg, sns_pid(arg));
		break;
	case 'a':
		sns_alive(arg);
		break;
	}
	return 0;
}

/* ---------- */
/* ARGP Junk  */
/* ---------- */

struct argp_option argp_options[] = {
	{"kill", 'k', "IDENT", 0, "Kill daemon 'IDENT'"},
	{"sched", 's', "IDENT", 0, "query scheduler for 'IDENT'"},
	{"alive", 'a', "IDENT", 0, "returns success if daemon 'IDENT' is alive"},
	{NULL, 0, NULL, 0, NULL}
};

/// argp object
struct argp argp = { argp_options, parse_opt, "args", ARGP_DESC, NULL, NULL, NULL };
