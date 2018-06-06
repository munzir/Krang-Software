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
 * @file util.cpp
 * @author Neil Dantam, Can Erdogan
 * @date June 28, 2013
 * @brief Contains basic utility functions such as conditional printfs, fail checks, signal
 * handlers and etc.
 */

#include <amino.h>
#include <signal.h>
#include <dirent.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/kd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include "somatic.h"
#include "somatic/util.h"
#include "somatic/lapack.h"

int somatic_opt_verbosity = 0;
int somatic_sig_received = 0;
int somatic_motor_state = 0;
pid_t spid;

const char *somatic_verbprintf_prefix = "sns";
void somatic_verbprintf( int level, const char fmt[], ... ) {
	va_list argp;
	va_start( argp, fmt );
	if( level <= somatic_opt_verbosity ) {
		fprintf(stderr, "[%s] ", somatic_verbprintf_prefix);
		vfprintf( stderr, fmt, argp );
	}
	va_end( argp );
}

void somatic_fail( const char fmt[], ... ) {
	va_list argp;
	va_start( argp, fmt );
	fprintf(stderr, "ERROR: ");
	vfprintf( stderr, fmt, argp );
	va_end( argp );
	abort();
	exit(EXIT_FAILURE);
}


void somatic_hard_assert( int test, const char fmt[], ... ) {
	if( ! test ) {
		va_list argp;
		va_start( argp, fmt );
		fprintf(stderr, "ERROR: ");
		vfprintf( stderr, fmt, argp );
		va_end( argp );
		abort();
		exit(EXIT_FAILURE);
	}
}


static void somatic_sighandler_simple (int sig, siginfo_t *siginfo, void *context)
{
	(void) context;
	somatic_verbprintf (1,
					  "Received Signal: %d, Sending PID: %ld, UID: %ld\n",
						sig, (long)siginfo->si_pid, (long)siginfo->si_uid);

	if(sig == SIGINT || sig == SIGTERM) {
		somatic_verbprintf(1, "setting somatic_sig_received=1\n");
		somatic_sig_received = 1;
	} else if(sig == SIGMSTART) {
		somatic_motor_state = 1;
	} else if (sig == SIGMSTOP) {
		somatic_motor_state = 0;
	} else if (sig == SIGMABORT) {
		somatic_motor_state = 0;
		somatic_sig_received = 1;
	} else {
		somatic_verbprintf(1, "Received a signal. No action defined.\n");
	}
}

// FIXME: think about this more
void somatic_sighandler_simple_install() {
	struct sigaction act;
	memset(&act, 0, sizeof(act));

	act.sa_sigaction = &somatic_sighandler_simple;

	/* The SA_SIGINFO flag tells sigaction() to use the sa_sigaction field,
	   not sa_handler. */
	act.sa_flags = SA_SIGINFO;

	if (sigaction(SIGMSTART, &act, NULL) < 0) {
		perror ("sigaction");
				somatic_fail( "Couldn't install handler\n");
	}

	if (sigaction(SIGMSTOP, &act, NULL) < 0) {
		perror ("sigaction");
				somatic_fail( "Couldn't install handler\n");
	}

	if (sigaction(SIGMABORT, &act, NULL) < 0) {
		perror ("sigaction");
				somatic_fail( "Couldn't install handler\n");
	}

	if (sigaction(SIGTERM, &act, NULL) < 0) {
		perror ("sigaction");
		somatic_fail( "Couldn't install handler\n");
	}

	if (sigaction(SIGINT, &act, NULL) < 0) {
		perror ("sigaction");
		somatic_fail( "Couldn't install handler\n");
	}

}

void somatic_sighandler_send_alive(pid_t pid, sigval_t sigval) {
		sigqueue(pid, SIGUSR1, sigval);
}

/* void somatic_wait_copd() { */
/*	  fprintf(stderr, "Finding copd\n"); */
/*	  while(!(somatic_find_copd() || somatic_sig_received)) { */
/*			  fprintf(stderr, "Waiting for copd\n"); */
/*			  usleep(10000); */
/*	  } */
/* } */

/* int somatic_find_copd() { */
/*	  DIR* dir = opendir("/proc/"); */
/*	  struct dirent *ent; */
/*	  char* name = (char*) malloc (100); */
/*	  pid_t pid; */
/*	  int fd, len; */
/*	  char buffer[4096]; */
/*	  int ret = 0; */

/*	  while ((ent = readdir(dir)) != NULL) */
/*	  { */
/*			  if (!isdigit(ent->d_name[0])) continue; */

/*			  pid = atoi(ent->d_name); */
/*			  sprintf(buffer, "/proc/%d/stat", pid); */
/*			  if((fd = open(buffer, O_RDONLY)) != -1) */
/*			  { */
/*					   if((len = read(fd, buffer, 1000)) > 1) */
/*					   { */
/*							   strtok(buffer, "("); */
/*							   name = strtok(NULL, ")"); */

/*							   if(strcmp(name, "copd") == 0) */
/*							   { */
/*									  fprintf(stderr, "%d:%s\n", pid, name); */
/*									  spid = pid; */
/*									  ret = 1; */
/*									  break; */
/*							   } */
/*					   } */
/*			  } */

/*			  close(fd); */
/*	  } */
/*	  closedir(dir); */
/*	  return ret; */
/* } */


int somatic_la_invert( size_t m, size_t n, double *A ) {
	int ipiv[m];
	int mi = (int) m;
	int ni = (int) n;
	int info;

	// LU-factor
	dgetrf_( &mi, &ni, A, &mi, ipiv, &info );

	// find optimal size
	double swork[1];
	int lwork_query = -1;
	dgetri_( &ni, A, &mi, ipiv, swork, &lwork_query, &info );
	int lwork = (int) swork[0];

	// invert
	double work[lwork];
	dgetri_( &ni, A, &mi, ipiv, work, &lwork, &info );

	return info;
}

// for fortran
int somatic_la_invert_( const int *m, const int *n, double *A ) {
	return somatic_la_invert( (size_t)*m, (size_t)*n, A );
}

/*****************************************************************
 * For managing ach channels, so we don't have to use achtool
 *
 * These are just functions pulled from ez.c and achtool.c, so
 * anyone developing a daemon can use them
 */

/*
 * Creates an ach channel
 */
int somatic_create_channel(const char *name, size_t frame_cnt, size_t frame_size) {

	ach_status_t i;
	{
		ach_create_attr_t attr;
		ach_create_attr_init(&attr);
		i = ach_create( (char*)name, frame_cnt, frame_size, &attr );
	}
	somatic_hard_assert( ACH_OK == i, "Error creating channel: %s\n",
							 ach_result_to_string( i ) );

	return i;
}

/*
 * Opens named ach channel
 */
ach_channel_t* somatic_open_channel(const char *name)
{
	ach_channel_t *chan = SOMATIC_NEW( ach_channel_t );
	ach_status_t r = ach_open( chan, name, NULL );
	somatic_hard_assert( ACH_OK == r, "Error opening channel: %s\n",
						 ach_result_to_string( r ) );

	// Uncomment to set channel permissions on every open call
	ach_chmod( chan, SOMATIC_CHANNEL_MODE );

	r = ach_flush( chan );
	somatic_hard_assert( ACH_OK == r, "Error flushing channel: %s\n",
						 ach_result_to_string( r ) );

		return(chan);
}

/*
 * Closes a channel
 */
int somatic_close_channel(ach_channel_t *chan)
{
		ach_status_t r = ach_close( chan );
		somatic_hard_assert( ACH_OK == r, "Error closing channel: %s\n",
								 ach_result_to_string( r ) );

		return(r);
}




const somatic_prototable_t *somatic_prototable_lookup_key( const somatic_prototable_t *table,
													 const char *key) {
	for( size_t i = 0; NULL != table[i].key; i ++ ) {
		if( 0 == strcmp(key, table[i].key ) ) return &table[i];
	}
	return NULL;
}
const somatic_prototable_t *somatic_prototable_lookup_value( const somatic_prototable_t *table,
													   uint32_t value) {

	for( size_t i = 0; NULL != table[i].key; i ++ ) {
		if( value == table[i].value ) return &table[i];
	}
	return NULL;
}

/* int somatic_prototable_key2value( somatic_prototable_t *table, */
/*								   const char *key, */
/*								   uint32_t *value ) { */
/*	 somatic_prototable_t *ent = somatic_prototable_lookup_key(table, key); */
/*	 if( ent ) {  */
/*		 *value = ent->value; */
/*		 return 0;  */
/*	 } else return -1; */
/* } */

const char* somatic_prototable_value2key( const somatic_prototable_t *table,
										  uint32_t value) {
	const somatic_prototable_t *ent = somatic_prototable_lookup_value(table, value);
	if( ent ) return ent->key;
	else return "unknown";
}

int somatic_beep( int fd, double freq, double dur ) {
	// PC mainboard timer 8254 is clocked at 1.19 MHz
	static const double TICK_RATE =  1193180;
	static const double COUNT_RATE = 1000; // seems to work
	long tone = (long)(TICK_RATE / freq);
	//long durticks = TICK_RATE * dur * TICK_KLUDGE;
	long durticks = (long)(dur * COUNT_RATE);
	long argument = tone | (durticks << (8*sizeof(long) / 2));
	printf("0x%lx 0x%lx : 0x%lx\n", tone, durticks, argument);
	return ioctl(fd, KDMKTONE, (long) argument);
	//return write(fd, "\a", 1);
}
