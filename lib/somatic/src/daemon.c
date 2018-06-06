/* -*- mode: C; c-basic-offset: 4 -*- */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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
// needed for getsid
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <amino.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include "somatic.h"
#include "somatic/daemon.h"
#include <sys/resource.h>
#include <sched.h>
#include <sys/mman.h>

#include <syslog.h>

#define SOMATIC_SYSLOG_FACILITY LOG_USER
#define SOMATIC_SYSLOG_OPTION (LOG_CONS | LOG_NDELAY | LOG_PERROR)

#define HOSTNAME_MAX_SIZE 512

static void d_check(int test, const char *fmt, ... ) {
    if( ! test ) {
        va_list ap;
        va_start(ap, fmt);
        vsyslog(LOG_CRIT, fmt, ap);
        va_end(ap);
        exit(EXIT_FAILURE);
    }
}

AA_API void somatic_d_daemonize( somatic_d_t *d ) {
    // Q: Should we do some setup work before or after the fork?
    pid_t pid;

    // check if already daemonized
    if( 1 == getppid() ) { // getppid always successful
        syslog(LOG_NOTICE, "parent is init, odd");
        d->pid = getpid();
        return;
    }

    // open new fds
    int new_out = open( "out", O_APPEND|O_CREAT|O_WRONLY, S_IRUSR|S_IWUSR|S_IRGRP|S_IROTH );
    d_check( new_out >= 0, "Couldn't open daemon output: %s", strerror(errno));

    // fork
    d_check( (pid = fork()) >= 0, "fork failed: %s", strerror(errno) );
    if( pid ) exit(EXIT_SUCCESS); // parent dies, now in child

    // set session id to lose our controlling terminal
    d_check( setsid() > 0, "Couldn't set sesion id: %s",
             strerror(errno) );

    // refork to prevent future controlling ttys
    struct sigaction sa;
    sa.sa_handler = SIG_IGN;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    d_check( !sigaction(SIGHUP, &sa, NULL),
             "Couldn't ignore SIGHUP: %s", strerror(errno) );
    d_check( (pid = fork()) >= 0, "Couldn't refork: %s", strerror(errno) );
    if( pid ) exit(EXIT_SUCCESS);  // second child dies, now in grandchild

    d->pid = getpid();

    // reopen syslog, no print to stderr
    closelog();
    openlog(d->ident, SOMATIC_SYSLOG_OPTION & ~LOG_PERROR,
            SOMATIC_SYSLOG_FACILITY);

    // dup fds
    d_check( dup2( new_out, STDOUT_FILENO ) , "dup to stdout failed: %s", strerror(errno) );
    d_check( dup2( new_out, STDERR_FILENO ) , "dup to stderr failed: %s", strerror(errno) );
    close(new_out);
    close(STDIN_FILENO); // daemon doesn't need to read stdin
}

AA_API void somatic_d_init( somatic_d_t *d, somatic_d_opts_t *opts ) {
    int rint;
    ach_status_t rach;
    // copy opts
    d->opts = *opts;
    d->opts.ident = NULL;
    d->opts.prefix = NULL;

    // open syslog
    openlog( (opts && opts->ident) ? opts->ident: "somatic",
             SOMATIC_SYSLOG_OPTION,
             SOMATIC_SYSLOG_FACILITY);

    // early check
    if( 0 != d->is_initialized ) {
        syslog(LOG_EMERG,
               "Tried to reinitialize somatic_d_t instance.  Quitting.");
        exit(EXIT_FAILURE);
    }

    // set limits
    {
        struct rlimit lim;
        // get max core size
        if( getrlimit( RLIMIT_CORE, &lim ) ) {
            syslog(LOG_ERR, "Couldn't get RLIMIT_CORE: %s", strerror(errno));
        } else {
            // set core size
            lim.rlim_cur = (lim.rlim_max < SOMATIC_D_DEFAULT_CORE_SIZE) ?
                lim.rlim_max : SOMATIC_D_DEFAULT_CORE_SIZE;
            if( setrlimit( RLIMIT_CORE, &lim ) ) {
                syslog(LOG_ERR, "Couldn't get RLIMIT_CORE: %s", strerror(errno));
            }
        }
    }

    // set ident
    d->ident = strdup((opts && opts->ident) ? opts->ident : "somatic");
    somatic_verbprintf_prefix  = d->ident;

    // setup memory allocator
    aa_mem_region_init( &d->memreg,
                    opts->region_size ?
                    opts->region_size :
                    SOMATIC_D_DEFAULT_REGION_SIZE );
    aa_mem_region_init( &d->tmpreg,
                    opts->tmpregion_size ?
                    opts->tmpregion_size :
                    SOMATIC_D_DEFAULT_TMPREGION_SIZE );

    somatic_pbregalloc_set( &d->pballoc, &d->memreg );

    // file mask
    umask(0); // always successful

    // create working directory
    const char *dirnam = aa_mem_region_printf(&d->memreg, SOMATIC_RUNROOT"%s", d->ident);
		// printf("Attempting to make dir: '%s'\n", dirnam); fflush(stdout);
    rint = mkdir( dirnam, 0775 );
    d_check( !rint || EEXIST == errno, "Couldn't make working directory: %s (%d)",
             strerror(errno), errno);

    // chdir
    const char *wd = aa_mem_region_printf(&d->memreg,SOMATIC_RUNROOT"%s", d->ident);
    d_check( !chdir(wd),  "Couldn't chdir to `%s': %s", wd, strerror(errno));

    // open pid file
    const char *pidnam = somatic_d_pidnam( d->ident, &d->memreg );
    d->lockfd = open( pidnam, O_RDWR| O_CREAT,0664);
    d_check( 0 < d->lockfd, "Couldn't open pidfile `%s': %s", pidnam, strerror(errno));

    // check pid file lock before maybe forking
    d_check( !lockf( d->lockfd, F_TEST, 0 ), "Couldn't lock `%s/pid', daemon already running", dirnam );

    // daemonize
    if( d->opts.daemonize ) {
        somatic_d_daemonize(d);
    } else d->pid = getpid();

    // lock pid file
    d_check( !lockf(d->lockfd, F_TLOCK, 0),
             "Couldn't lock `%s' in child, possible race: %s", strerror(errno));
    d->lockfile = fdopen(d->lockfd, "w");
    d_check( NULL != d->lockfile, "Couldn't fdopen pidfile `%s/pid': %s", dirnam, strerror(errno));

    // write pid
    rint = fprintf(d->lockfile, "%d", d->pid );
    d_check( 0 < rint, "Couldn't write pid to `%s/pid': printf said %d", dirnam, rint);
    do{ rint = fflush(d->lockfile); }
    while( 0 != rint && EINTR == errno );
    d_check( !rint, "Couldn't flush pid to `%s/pid':  %s", dirnam, strerror(errno));

    // open channels
    {
        if( 0 != (rach = ach_open(&d->chan_event, "event", NULL)) ) {
            syslog(LOG_EMERG, "Couldn't open event channel: %s\n",
                   ach_result_to_string(rach));
            exit(EXIT_FAILURE);
        }
    }

    // set host
    {
        char buf[HOSTNAME_MAX_SIZE];
        if( ! gethostname(buf,HOSTNAME_MAX_SIZE-2) ) {
            d->host = strdup(buf);
        }else{
            d->host = strdup("0.0.0.0");
            syslog(LOG_ERR, "Couldn't get hostname: %s", strerror(errno));
        }
    }

    // log direct
    //syslog(LOG_NOTICE, "init daemon");

    // set scheduling policy
    {
        int max = sched_get_priority_max( SCHED_RR );
        int min = sched_get_priority_min( SCHED_RR );
        if( max >= 0 && min >= 0 ) {
            if( opts->sched_rt > 0 && opts->sched_rt >= min ) {
                struct sched_param sp;
                /* 32 is max portable priority*/
                int pri = opts->sched_rt;
                if( pri > max ) {
                    syslog(LOG_WARNING, "Requested priority %d exceeds max %d",
                           pri, max);
                    pri = max;
                }
                sp.sched_priority = pri;
                if( sched_setscheduler( 0, SCHED_RR, &sp) < 0 ) {
                    syslog(LOG_ERR, "Couldn't set scheduling priority to %d: %s\n",
                           pri, strerror(errno) );
                }
            }
        } else {
            syslog(LOG_ERR, "Couldn't get scheduling priorities: %d, %d, %s\n",
                   max, min, strerror(errno) );
        }
    }

    // lock memory, can't swap to disk
    if( ! opts->skip_mlock ) {
        if( mlockall( MCL_CURRENT | MCL_FUTURE ) ) {
            syslog( LOG_ERR, "Couldn't lock pages in memory: %s",
                    strerror(errno) );
        }
    }

    // install signale handler
    if( ! opts->skip_sighandler )
	    somatic_sighandler_simple_install();

    // set state
    d->is_initialized = 1;

    // notify event
    somatic_d_event( d, SOMATIC__EVENT__PRIORITIES__NOTICE,
                     SOMATIC__EVENT__CODES__PROC_STARTING,
                     NULL, NULL );
    // release regions
    aa_mem_region_release(&d->memreg);

}

AA_API void somatic_d_destroy( somatic_d_t *d) {
    somatic_d_event( d, SOMATIC__EVENT__PRIORITIES__NOTICE,
                     SOMATIC__EVENT__CODES__PROC_HALTED,
                     NULL, NULL );

    int r;
    // undaemonize
    if( d->opts.daemonize ) {
        // close the pid file, this clobbers our lock as well
        char *nam = aa_mem_region_printf(&d->memreg, SOMATIC_RUNROOT"%s/pid", d->ident);
        if( 0 != fclose(d->lockfile) ) {
            syslog(LOG_ERR, "Error closing `%s': %s", nam, strerror(errno) );
        }
        // unlink pid file so nobody gets confuse later
        if( 0 != unlink(nam) ) {
            syslog(LOG_ERR, "Error unlinking `%s': %s", nam, strerror(errno) );
        }
    }

    // close channels
    r = ach_close(&d->chan_event);

    // free
    aa_mem_region_destroy(&d->memreg);
    aa_mem_region_destroy(&d->tmpreg);

    // close log
    //syslog(LOG_NOTICE, "destroy daemon");
    closelog();

    // free ident (used by syslog)
    if( d->ident ) free(d->ident);
}

AA_API void somatic_d_state( somatic_d_t *d, Somatic__Event__Codes state ) {
    somatic_d_event( d, SOMATIC__EVENT__PRIORITIES__NOTICE,
                     state,
                     NULL, NULL );
}

AA_API void somatic_d_require( somatic_d_t *d, int test,
                               const char fmt[], ... ) {
    if( ! test ) {
        va_list arg;
        va_start( arg, fmt );
        somatic_d_vevent(d, SOMATIC__EVENT__PRIORITIES__EMERG,
                         SOMATIC__EVENT__CODES__BAD_ASSERT,
                         NULL, fmt, arg);
        va_end( arg );
        somatic_d_die(d);
    }
}

AA_API int somatic_d_vcheck_bit( somatic_d_t *d, int mask,
                                 bool notify, int prev, int word,
                                 Somatic__Event__Priorities level, Somatic__Event__Codes code,
                                 const char *type, const char fmt[],
                                 va_list arg ) {
    bool pbit = (prev & mask) ? 1 : 0;
    bool cbit = (word & mask) ? 1 : 0;
    if( (pbit != cbit) && (cbit == notify) ) {
        somatic_d_vevent( d, level, code, type, fmt, arg );
        return 1;
    } else {
        return 0;
    }
}

AA_API int somatic_d_check_bit( somatic_d_t *d, int mask,
                                bool notify, int prev, int word,
                                Somatic__Event__Priorities level, Somatic__Event__Codes code,
                                const char *type, const char fmt[],
                                ... ) {
    va_list arg;
    va_start( arg, fmt );
    int i = somatic_d_vcheck_bit( d, mask, prev, word, notify, level, code,
                                  type, fmt, arg);
    va_end(arg);
    return i;
}

void somatic_d_vevent( somatic_d_t *d, Somatic__Event__Priorities level, Somatic__Event__Codes code,
                       const char *type, const char comment_fmt[], va_list argp ) {
    Somatic__Event pb;
    memset(&pb, 0, sizeof pb);
    somatic__event__init(&pb);
    pb.priority = level;
    pb.has_priority = 1;
    pb.code = code;
    pb.has_code = 1;
    pb.ident = d->ident;
    pb.host = d->host;
    pb.pid = d->pid;
    pb.has_pid = 1;
    char cpy_type[ type ? strlen(type) : 0 ];
    char fmt_buf[ 512 ] = {0};
    if(type) {
        strcpy(cpy_type, type);
        pb.type = cpy_type;
    }

    if( comment_fmt ) {
        vsnprintf( fmt_buf, sizeof(fmt_buf)-2, comment_fmt, argp );
        pb.comment =  fmt_buf;
    }

    ach_status_t rach = SOMATIC_PACK_SEND( &d->chan_event, somatic__event, &pb );
    if( ACH_OK != rach ) {
        syslog( LOG_ERR, "couldn't send event: %s",
                ach_result_to_string(rach));
    }
}

AA_API void somatic_d_event( somatic_d_t *d, Somatic__Event__Priorities level, Somatic__Event__Codes code,
                             const char *type, const char comment_fmt[], ... ) {
    va_list argp;
    va_start( argp, comment_fmt );
    somatic_d_vevent(d, level, code, type, comment_fmt, argp);
    va_end( argp );

}

int somatic_d_vcheck( somatic_d_t *d, Somatic__Event__Priorities priority, Somatic__Event__Codes code,
                      int test, const char *type,
                      const char fmt[], va_list argp ) {
    if( !test ) {
        va_list argp2;
        va_copy( argp2, argp );
        if( d && d->is_initialized ) {
            somatic_d_vevent(d, priority, code, type, fmt, argp);
        }else {
            fprintf(stderr, "no valid somatic context, can't send event\n");
        }

        fprintf(stderr, "[%s]:%d (%d).(%s) ",
                d ? d->ident : "unknown",
                priority, code, type ? type : "");
        vfprintf(stderr, fmt, argp );
        fputc('\n', stderr);
        va_end(argp2);
    }
    return test;
}

AA_API int somatic_d_check( somatic_d_t *d, Somatic__Event__Priorities priority, Somatic__Event__Codes code,
                            int test, const char *type, const char fmt[], ... ) {
    if( !test ) {
        va_list argp;
        va_start( argp, fmt );
        somatic_d_vcheck( d, priority, code, test, type, fmt, argp );
        va_end( argp );
    }
    return test;
}

AA_API int somatic_d_assert_err( somatic_d_t *d, int test,
                                 const char fmt[], ... ) {
    if( !test ) {
        va_list argp;
        va_start( argp, fmt );
        somatic_d_vcheck( d, SOMATIC__EVENT__PRIORITIES__ERR,
                          SOMATIC__EVENT__CODES__BAD_ASSERT,
                          test, "assert", fmt, argp );
        va_end( argp );
    }
    return test;
}


AA_API void somatic_d_die(somatic_d_t *d) {
    (void)d;
    abort();
}

void somatic_d_channel_open(somatic_d_t *d,
                                   ach_channel_t *chan, const char *name,
                                   ach_attr_t *attr) {
	ach_status_t rach =  ach_open( chan, name, attr );
    somatic_d_check(d, SOMATIC__EVENT__PRIORITIES__EMERG,
                    SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT,
                    ACH_OK == rach, "ach_open",
                    "opening channel `%s': %s\n",
                    name, ach_result_to_string(rach));
    if( ACH_OK != rach ) somatic_d_die(d);
    rach =  ach_flush( chan );
    somatic_d_check(d, SOMATIC__EVENT__PRIORITIES__EMERG,
                    SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT,
                    ACH_OK == rach, "fflush",
                    "flushing channel `%s': %s\n",
                    name, ach_result_to_string(rach));
    if( ACH_OK != rach ) somatic_d_die(d);
}

void somatic_d_channel_close(somatic_d_t *d, ach_channel_t *chan ) {
	ach_status_t rach =  ach_close( chan );
    // not much to do if it fails, just log it
    somatic_d_check(d, SOMATIC__EVENT__PRIORITIES__ERR,
                    SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT,
                    ACH_OK == rach, "ach_close",
                    "closing channel: %s\n",
                    ach_result_to_string(rach));
}

AA_API void somatic_d_limit( somatic_d_t *d, Somatic__Event__Priorities level,
                             const char *type, int quantity,
                             int idx, double actual,
                             double min, double max) {
    (void)quantity;

    Somatic__Event pb;
    memset(&pb, 0, sizeof pb);
    somatic__event__init(&pb);
    pb.priority = level;
    pb.has_priority = 1;
    pb.code = SOMATIC__EVENT__CODES__LIMIT;
    pb.has_code = 1;
    pb.ident = d->ident;
    pb.host = d->host;
    pb.pid = d->pid;
    pb.has_pid = 1;
    char cpy_type[ type ? strlen(type)+2 : 0 ];
    if(type) {
        strcpy(cpy_type, type);
        pb.type = cpy_type;
    }
    // FIXME: add limit stuff

    ach_status_t rach = SOMATIC_PACK_SEND( &d->chan_event, somatic__event, &pb );
    if( ACH_OK != rach ) {
        syslog( LOG_ERR, "couldn't send event: %s",
                ach_result_to_string(rach));
    }

    fprintf(stderr, "[%s]:%d (%d).(%s) LIMIT i: %d, val: %f, min: %f, max: %f\n",
            d ? d->ident : "unknown",
            pb.priority, pb.code, type ? type : "",
            idx, actual, min, max);

 };

AA_API int somatic_d_check_v( somatic_d_t *d, Somatic__Event__Priorities priority,
                              Somatic__Event__Codes code,
                              const char *type,
                              double *data, size_t n,
                              double *min, double *max, size_t n_desired ) {
    int r = aa_valid_v( data, n, min, max, n_desired );
    somatic_d_check( d, priority, code,
                     r >= 0 && n == n_desired, type,
                     "length mismatch: %d given, %d wanted", n, n_desired );
    if( r > 0 ) {
        int idx = r-1;
        somatic_d_limit( d, priority, type, 0,
                         idx, data[idx], min[idx], max[idx] );

    }
    return !r ? 1 : 0;
}

AA_API int somatic_d_check_param( somatic_d_t *d, int test_val,
                                  const char *file_name,
                                  unsigned int line_no,
                                  const char *fun_name,
                                  const char *test_exp) {
    if( !test_val ) {
        somatic_d_check( d, SOMATIC__EVENT__PRIORITIES__ERR,
                         SOMATIC__EVENT__CODES__BAD_PARAM,
                         test_val,
                         fun_name, "%s:%d `%s'",
                         file_name, line_no, test_exp );
    }
    return test_val;
}

AA_API int somatic_d_check_msg( somatic_d_t *d, int test,
                                const char *type, const char *fmt, ... ) {
    if( !test) {
        va_list argp;
        va_start( argp, fmt );
        somatic_d_vcheck( d, SOMATIC__EVENT__PRIORITIES__ERR,
                          SOMATIC__EVENT__CODES__COMM_BAD_MSG,
                          test, type, fmt, argp );
        va_end(argp);
    }
    return test;
}

AA_API int somatic_d_check_msg_v( somatic_d_t *d, const char *type,
                              double *data, size_t n,
                              double *min, double *max, size_t n_desired ) {
    return somatic_d_check_v( d, SOMATIC__EVENT__PRIORITIES__ERR,
                              SOMATIC__EVENT__CODES__COMM_BAD_MSG,
                              type, data, n, min, max, n_desired );
}

AA_API void *somatic_d_get( somatic_d_t *d, ach_channel_t *chan, size_t *frame_size,
                            const struct timespec *ACH_RESTRICT abstime, int options, int *ret ) {
    size_t fs;
    while(1){
        fs = 0;
        // try to get the frame
        *ret = ach_get( chan, aa_mem_region_ptr(&d->tmpreg), aa_mem_region_freesize(&d->tmpreg),
                        &fs, abstime, options );
        if( ACH_OVERFLOW != *ret ) break;
        // if region is too small, get a bigger buffer
        aa_mem_region_tmpalloc( &d->tmpreg, *frame_size );
    }
    *frame_size = fs;
    return aa_mem_region_ptr(&d->tmpreg);
}


AA_API void somatic_d_argp_parse( int key, char *arg, somatic_d_opts_t *opt ) {
    switch(key) {
    case 'I':
        opt->ident = strdup(arg);
        break;
    case 'd':
        opt->daemonize = 1;
        break;
    }
}
