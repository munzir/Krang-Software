/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
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

#ifndef SOMATIC_UTIL_H
#define SOMATIC_UTIL_H

#include <amino.h>
#include <cblas.h>
#include <signal.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <ach.h>

#ifdef __cplusplus
extern "C" {
#endif

/// verbosity level for function somatic_verbprintf
extern int somatic_opt_verbosity;
/// future use. this has the pid of the watchdog
extern pid_t spid;

#define SIGMSTART SIGRTMIN+5
#define SIGMSTOP SIGRTMIN+6
#define SIGMABORT SIGRTMIN+7
/// mode_t mode for 666 access to daemon channels
#define SOMATIC_CHANNEL_MODE (S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH)

/// macro to squash compiler warning about unused argument to function
#define SOMATIC_UNUSED_ARG(expr) do { (void)(expr); } while (0)

/*----------------*/
/* Printing/Error */

/*----------------*/
/// prefix string for somatic_verbprintf
extern const char *somatic_verbprintf_prefix;

/** printf's message if level <= somatic_opt_verbosity */
void somatic_verbprintf( int level, const char fmt[], ... );

/** printf's message and exits */
void somatic_fail( const char fmt[], ... );

/** If test is false, printf's message and exits */
void somatic_hard_assert( int test, const char fmt[], ... );


/*------------*/
/* ALLOCATION */
/*------------*/

/// malloc n bytes and zero initialize.  Terminates on error.
/// C.E. Removed the deprecated flag here. I am not sure why it was deprecated
/// in the first place and what was supposed to be used instead.
static inline void *somatic_xmalloc( size_t n );
static inline void *somatic_xmalloc( size_t n ) {
    void *p = malloc(n);
    somatic_hard_assert( NULL != p, "Failed to allocate %d bytes.\n", n );
    return memset(p, 0, n );
}

/// allocate and zero initialize a new array of type with n elements
#define SOMATIC_NEW_AR( type, n ) ((type*) somatic_xmalloc( sizeof(type) * (n) ) )

/// allocate and zero initialize an object of type
#define SOMATIC_NEW( type ) ( SOMATIC_NEW_AR( type, 1 ) )


/*--------*/
/* Arrays */
/*--------*/

/// set array a of n elements to zero
#define SOMATIC_ZERO_AR( a, n ) ( memset( (a), 0, (n) * sizeof((a)[0]) ) )

/// convert double floats to single floats
static void somatic_d2s( float *dst, const double *src, size_t cnt ) {
    for( size_t i = 0; i < cnt; i++ )
        dst[i] = (float) src[i];
}


/// convert single floats to double floats
static void somatic_s2d( double *dst, const float *src, size_t cnt ) {
    for( size_t i = 0; i < cnt; i++ )
        dst[i] = (double) src[i];
}

/* /// allocate and zero initialize n doubles */
/* static double *somatic_malloc_real( size_t n ) AA_DEPRECATED; */
/* static double *somatic_malloc_real( size_t n ) { */
/*     return (double*) SOMATIC_NEW_AR( double, n ); */
/* } */

/* /// copy n double floats from src to dst */
/* static void somatic_realcpy( double *dst, const double *src, size_t n ) AA_DEPRECATED; */
/* static void somatic_realcpy( double *dst, const double *src, size_t n ) { */
/*     memcpy( dst, src, sizeof( dst[0] ) * n ); */
/* } */


/* /// set n double floats to val */
/* static void somatic_realset( double *dst, double val, size_t n ) AA_DEPRECATED; */
/* static void somatic_realset( double *dst, double val, size_t n ) { */
/*     for( size_t i = 0; i < n; i ++ ) */
/*         dst[i] = val; */
/* } */

/* /// set n double floats to val */
/* static void somatic_realprint( double *arr, size_t n ) AA_DEPRECATED; */
/* static void somatic_realprint( double *arr, size_t n ) { */
/*     for( size_t i = 0; i < n; i ++ ) */
/*         if (i < n - 1) */
/*                 printf("%lf::", arr[i]); */
/*         else */
/*                 printf("%lf\n", arr[i]); */
/* } */

/*----------------*/
/* LINEAR ALGEBRA */
/*----------------*/

/// calculate a column major matrix offset
/* static size_t somatic_la_colmajor_k( size_t rows, size_t cols, size_t i, size_t j ) AA_DEPRECATED; */
/* static size_t somatic_la_colmajor_k( size_t rows, size_t cols, size_t i, size_t j ) { */
/*     assert( i < rows ); */
/*     assert( j < cols ); */
/*     size_t k = (j * rows) + i; */
/*     assert( k < rows*cols ); */
/*     return k; */
/* } */

/* /// sum of squared differences */
/* static double somatic_la_ssd( const double *a, const double *b, size_t n ) AA_DEPRECATED; */
/* static double somatic_la_ssd( const double *a, const double *b, size_t n ) { */
/*     double r = 0; */
/*     for( size_t i = 0; i < n; i ++ ) { */
/*         double t = a[i] - b[i]; */
/*         r += t*t; */
/*     } */
/*     return r; */
/* } */
/* /// get a column major matrix entry */
/* static double somatic_la_mget( const double *m, size_t rows, size_t cols, */
/*                                size_t i, size_t j ) AA_DEPRECATED; */
/* static double somatic_la_mget( const double *m, size_t rows, size_t cols, */
/*                                size_t i, size_t j ) { */
/*     return m[ somatic_la_colmajor_k( rows, cols, i, j ) ]; */
/* } */

/* /// set a column major matrix entry */
/* static double somatic_la_mset( double *m, size_t rows, size_t cols, */
/*                              size_t i, size_t j, double v ) AA_DEPRECATED; */
/* static double somatic_la_mset( double *m, size_t rows, size_t cols, */
/*                              size_t i, size_t j, double v ) { */
/*     return m[ somatic_la_colmajor_k( rows, cols, i, j ) ] = v; */
/* } */

/* /// Set n*n matrix A to diagonal matrix with all diagonal entries equal to v. */
/* static void somatic_la_ident_v( double *A, size_t n, double v ) AA_DEPRECATED; */
/* static void somatic_la_ident_v( double *A, size_t n, double v ) { */
/*     somatic_realset( A, 0, n*n ); */
/*     for( size_t i = 0; i < n; i ++ ) { */
/*         somatic_la_mset(A, n, n, i, i, v); */
/*     } */
/* } */

/* /// Set n*n matrix A to identity matrix. */
/* static void somatic_la_ident( double *A, size_t n ) AA_DEPRECATED; */
/* static void somatic_la_ident( double *A, size_t n ) { */
/*     somatic_la_ident_v( A, n, 1 ); */
/* } */

/* /// invert column major m*n matrix A in place. */
/* int somatic_la_invert( size_t m, size_t n, double *A ) AA_DEPRECATED; */

/* /\** r = a - b *\/ */
/* static inline void somatic_la_vec_sub( double *r, const double *a, const double *b, */
/*                                        size_t n ) AA_DEPRECATED; */
/* static inline void somatic_la_vec_sub( double *r, const double *a, const double *b, */
/*                                        size_t n ) { */
/*     somatic_realcpy( r, a, n ); // r := a */
/*     cblas_daxpy( (int)n, -1, b, 1, r, 1 ); // r := -1*b + r */
/* } */
/* /\** y = alpha A x. */
/*  A is column major. *\/ */
/* static inline void somatic_la_gemv1( double *y, double alpha, */
/*                                      const double *A, const double *x, */
/*                                      size_t n_y, size_t n_x ) AA_DEPRECATED; */
/* static inline void somatic_la_gemv1( double *y, double alpha, */
/*                                      const double *A, const double *x, */
/*                                      size_t n_y, size_t n_x ) { */
/*     cblas_dgemv( CblasColMajor, CblasNoTrans, (int)n_y, (int)n_x, */
/*                  alpha, A, (int)n_y, */
/*                  x, 1, */
/*                  0, y, 1  ); */
/* } */

/*-------------*/
/* ACH helpers */
/*-------------*/

/* // Open an ach channel or die trying */
/* ach_channel_t* somatic_open_channel(const char *name) AA_DEPRECATED; */

/* // Create an ach channel or die trying */
/* int somatic_create_channel(const char *name, size_t frame_cnt, size_t frame_size) AA_DEPRECATED; */

/* // Close an ach channel or die trying */
/* int somatic_close_channel(ach_channel_t *chan) AA_DEPRECATED; */


/*---------*/
/* Signals */
/*---------*/

/// variable set when signal INT or TERM received by somatic's simple sighandler
extern int somatic_sig_received;
/// variable to control motor state in a process. Use with motor commands.
extern int somatic_motor_state;

/** installs handler for sigint and sigterm that sets
 * somatic_sig_received to 1.
 *
 * You should use this so that the main loop of your program can be a
 * `while(!somatic_sig_received) {}' and thus terminate cleanly on a
 * signal.
 */
void somatic_sighandler_simple_install();

/** unused, deprecated!
 *
 * function sends an alive signal to the server (watchdog) program.
 * Process with id pid will get a SIGUSR1 signal. By default, the server wants
 * senders pid in sigval info.
 */
void somatic_sighandler_send_alive(pid_t pid, sigval_t sigval);

/** blocks till it can find the watchdog program running
 *
 * This should be used just after the somatic_sighandler_simple_install() function.
 * It will block the program till the watchdog starts. This can be integrated into
 * somatic_sighandler_simple_install, but not all daemons will wait on watchdog
 */
void somatic_wait_copd();

/** function finds if the watchdog is running or not
 *
 * This is used internally within somatic_wait_copd
 */
int somatic_find_copd();

#ifdef __cplusplus
}
#endif

#endif // SOMATIC_UTIL_H
