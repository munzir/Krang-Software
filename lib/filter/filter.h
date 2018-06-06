/* -*- mode: C; c-basic-offset: 4  -*- */
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


#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
#define FILTER_RESTRICT 
#else
#define FILTER_RESTRICT  restrict
#endif

#ifdef __cplusplus
extern "C" {
#endif

/** \file filter.h
 * \author Neil T. Dantam
 */ 
/*-------------------------*/
/* Finite Impulse Response */
/*-------------------------*/

/// Finite Impulse Response Filter
typedef struct {
    double *X;     ///< state history
    double *b;     ///< gains
    size_t n;      ///< space
    size_t i;      ///< modular index into X
    size_t order;  ///< length of b
} filter_fir_t;

/// Finite Impulse Response filter a scalar value
double filter_fir( filter_fir_t *f, double x );

/// Finite Impulse Response filter n values
void filter_fir_n( double *y, filter_fir_t *f, double *x, size_t n );


/*--------*/
/* Kalman */
/*--------*/

/// kalman filter.
/// matrices are COLUMN-MAJOR
typedef struct {
    double *x;  ///< Mean state
    double *u;  ///< Input
    double *z;  ///< Measurement
    double *A;  ///< Process Model
    double *B;  ///< Input model
    double *C;  ///< Measurement model
    double *E;  ///< Covariance
    double *R;  ///< Process Noise Covariance
    double *Q;  ///< Measurement Noise Covariance
    size_t n_x; ///< size of x
    size_t n_u; ///< size of u
    size_t n_z; ///< size of z
} filter_kalman_t;

/// Allocate all the arrays
void filter_kalman_init( filter_kalman_t *kf, size_t n_x, size_t n_u, size_t n_z );

/// Free all the arrays
void filter_kalman_destroy( filter_kalman_t *kf );

/// Predict step
void filter_kalman_predict( filter_kalman_t *kf );

/// Predict step with euler approximation
void filter_kalman_predict_euler( filter_kalman_t *kf, double dt );

/// Correct step
void filter_kalman_correct( filter_kalman_t *kf );


/** Kalman filter with some strong assumptions.
 * - \f$n_x = n_z\f$
 * - A = C = I
 * - E, Q, R are diagonal
 *
 * This let's us run in \f$O(n_x + n_xn_u)\f$ instead of \f$O({n_x}^2
 * + n_xn_u + {n_z}^{2.4})\f$.  We avoid the \f${n_x}^2\f$ term by not
 * doing multiplication \f$Ax\f$ and we avoid the \f${n_z}^{2.4}\f$
 * term by computing the inverse of a diagonal matrix in \f$O(n_x)\f$
 * as simply the scalar inverse of each diagonal element.
 * 
 */
typedef struct {
    size_t n_x; ///< state space
    size_t n_u; ///< input space
    double *x;  ///< state estimate
    double *u;  ///< input
    double *z;  ///< measurement
    double *B;  ///< motion model
    double *E;  ///< variances
    double *Q;  ///< Measurement noise
    double *R;  ///< Process noise
} filter_kalman_simple_t;


/// Allocate all the arrays
void filter_kalman_simple_init( filter_kalman_simple_t *kf, size_t n_x, size_t n_u );

/// Free all the arrays
void filter_kalman_simple_destroy( filter_kalman_simple_t *kf );

/// Predict step
void filter_kalman_simple_predict( filter_kalman_simple_t *kf );

/// Correct step
void filter_kalman_simple_correct( filter_kalman_simple_t *kf );

/*-----------------*/
/* Particle Filter */
/*-----------------*/

// Warning! this is completely untested!


/// particle filter motion model function
typedef void (*filter_particle_motion_fun)( void *env, 
                                            size_t n_x, double *FILTER_RESTRICT x1, 
                                            const double *x0,
                                            size_t n_u, const double *u );

/// particle filter measurement model function
typedef double (*filter_particle_measure_fun)( void *env, 
                                               size_t n_x, const double *x, 
                                               size_t n_z, const double *z );
                                          
/// particle filter struct
typedef struct {
    size_t n_x;  ///< state space
    size_t n_u;  ///< input space
    size_t n_z;  ///< measurement space
    size_t n_p;  ///< particle count
    double *X;   ///< particles
    double *Xp;  ///< particles
    double *u;   ///< input
    double *z;   ///< measurement
    double *w;   ///< weights
    filter_particle_motion_fun motion;  ///< motion model
    filter_particle_measure_fun measure; ///< measurement model
} filter_particle_t;

/// init particle filter
void filter_particle_init( filter_particle_t *pf, 
                           size_t n_x, size_t n_u, size_t n_z, size_t n_p,
                           filter_particle_motion_fun motion,
                           filter_particle_measure_fun measure );
/// run particle filter
void filter_particle( filter_particle_t *pf, double rand,
                      void *motion_env, void *measure_env );

/// destroy particle filter
void filter_particle_destroy( filter_particle_t *pf );

#ifdef __cplusplus
}
#endif

#endif

