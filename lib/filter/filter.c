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


#include <amino.h>
#include "filter.h"

/*-------------------------*/
/* Finite Impulse Response */
/*-------------------------*/
double filter_fir( filter_fir_t *f, double x ) {
    f->X[ f->i ] = x;
    double y = 0;
    for( size_t i = 0; i < f->n; i ++ ) {
        size_t j = (i+f->i) % f->n;
        assert( j < f->n );
        y += f->b[i] * f->X[j];
    }
    f->i = (f->i+1) % f->n;
    return y;
}

void filter_fir_n( double *y, filter_fir_t *f, double *x, size_t n ) {
    for( size_t i = 0; i < n; i ++ ) {
        y[i] = filter_fir( &f[i], x[i] );
    }
}



/*--------*/
/* Kalman */
/*--------*/

// fortran prototype
void filter_kalman_predict_( const int *n_x, const int *n_u,
                             double *x, const double *A,
                             const double *B, const double *u,
                             double *E, const double *R );
// fortran prototype
int filter_kalman_correct_( const int *n_x, const int *n_z,
                            double *x, const double *C,
                            const double *z, 
                            double *E, const double *Q);
                            
                           

void filter_kalman_init( filter_kalman_t *kf, size_t n_x, size_t n_u, size_t n_z ) {
    kf->n_x = n_x;
    kf->n_u = n_u;
    kf->n_z = n_z;

    kf->x = AA_NEW0_AR( double, n_x );
    kf->u = AA_NEW0_AR( double, n_u );
    kf->z = AA_NEW0_AR( double, n_z );

    kf->A = AA_NEW0_AR( double, n_x*n_x );
    kf->B = AA_NEW0_AR( double, n_x*n_u );
    kf->C = AA_NEW0_AR( double, n_z*n_x );

    kf->E = AA_NEW0_AR( double, n_x*n_x );
    kf->R = AA_NEW0_AR( double, n_x*n_x );
    kf->Q = AA_NEW0_AR( double, n_z*n_z );
}

void filter_kalman_destroy( filter_kalman_t *kf ) {
    free( kf->x );
    free( kf->u );
    free( kf->z );

    free( kf->A );
    free( kf->B );
    free( kf->C );

    free( kf->E );
    free( kf->R );
    free( kf->Q );
}

void filter_kalman_predict( filter_kalman_t *kf ) {
    int n_x = (int)kf->n_x;
    int n_u = (int)kf->n_u;
    filter_kalman_predict_( &n_x, &n_u,
                            kf->x, kf->A, kf->B, kf->u,
                            kf->E, kf->R );
}


void filter_kalman_predict_euler( filter_kalman_t *kf, double dt ) {
    int n_x = (int)kf->n_x;
    int n_u = (int)kf->n_u;
    double A_c[n_x*n_x];
    double B_c[n_x*n_u];
    double R_c[n_x*n_x];
   
    // A_c := (I + dt*A)
    aa_la_ident( kf->n_x, A_c );
    cblas_daxpy( n_x*n_x, dt, 
                 kf->A, 1,
                 A_c, 1 );

    // B_c := dt*B
    aa_fset( B_c, 0, kf->n_x*kf->n_u );
    cblas_daxpy( n_x*n_u, dt, 
                 kf->B, 1,
                 B_c, 1 );

    // R_c := dt^2 * R
    aa_fset( R_c, 0, kf->n_x*kf->n_x );
    cblas_daxpy( n_x*n_x, dt*dt, 
                 kf->R, 1,
                 R_c, 1 );

    filter_kalman_predict_( &n_x, &n_u,
                            kf->x, A_c, B_c, kf->u,
                            kf->E, R_c );
}

void filter_kalman_correct( filter_kalman_t *kf ) {
    int n_x = (int)kf->n_x;
    int n_z = (int)kf->n_z;
    //FIXME: should do something reasonable when 
    //       matrix inversion fails
    filter_kalman_correct_( &n_x, &n_z,
                            kf->x, kf->C, kf->z,
                            kf->E, kf->Q );
                  
}

/*---------------*/
/* Simple Kalman */
/*---------------*/

void filter_kalman_simple_init( filter_kalman_simple_t *kf, size_t n_x, size_t n_u ) {
    kf->n_x = n_x;
    kf->n_u = n_u;
    kf->x = AA_NEW0_AR( double, n_x );
    kf->z = AA_NEW0_AR( double, n_x );
    kf->u = AA_NEW0_AR( double, n_u );
    kf->B = AA_NEW0_AR( double, n_x*n_u );
    kf->E = AA_NEW0_AR( double, n_x );
    kf->Q = AA_NEW0_AR( double, n_x );
    kf->R = AA_NEW0_AR( double, n_x );
}

void filter_kalman_simple_destroy( filter_kalman_simple_t *kf ) {
    free( kf->x );
    free( kf->z );
    free( kf->u );
    free( kf->B );
    free( kf->E );
    free( kf->Q );
    free( kf->R );
}

void filter_kalman_simple_predict( filter_kalman_simple_t *kf ) {
    // x = Ix + Bx
    cblas_dgemv( CblasColMajor, CblasNoTrans,
                 (int)kf->n_u, (int)kf->n_x,
                 1.0, kf->B, (int)kf->n_x,
                 kf->u, 1,
                 1.0, kf->x, 1 );
    // E = I E I**T + R
    cblas_daxpy( (int)kf->n_x, 
                 1.0, kf->R, 1,
                 kf->E, 1 );
                
}

void filter_kalman_simple_correct( filter_kalman_simple_t *kf ) {
    for( size_t i = 0; i < kf->n_x; i ++ ) {
        double k = kf->E[i] / (kf->E[i] + kf->Q[i]);
        kf->x[i] += k * (kf->z[i] - kf->x[i]);
        kf->E[i] *= (1.0 - k);
    }
}

/*-----------------*/
/* Particle Filter */
/*-----------------*/

void filter_particle_init( filter_particle_t *pf, 
                           size_t n_x, size_t n_u, size_t n_z, size_t n_p,
                           filter_particle_motion_fun motion,
                           filter_particle_measure_fun measure ) {
    pf->n_x = n_x;
    pf->n_p = n_p;
    pf->n_u = n_u;

    pf->X = AA_NEW0_AR( double, n_x*n_p );
    pf->u = AA_NEW0_AR( double, n_u );
    pf->z = AA_NEW0_AR( double, n_z );
    pf->w = AA_NEW0_AR( double, n_x );

    pf->motion = motion;
    pf->measure = measure;
}

void filter_particle( filter_particle_t *pf, double r,
                      void *motion_env, void *measure_env ) {
    // run motion model
    for( size_t i = 0; i < pf->n_p; i ++ ) {
        pf->motion( motion_env, 
                    pf->n_x, &pf->Xp[i*pf->n_x],
                    &pf->X[i*pf->n_x], pf->n_u, pf->u );
                    
    }

    // find measurement probability
    size_t i_ml = 0;
    double p_ml = -1;
    for( size_t i = 0; i < pf->n_p; i ++ ) {
        double p = pf->measure( measure_env, 
                                pf->n_x, & pf->Xp[i*pf->n_x],
                                pf->n_z, pf->z );
        pf->w[i] = p;
        // find most likely estimate
        if( p > p_ml ) {
            p_ml = p;
            i_ml = i;
        }
    }

    // normalize weights: w /= sum(w)
    cblas_dscal( (int)pf->n_p, 
                 1.0 / cblas_dasum( (int)pf->n_p, pf->w, 1 ),
                 pf->w, 1 );

    // resample (low variance sampler)
    {
        double c = pf->w[0];
        size_t j = 0; // index into Xp
        for( size_t i = 0; i < pf->n_p; i ++ ) {
            // probablity point for the next sample
            double u = r + (double)i * 1.0 / (double)pf->n_p;
            // find the j in w corresponding to u
            while( u > c ) {
                j++;
                c += pf->w[j];
            }
            // copy the sample in Xp back to our particle buffer X
            aa_fcpy( &pf->X[ i*pf->n_x ], &pf->Xp[ j*pf->n_x ], pf->n_x );
        }
    }
}

void filter_particle_destroy( filter_particle_t *pf ) {
    free( pf->X );
    free( pf->u );
    free( pf->z );
    free( pf->w );
}
