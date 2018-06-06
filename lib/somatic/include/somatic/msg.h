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

#ifndef SOMATIC_MSG_H
#define SOMATIC_MSG_H

#include "somatic.pb-c.h"
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif

// naming convention
// somatic_MSGNAME_alloc
// somatic_MSGNAME_free
// somatic_MSGNAME_set_FIELD
// somatic_MSGNAME_get_FIELD

//=== Allocator ===
typedef ProtobufCAllocator somatic_pbregalloc_t;

AA_API void somatic_pbregalloc_set( somatic_pbregalloc_t *pba, aa_mem_region_t *reg );
AA_API void somatic_pbregalloc_init( somatic_pbregalloc_t *a, size_t n );
AA_API void somatic_pbregalloc_release( somatic_pbregalloc_t *a );
AA_API void somatic_pbregalloc_destroy( somatic_pbregalloc_t *a );

//=== Timespec ===

Somatic__Timespec *somatic_timespec_alloc();
void somatic_timespec_free(Somatic__Timespec *pb);
static inline void somatic_timespec_set_s( Somatic__Timespec *pb, int64_t s ) {
    pb->sec = s;
}
static inline void somatic_timespec_set_ns( Somatic__Timespec *pb, int32_t ns ) {
    pb->nsec = ns;
    pb->has_nsec = 1;
}
static inline struct timespec somatic_timespec_get( Somatic__Timespec *pb ) {
    return aa_tm_make( (time_t) pb->sec, pb->has_nsec ? (long) pb->nsec : 0 );
}

//=== Vector ===
Somatic__Vector *somatic_vector_alloc(size_t size);
void somatic_vector_free(Somatic__Vector *pb);
void somatic_vector_set_unit(Somatic__Vector *pb, Somatic__Unit unit);
void somatic_vector_set_data(Somatic__Vector *pb, const double *x, size_t n);

//=== IVector ===
AA_API Somatic__Ivector *somatic_ivector_alloc(size_t size);
AA_API void somatic_ivector_free(Somatic__Ivector *pb);

//=== Transform ===
AA_API Somatic__Transform *somatic_transform_alloc();
AA_API void somatic_transform_free(Somatic__Transform *pb);
AA_API void somatic_transform_set_quat( Somatic__Transform *pb, const double r[4] );
AA_API void somatic_transform_set_vec( Somatic__Transform *pb, const double r[3] );
AA_API void somatic_transform_set_tf12( Somatic__Transform *pb, double r[12] );
AA_API void somatic_transform_get_quat( Somatic__Transform *pb, double r[4] );
AA_API void somatic_transform_get_vec( Somatic__Transform *pb, double r[3] );
AA_API void somatic_transform_get_tf12( Somatic__Transform *pb, double r[12] );

//=== Metadata ===
/// heap allocates metadata object
AA_API Somatic__Metadata *somatic_metadata_alloc();
/// frees heap allocated metadata object and all children
AA_API void somatic_metadata_free( Somatic__Metadata *pb );
/// strdups the label
AA_API void somatic_metadata_set_label( Somatic__Metadata *pb, const char *label );
/// heap allocates timespec for time field
AA_API void somatic_metadata_set_time( Somatic__Metadata *pb, int64_t sec, int32_t nsec );
/// heap allocates timespec for time field and sets to current time
AA_API void somatic_metadata_set_time_now( Somatic__Metadata *pb );
/// heap allocates timespec for time field and sets to timespec
AA_API void somatic_metadata_set_time_timespec( Somatic__Metadata *pb, struct timespec ts );
/// heap allocates timespec for until field
AA_API void somatic_metadata_set_until( Somatic__Metadata *pb, int64_t sec, int32_t nsec );
/// heap allocates timespec for until field
AA_API void somatic_metadata_set_until_timespec( Somatic__Metadata *pb, struct timespec ts);
/// heap allocates timespec for until field and sets to secs offset from pb->time
AA_API void somatic_metadata_set_until_duration( Somatic__Metadata *pb,
                                          double secs );

//=== Battery ===
/// heap allocate battery message
Somatic__Battery *somatic_battery_alloc( size_t n );
/// free heap allocated battery message
void somatic_battery_free( Somatic__Battery *pb );

//=== Multi Transform ===
AA_API Somatic__MultiTransform *somatic_multi_transform_alloc(size_t n);
AA_API void somatic_multi_transform_free(Somatic__MultiTransform *pb);

//=== Force Moment ===
AA_API Somatic__ForceMoment *somatic_force_moment_alloc( int alloc_force, int alloc_moment );
AA_API void somatic_force_moment_free( Somatic__ForceMoment *pb );
AA_API void somatic_force_moment_set( Somatic__ForceMoment *pb, const double[6] );
AA_API void somatic_force_moment_get( const Somatic__ForceMoment *pb, double[6] );

//=== Waist Cmd ===
AA_API Somatic__WaistCmd *somatic_waist_cmd_alloc( );
AA_API void somatic_waist_cmd_free( Somatic__WaistCmd *pb );
AA_API void somatic_waist_cmd_set( Somatic__WaistCmd *pb, const Somatic__WaistMode );
AA_API void somatic_waist_cmd_get( const Somatic__WaistCmd *pb, Somatic__WaistMode* );

//=== Motor Cmd ===
AA_API Somatic__MotorCmd *somatic_motor_cmd_alloc( size_t n );
AA_API void somatic_motor_cmd_free( Somatic__MotorCmd *pb );
AA_API void somatic_motor_cmd_set( Somatic__MotorCmd *pb,
                            Somatic__MotorParam param, const double *x, size_t n, int64_t* ix);

//=== Motor State ===
AA_API Somatic__MotorState *somatic_motor_state_alloc();
AA_API void somatic_motor_state_free(Somatic__MotorState *pb);
AA_API void somatic_motor_state_set_position( Somatic__MotorState *pb,
                                              const double *x, size_t n );
AA_API void somatic_motor_state_set_velocity( Somatic__MotorState *pb,
                                              const double *x, size_t n );
AA_API void somatic_motor_state_set_acceleraton( Somatic__MotorState *pb,
                                                 const double *x, size_t n );
AA_API void somatic_motor_state_set_current( Somatic__MotorState *pb,
                                             const double *x, size_t n );
AA_API void somatic_motor_state_alloc_position( Somatic__MotorState *pb,
                                                size_t n );
AA_API void somatic_motor_state_alloc_velocity( Somatic__MotorState *pb,
                                                size_t n );
AA_API void somatic_motor_state_alloc_acceleration( Somatic__MotorState *pb,
                                                    size_t n );
AA_API void somatic_motor_state_alloc_current( Somatic__MotorState *pb,
                                               size_t n );

//=== Joystick ===
AA_API Somatic__Joystick *somatic_joystick_alloc(size_t n_axes, size_t n_buttons);
AA_API void somatic_joystick_free(Somatic__Joystick *pb);

//=== Liberty ===
AA_API Somatic__Liberty *somatic_liberty_alloc(); //size_t n_sensors
AA_API void somatic_liberty_free(Somatic__Liberty *pb);

//=== Cinder ===
AA_API Somatic__Cinder *somatic_cinder_alloc(); //size_t n_sensors
AA_API void somatic_cinder_free(Somatic__Cinder *pb);

//=== VisualizeData ===
AA_API Somatic__VisualizeData* somatic__visualize_data__alloc(size_t nvecs,
                                                              const size_t* vecsizes,
                                                              size_t ivecsize);
AA_API void somatic__visualize_data__free(Somatic__VisualizeData* pb);

//=== Event ===
AA_API const char *somatic_event_code2str(Somatic__Event__Codes code);
AA_API const char *somatic_event_pri2str(Somatic__Event__Priorities pri);

#ifdef __cplusplus
} // extern C
#endif

#endif //SOMATIC_MSG_H
