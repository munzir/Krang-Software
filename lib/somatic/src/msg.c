/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
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
#include <amino.h>
#include "somatic.pb-c.h"
#include "somatic/msg.h"


static void *pbregalloc_closure_alloc(void *cx, size_t size) {
    return aa_mem_region_alloc( (aa_mem_region_t*)cx, size);
}
static void pbregalloc_closure_free(void *cx, void *ptr) {
    (void)cx;
    (void)ptr;
}


AA_API void somatic_pbregalloc_set( somatic_pbregalloc_t *pba, aa_mem_region_t *reg ) {
    pba->allocator_data = reg;
    pba->alloc = pbregalloc_closure_alloc;
    pba->tmp_alloc = pbregalloc_closure_alloc;
    pba->free = pbregalloc_closure_free;
}

AA_API void somatic_pbregalloc_init( somatic_pbregalloc_t *pba, size_t n ) {
    memset(pba, 0, sizeof(*pba));
    aa_mem_region_t *reg = AA_NEW0(aa_mem_region_t);
    aa_mem_region_init(reg,n);
    somatic_pbregalloc_set(pba, reg);
}
AA_API void somatic_pbregalloc_destroy( somatic_pbregalloc_t *a ) {
    aa_mem_region_destroy( (aa_mem_region_t*) a->allocator_data );
    free( a->allocator_data );
}
AA_API void somatic_pbregalloc_release( somatic_pbregalloc_t *a ) {
    aa_mem_region_release( (aa_mem_region_t*) a->allocator_data );
}

#define VECTOR_FIELD_INIT( PB, FIELD, SIZE )              \
    if( NULL == (PB)->FIELD ) {                           \
        (PB)->FIELD = somatic_vector_alloc( (SIZE) );     \
    }                                                     \
    assert( (PB)->FIELD->n_data == SIZE );                \


#define VECTOR_FIELD_SET( PB, FIELD, SIZE, DATA )       \
    VECTOR_FIELD_INIT( PB, FIELD, SIZE )                \
    aa_fcpy( (PB)->FIELD->data, DATA, SIZE )

//=== Timespec ===

Somatic__Timespec *somatic_timespec_alloc() {
   Somatic__Timespec *pb = AA_NEW0( Somatic__Timespec );
   somatic__timespec__init( pb );
   return pb;
}
void somatic_timespec_free(Somatic__Timespec *pb) {
    aa_free_if_valid( pb );
}

//=== Vector ===
Somatic__Vector *somatic_vector_alloc(size_t size) {
    Somatic__Vector *pb = AA_NEW0( Somatic__Vector );
    somatic__vector__init( pb );
    pb->data = AA_NEW0_AR(double, size);
    pb->n_data = size;
    return pb;
}

void somatic_vector_free(Somatic__Vector *pb) {
    if( pb ) {
        aa_free_if_valid( pb->units );
        aa_free_if_valid( pb->data );
        free( pb );
    }
}
void somatic_vector_set_unit(Somatic__Vector *pb, Somatic__Unit unit) {
    pb->units = AA_NEW( Somatic__Unit );
    pb->units[0] = unit;
    pb->n_units = 1;
}

void somatic_vector_set_data(Somatic__Vector *pb, const double *x, size_t n) {
    assert( n <= pb->n_data );
    aa_fcpy( pb->data, x, n );
}


//=== IVector ===
Somatic__Ivector *somatic_ivector_alloc(size_t size) {
    Somatic__Ivector *pb = AA_NEW0( Somatic__Ivector );
    somatic__ivector__init( pb );
    pb->data = AA_NEW0_AR(int64_t, size);
    pb->n_data = size;
    return pb;
}

void somatic_ivector_free(Somatic__Ivector *pb) {
    if( pb ) {
        aa_free_if_valid( pb->data );
        free( pb );
    }
}

//=== Transform ===
Somatic__Transform *somatic_transform_alloc() {
    Somatic__Transform *pb = AA_NEW0(Somatic__Transform);
    somatic__transform__init(pb);
    pb->meta = somatic_metadata_alloc();
    pb->meta->type = SOMATIC__MSG_TYPE__TRANSFORM;
    pb->meta->has_type = 1;
    return pb;

}
void somatic_transform_free(Somatic__Transform *pb) {
    if( pb ) {
        somatic_vector_free( pb->translation );
        somatic_vector_free( pb->rotation );
        somatic_metadata_free( pb->meta );
        free(pb);
    }
}

void somatic_transform_set_quat( Somatic__Transform *pb, const double r[4] ) {
    VECTOR_FIELD_SET( pb, rotation, 4, r );
}

void somatic_transform_set_vec( Somatic__Transform *pb, const double r[3] ) {
    VECTOR_FIELD_SET( pb, translation, 3, r );
}

void somatic_transform_set_tf12( Somatic__Transform *pb, double r[12] ) {
    VECTOR_FIELD_SET( pb, translation, 3, r+9 );
    VECTOR_FIELD_INIT( pb, rotation, 4 );
    aa_tf_rotmat2quat( r, pb->rotation->data );

}
void somatic_transform_get_quat( Somatic__Transform *pb, double r[4] ) {
    assert( pb->rotation && pb->rotation->data && 4 == pb->rotation->n_data );
    aa_fcpy( r, pb->rotation->data, 4 );

}
void somatic_transform_get_vec( Somatic__Transform *pb, double r[3] ) {
    assert( pb->translation && pb->translation->data && 3 == pb->translation->n_data );
    aa_fcpy( r, pb->translation->data, 3 );

}
void somatic_transform_get_tf12( Somatic__Transform *pb, double r[12] ) {
    assert( pb->rotation && pb->rotation->data && 4 == pb->rotation->n_data );
    assert( pb->translation && pb->translation->data && 3 == pb->translation->n_data );
    aa_fcpy( r+9, pb->translation->data, 3 );
    aa_tf_quat2rotmat( pb->rotation->data, r );

}

//=== Metadata ===
Somatic__Metadata *somatic_metadata_alloc() {
    Somatic__Metadata *pb = AA_NEW0(Somatic__Metadata);
    somatic__metadata__init( pb );
    return pb;
}
void somatic_metadata_free( Somatic__Metadata *pb ) {
    if( pb ) {
        somatic_timespec_free( pb->time );
        somatic_timespec_free( pb->until );
        aa_free_if_valid( pb->label );
    }
}
void somatic_metadata_set_label( Somatic__Metadata *pb, const char *label ) {
    assert( NULL == pb->label );
    pb->label = strdup(label);
}
void somatic_metadata_set_time( Somatic__Metadata *pb, int64_t sec, int32_t nsec ) {
    if( NULL == pb->time ) {
        pb->time = somatic_timespec_alloc();
    }
    pb->time->sec = sec;
    pb->time->nsec = nsec;
    pb->time->has_nsec = nsec ? 1 : 0;
}

void somatic_metadata_set_time_timespec( Somatic__Metadata *pb, struct timespec ts ) {
    somatic_metadata_set_time( pb, ts.tv_sec, (int32_t) ts.tv_nsec );
}

void somatic_metadata_set_time_now( Somatic__Metadata *pb ) {
    somatic_metadata_set_time_timespec( pb, aa_tm_now() );
}

void somatic_metadata_set_until( Somatic__Metadata *pb, int64_t sec, int32_t nsec ) {
    if( NULL == pb->until ) {
        pb->until = somatic_timespec_alloc();
    }
    pb->until->sec = sec;
    pb->until->nsec = nsec;
    pb->until->has_nsec = 1;
}

void somatic_metadata_set_until_timespec( Somatic__Metadata *pb, struct timespec ts ) {
    somatic_metadata_set_until( pb, ts.tv_sec, (int32_t) ts.tv_nsec );
}

void somatic_metadata_set_until_duration( Somatic__Metadata *pb,
                                          double duration ) {
    struct timespec now;
    now.tv_sec = (time_t)pb->time->sec;
    now.tv_nsec = pb->time->has_nsec ?  pb->time->nsec : 0;
    struct timespec until = aa_tm_add(now,  aa_tm_sec2timespec( duration ) );
    somatic_metadata_set_until( pb, until.tv_sec, (int32_t) until.tv_nsec );
}

//=== Multi Transform ===
Somatic__MultiTransform *somatic_multi_transform_alloc(size_t n) {
    Somatic__MultiTransform *pb = AA_NEW0( Somatic__MultiTransform );
    somatic__multi_transform__init( pb );
    pb->tf = AA_NEW_AR( Somatic__Transform*, n );
    pb->n_tf = n;
    for( size_t i = 0; i < n; i ++ ) {
        pb->tf[i] = somatic_transform_alloc();
    }
    pb->meta = somatic_metadata_alloc();
    pb->meta->type = SOMATIC__MSG_TYPE__MULTI_TRANSFORM;
    pb->meta->has_type = 1;
    return pb;
}
void somatic_multi_transform_free(Somatic__MultiTransform *pb) {
    if( pb ) {
        somatic_transform_free( pb->origin );
        somatic_metadata_free( pb->meta );
        if( pb->tf ) {
            for( size_t i = 0; i < pb->n_tf; i++ ) {
                somatic_transform_free( pb->tf[i] );
            }
        }
    }
}

//=== Force Moment ===
Somatic__ForceMoment *somatic_force_moment_alloc( int alloc_force, int alloc_moment ) {
    Somatic__ForceMoment *pb = AA_NEW0(Somatic__ForceMoment);
    somatic__force_moment__init( pb );
    if( alloc_force ) pb->force = somatic_vector_alloc(3);
    if( alloc_moment ) pb->moment = somatic_vector_alloc(3);
    pb->meta = somatic_metadata_alloc();
    pb->meta->type = SOMATIC__MSG_TYPE__FORCE_MOMENT;
    pb->meta->has_type = 1;
    return pb;
}

void somatic_force_moment_free( Somatic__ForceMoment *pb ) {
    if( pb ) {
        somatic_vector_free( pb->force );
        somatic_vector_free( pb->moment );
        somatic_metadata_free( pb->meta );
    }
}

void somatic_force_moment_set( Somatic__ForceMoment *pb, const double v[6] ) {
    aa_fcpy( pb->force->data, v, 3 );
    aa_fcpy( pb->moment->data, v+3, 3 );
}
void somatic_force_moment_get( const Somatic__ForceMoment *pb, double v[6] ) {
    aa_fcpy( v, pb->force->data, 3 );
    aa_fcpy( v+3, pb->moment->data, 3 );
}

//=== Waist Cmd ===
Somatic__WaistCmd *somatic_waist_cmd_alloc( ) {
    Somatic__WaistCmd *pb = AA_NEW0(Somatic__WaistCmd);
    somatic__waist_cmd__init( pb );
    pb->data = somatic_vector_alloc ( 1 );
    pb->meta = somatic_metadata_alloc();
    pb->meta->type = SOMATIC__MSG_TYPE__WAIST_CMD;
    pb->meta->has_type = 1;
    return pb;
}

void somatic_waist_cmd_free( Somatic__WaistCmd *pb ) {
    if( pb ) {
        somatic_vector_free( pb->data );
        somatic_metadata_free( pb->meta );
        free(pb);
    }
}

void somatic_waist_cmd_set( Somatic__WaistCmd *pb,
                            const Somatic__WaistMode mode ) {
    pb->mode = mode;
		pb->has_mode = 1;
}

void somatic_waist_cmd_get( const Somatic__WaistCmd *pb,
                            Somatic__WaistMode* mode ) {
    *mode = pb->mode;
}


//=== Motor Cmd ===
Somatic__MotorCmd *somatic_motor_cmd_alloc( size_t n ) {
    Somatic__MotorCmd *pb = AA_NEW0(Somatic__MotorCmd);
    somatic__motor_cmd__init( pb );
    pb->values = somatic_vector_alloc( n );
    pb->meta = somatic_metadata_alloc();
    pb->meta->type = SOMATIC__MSG_TYPE__MOTOR_CMD;
    pb->meta->has_type = 1;
    return pb;
}
void somatic_motor_cmd_free( Somatic__MotorCmd *pb ) {
    if( pb ) {
        somatic_vector_free( pb->values );
        somatic_metadata_free( pb->meta );
        free(pb);
    }
}
void somatic_motor_cmd_set( Somatic__MotorCmd *pb,
                            Somatic__MotorParam param, const double *x, size_t n, int64_t *ix ) {
    if( NULL == pb->values ) {
        pb->values = somatic_vector_alloc( n );
    }

		// If ivalues are being set the first time, allocate 2 locations for ivalues. One for port number
		// and one for the value to be sent
		if( pb->ivalues == NULL && ix != NULL ) {
				pb->ivalues = somatic_ivector_alloc( 2 );
		}
    pb->param = param;
    pb->has_param = 1;
    if( x ) somatic_vector_set_data( pb->values, x, n );
		// If ix values are being sent, copy ix to pb->ivalues
		if( ix ) { pb->ivalues->data[0] = ix[0]; pb->ivalues->data[1] = ix[1]; }
}


//=== Battery ===
Somatic__Battery *somatic_battery_alloc( size_t n ) {
    Somatic__Battery *pb = AA_NEW0(Somatic__Battery);
    somatic__battery__init( pb );
    pb->voltage = somatic_vector_alloc( n );
    pb->temp = somatic_vector_alloc( n );

    pb->meta = somatic_metadata_alloc();
    pb->meta->type = SOMATIC__MSG_TYPE__BATTERY;
    pb->meta->has_type = 1;

    return pb;
}
void somatic_battery_free( Somatic__Battery *pb ) {
    if( pb ) {
        somatic_vector_free(pb->voltage);
        somatic_vector_free(pb->temp);
        somatic_metadata_free(pb->meta);
        free(pb);
    }
}

//=== Motor State ===
Somatic__MotorState *somatic_motor_state_alloc() {
    Somatic__MotorState *pb = AA_NEW0(Somatic__MotorState);
    somatic__motor_state__init(pb);
    pb->meta = somatic_metadata_alloc();
    pb->meta->type = SOMATIC__MSG_TYPE__MOTOR_STATE;
    pb->meta->has_type = 1;
    return pb;
}
void somatic_motor_state_free(Somatic__MotorState *pb) {
    if( pb ) {
        somatic_vector_free( pb->position );
        somatic_vector_free( pb->velocity );
        somatic_vector_free( pb->acceleration );
        somatic_vector_free( pb->current );
        somatic_metadata_free( pb->meta );
        free(pb);
    }
}
void somatic_motor_state_set_position( Somatic__MotorState *pb,
                                       const double *x, size_t n ) {
    VECTOR_FIELD_SET( pb, position, n, x );
}
void somatic_motor_state_set_velocity( Somatic__MotorState *pb,
                                       const double *x, size_t n ) {
    VECTOR_FIELD_SET( pb, velocity, n, x );
}
void somatic_motor_state_set_acceleraton( Somatic__MotorState *pb,
                                          const double *x, size_t n ) {
    VECTOR_FIELD_SET( pb, acceleration, n, x );
}
void somatic_motor_state_set_current( Somatic__MotorState *pb,
                                      const double *x, size_t n ) {
    VECTOR_FIELD_SET( pb, current, n, x );
}

AA_API void somatic_motor_state_alloc_position( Somatic__MotorState *pb,
                                                size_t n ) {
    pb->position = somatic_vector_alloc(n);
}


AA_API void somatic_motor_state_alloc_velocity( Somatic__MotorState *pb,
                                                size_t n ) {
    pb->velocity = somatic_vector_alloc(n);
}
AA_API void somatic_motor_state_alloc_acceleraton( Somatic__MotorState *pb,
                                                   size_t n ) {
    pb->acceleration = somatic_vector_alloc(n);
}
AA_API void somatic_motor_state_alloc_current( Somatic__MotorState *pb,
                                               size_t n ) {
    pb->current = somatic_vector_alloc(n);
}

//=== Joystick ===
Somatic__Joystick *somatic_joystick_alloc(size_t n_axes, size_t n_buttons) {
    Somatic__Joystick *pb = AA_NEW0(Somatic__Joystick);
    somatic__joystick__init(pb);
    pb->meta = somatic_metadata_alloc();
    pb->meta->type = SOMATIC__MSG_TYPE__JOYSTICK;
    pb->meta->has_type = 1;
    pb->axes = somatic_vector_alloc(n_axes);
    pb->buttons = somatic_ivector_alloc(n_buttons);
    return pb;
}
void somatic_joystick_free(Somatic__Joystick *pb) {
    if(pb) {
        somatic_vector_free(pb->axes);
        somatic_ivector_free(pb->buttons);
        somatic_metadata_free(pb->meta);
        free(pb);
    }
}

//=== Liberty ===
Somatic__Liberty *somatic_liberty_alloc() { // size_t n_sensors
    Somatic__Liberty *pb = AA_NEW0(Somatic__Liberty);
    somatic__liberty__init(pb);
    pb->meta = somatic_metadata_alloc();
    pb->meta->type = SOMATIC__MSG_TYPE__LIBERTY;
    pb->meta->has_type = 1;
    pb->sensor1 = somatic_vector_alloc(7);
    pb->sensor2 = somatic_vector_alloc(7);
    pb->sensor3 = somatic_vector_alloc(7);
    pb->sensor4 = somatic_vector_alloc(7);
    pb->sensor5 = somatic_vector_alloc(7);
    pb->sensor6 = somatic_vector_alloc(7);
    pb->sensor7 = somatic_vector_alloc(7);
    pb->sensor8 = somatic_vector_alloc(7);
    return pb;
}
void somatic_liberty_free(Somatic__Liberty *pb) {
    if(pb) {
        somatic_vector_free(pb->sensor1);
	somatic_vector_free(pb->sensor2);
	somatic_vector_free(pb->sensor3);
	somatic_vector_free(pb->sensor4);
	somatic_vector_free(pb->sensor5);
	somatic_vector_free(pb->sensor6);
	somatic_vector_free(pb->sensor7);
	somatic_vector_free(pb->sensor8);
        somatic_metadata_free(pb->meta);
        free(pb);
    }
}

//=== Cinder ===
Somatic__Cinder *somatic_cinder_alloc() { 
    Somatic__Cinder *pb = AA_NEW0(Somatic__Cinder);
    somatic__cinder__init(pb);
    pb->meta = somatic_metadata_alloc();
    pb->meta->type = SOMATIC__MSG_TYPE__CINDER;
    pb->meta->has_type = 1;
    pb->hole = somatic_vector_alloc(3);
    pb->normal = somatic_vector_alloc(3);
    return pb;
}
void somatic_cinder_free(Somatic__Cinder *pb) {
    if(pb) {
        somatic_vector_free(pb->hole);
        somatic_vector_free(pb->normal);
        somatic_metadata_free(pb->meta);
        free(pb);
    }
}

Somatic__VisualizeData* somatic__visualize_data__alloc(size_t nvecs,
                                                       const size_t* vecsizes,
                                                       size_t ivecsize) {
	Somatic__VisualizeData* pb = AA_NEW0(Somatic__VisualizeData);
	somatic__visualize_data__init(pb);
	pb->vecs = AA_NEW_AR(Somatic__Vector*, nvecs);
	pb->n_vecs = nvecs;
	for(size_t i = 0; i < pb->n_vecs; i++) {
		pb->vecs[i] = somatic_vector_alloc(vecsizes[i]);
	}
	pb->bools = somatic_ivector_alloc(ivecsize);
	pb->meta = somatic_metadata_alloc();
	pb->meta->type = SOMATIC__MSG_TYPE__VISUALIZE_DATA;
	pb->meta->has_type = 1;
	return pb;
}

void somatic__visualize_data__free(Somatic__VisualizeData* pb) {
	if(pb) {
		for(size_t i = 0; i < pb->n_vecs; i++)
			somatic_vector_free(pb->vecs[i]);
		somatic_ivector_free(pb->bools);
		somatic_metadata_free(pb->meta);
		free(pb);
	}
}

//=== Event ===

const char *somatic_event_pri2str(Somatic__Event__Priorities pri) {
    switch(pri) {
    case SOMATIC__EVENT__PRIORITIES__EMERG: return "EMERG";
    case SOMATIC__EVENT__PRIORITIES__CRIT: return "CRIT";
    case SOMATIC__EVENT__PRIORITIES__ALERT: return "ALERT";
    case SOMATIC__EVENT__PRIORITIES__ERR: return "ERR";
    case SOMATIC__EVENT__PRIORITIES__WARNING: return "WARNING";
    case SOMATIC__EVENT__PRIORITIES__NOTICE: return "NOTICE";
    case SOMATIC__EVENT__PRIORITIES__INFO: return "INFO";
    case SOMATIC__EVENT__PRIORITIES__DEBUG: return "DEBUG";
    }
    return "unknown";
}

const char *somatic_event_code2str(Somatic__Event__Codes code) {
    switch(code) {
    case SOMATIC__EVENT__CODES__UNKNOWN: return "UNKNOWN";
    case SOMATIC__EVENT__CODES__PROC_HEARTBEAT: return "PROC_HEARTBEAT";
    case SOMATIC__EVENT__CODES__PROC_STARTING: return "PROC_STARTING";
    case SOMATIC__EVENT__CODES__PROC_RUNNING: return "PROC_RUNNING";
    case SOMATIC__EVENT__CODES__PROC_STOPPING: return "PROC_STOPPING";
    case SOMATIC__EVENT__CODES__PROC_HALTED: return "PROC_HALTED";
    case SOMATIC__EVENT__CODES__PROC_ERR: return "PROC_ERR";
    case SOMATIC__EVENT__CODES__PROC_FAILED: return "PROC_FAILED";
    case SOMATIC__EVENT__CODES__COMM_BAD_MSG: return "COMM_BAD_MSG";
    case SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT: return "COMM_FAILED_TRANSPORT";
    case SOMATIC__EVENT__CODES__COMM_TIMEOUT: return "COMM_TIMEOUT";
    case SOMATIC__EVENT__CODES__COMM_DEV: return "COMM_DEV";
    case SOMATIC__EVENT__CODES__LIMIT: return "LIMIT";
    case SOMATIC__EVENT__CODES__BAD_PARAM: return "BAD_PARAM";
    case SOMATIC__EVENT__CODES__LOGIC: return "LOGIC";
    case SOMATIC__EVENT__CODES__UI: return "UI";
    case SOMATIC__EVENT__CODES__DEV_ERR: return "DEV_ERR";
    case SOMATIC__EVENT__CODES__SYS_HALT: return "SYS_HALT";
    case SOMATIC__EVENT__CODES__BAD_ASSERT: return "BAD_ASSERT";
    case SOMATIC__EVENT__CODES__INSANE: return "INSANE";
    }
    return "unknown";
}
