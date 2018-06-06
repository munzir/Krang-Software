/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
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
#include "somatic.h"
#include "somatic.pb-c.h"
#include "somatic/daemon.h"
#include "somatic/motor.h"

void somatic_motor_init(somatic_d_t *d, somatic_motor_t *m, size_t n,
                        const char *chan_cmd_name, const char *chan_state_name ) {
    if( SOMATIC_D_CHECK_PARM(d,  d && m && chan_state_name ) &&
        SOMATIC_D_CHECK_PARM(d,  n > 0 && n < SOMATIC_MOTOR_N_MAX ) ) {

        // allocate
        m->n = n;
        m->pos = AA_NEW0_AR( double, n );
        m->vel = AA_NEW0_AR( double, n );
        m->cur = AA_NEW0_AR( double, n );
        m->pos_offset = AA_NEW0_AR( double, n );
        m->pos_limit_max = AA_NEW0_AR( double, n );
        m->pos_limit_min = AA_NEW0_AR( double, n );
        m->vel_limit_max = AA_NEW0_AR( double, n );
        m->vel_limit_min = AA_NEW0_AR( double, n );
        m->cur_limit_max = AA_NEW0_AR( double, n );
        m->cur_limit_min = AA_NEW0_AR( double, n );
        m->pos_valid_max = AA_NEW0_AR( double, n );
        m->pos_valid_min = AA_NEW0_AR( double, n );
        m->vel_valid_max = AA_NEW0_AR( double, n );
        m->vel_valid_min = AA_NEW0_AR( double, n );
        m->cmd_msg = somatic_motor_cmd_alloc( n );

        // Open the state channel
        somatic_d_channel_open( d, &m->state_chan, chan_state_name, NULL );

				// Open the command channel if one is provided (may not be for waist)
				m->noCommands = (chan_cmd_name == NULL);
				if(!m->noCommands) somatic_d_channel_open(d, &m->cmd_chan, chan_cmd_name, NULL);
				
    } else {  // bad params
        somatic_d_die(d);
    }
}

void somatic_motor_destroy(somatic_d_t *d, somatic_motor_t *m) {
    if( SOMATIC_D_CHECK_PARM(d,  NULL != m) ) {
        aa_free_if_valid(m->pos);
        aa_free_if_valid(m->pos_offset);
        aa_free_if_valid(m->vel);
        aa_free_if_valid(m->pos_limit_max);
        aa_free_if_valid(m->pos_limit_min);
        aa_free_if_valid(m->vel_limit_max);
        aa_free_if_valid(m->vel_limit_min);
        aa_free_if_valid(m->cur_limit_max);
        aa_free_if_valid(m->cur_limit_min);
        aa_free_if_valid(m->pos_valid_max);
        aa_free_if_valid(m->pos_valid_min);
        aa_free_if_valid(m->vel_valid_max);
        aa_free_if_valid(m->vel_valid_min);
        if( m->cmd_msg) somatic_motor_cmd_free(m->cmd_msg);
    } else {  // bad params
        somatic_d_die(d);
    }
}


AA_API void somatic_motor_setvel( somatic_d_t *d, somatic_motor_t *m,
                                  double *x, size_t n ) {
    somatic_motor_cmd( d, m, SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY,
                       x, n, NULL );
}
AA_API void somatic_motor_setpos( somatic_d_t *d, somatic_motor_t *m,
                                  double *x, size_t n ) {
    somatic_motor_cmd( d, m, SOMATIC__MOTOR_PARAM__MOTOR_POSITION,
                       x, n, NULL );
}
AA_API void somatic_motor_halt( somatic_d_t *d, somatic_motor_t *m ) {
    somatic_motor_cmd( d, m, SOMATIC__MOTOR_PARAM__MOTOR_HALT, NULL, 0, NULL );
}
AA_API void somatic_motor_reset( somatic_d_t *d, somatic_motor_t *m ) {
    somatic_motor_cmd( d, m, SOMATIC__MOTOR_PARAM__MOTOR_RESET, NULL, 0, NULL );
}

/*
 * Function to be called for toggling auxillary digital outputs on AMC drives
 * \pre The port number on the device. For AMC Left: use 0-15. For Right: 16-31
 * \pre The value to be set. 1 for ACTIVE and 0 for INACTIVE.
 * Note that for AMC drives, these values go to virtual ports. And virtual ports 
 * ports have to be mapped to one of the available digital output ports on 
 * hardware, using windows software.
 */

AA_API void somatic_motor_digital_out( somatic_d_t *d, somatic_motor_t *m,
																			 size_t portNumber, bool value) {
		int64_t x[2];
		x[0]=(int64_t) portNumber;
		x[1]=(int64_t) value;
		somatic_motor_cmd( d, m, SOMATIC__MOTOR_PARAM__MOTOR_DIGITAL_OUT, NULL, 0 , x);
}


void somatic_motor_cmd( somatic_d_t *d, somatic_motor_t *m,
                        Somatic__MotorParam cmd_type,
                        double *x, size_t n,
												int64_t *ix) {

		// Check if this somatic motor group allows commands to be sent
		if(m->noCommands) return;

    //FIXME: check limits
    if( // valid context
        SOMATIC_D_CHECK_PARM(d, NULL != m) &&
        // valid cmd_type
        SOMATIC_D_CHECK_PARM(d,
                             SOMATIC__MOTOR_PARAM__MOTOR_CURRENT == cmd_type  ||
                             SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY == cmd_type ||
                             SOMATIC__MOTOR_PARAM__MOTOR_POSITION == cmd_type ||
                             SOMATIC__MOTOR_PARAM__MOTOR_HALT == cmd_type  ||
                             SOMATIC__MOTOR_PARAM__MOTOR_RESET == cmd_type ||
														 SOMATIC__MOTOR_PARAM__MOTOR_DIGITAL_OUT == cmd_type) &&
        // valid array
        SOMATIC_D_CHECK_PARM(d,
                             (SOMATIC__MOTOR_PARAM__MOTOR_CURRENT == cmd_type  ||
                              SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY == cmd_type ||
                              SOMATIC__MOTOR_PARAM__MOTOR_POSITION == cmd_type )
                             ? (x && m->n == n) : 1 ) &&
				// validate the digital output command data values. ix[0] is port number, ix[1] is value
				SOMATIC_D_CHECK_PARM(d,
														 (SOMATIC__MOTOR_PARAM__MOTOR_DIGITAL_OUT == cmd_type )
														 ? ( ix && ix[0]>=0 && ix[0]<256 && ix[1]>=0 && ix[1]<2) : 1 )
        ) {
        if( SOMATIC__MOTOR_PARAM__MOTOR_CURRENT == cmd_type  ||
            SOMATIC__MOTOR_PARAM__MOTOR_VELOCITY == cmd_type ||
            SOMATIC__MOTOR_PARAM__MOTOR_POSITION == cmd_type ||
            SOMATIC__MOTOR_PARAM__MOTOR_HALT == cmd_type ) {
            // value message
            somatic_motor_cmd_set(m->cmd_msg, cmd_type, x, n, NULL);
        } 
				// If the command is a digital output ctrl then send the ix values
				else if(SOMATIC__MOTOR_PARAM__MOTOR_DIGITAL_OUT == cmd_type ) {
						somatic_motor_cmd_set(m->cmd_msg, cmd_type, NULL, 0, ix);
				}
				else {
            // halt/reset message
            Somatic__Vector *vals = m->cmd_msg->values;
            m->cmd_msg->values = NULL;
            m->cmd_msg->param = cmd_type;
            m->cmd_msg->has_param = 1;
            m->cmd_msg->values = vals;
        }
        ach_status_t rach = SOMATIC_PACK_SEND( &m->cmd_chan, somatic__motor_cmd, m->cmd_msg );
        somatic_d_check( d, SOMATIC__EVENT__PRIORITIES__CRIT,
                         SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT,
                         ACH_OK == rach, "somatic_motor_cmd",
                         "ach result: %s", ach_result_to_string(rach) );
    }
}

void somatic_motor_update( somatic_d_t *d, somatic_motor_t *m ) {
    /// FIXME: store timestamps
	ach_status_t rach;
    if( SOMATIC_D_CHECK_PARM(d, NULL != m) &&
        SOMATIC_D_CHECK_PARM(d, m->pos && m->vel && m->cur) ) {

        Somatic__MotorState *state =
            SOMATIC_GET_LAST_UNPACK( rach, somatic__motor_state,
                                     &protobuf_c_system_allocator,
                                     1024 + 8*m->n, &m->state_chan );

        somatic_d_check( d, SOMATIC__EVENT__PRIORITIES__CRIT,
                         SOMATIC__EVENT__CODES__COMM_FAILED_TRANSPORT,
                         ACH_OK == rach || ACH_MISSED_FRAME == rach ||
                         ACH_STALE_FRAMES == rach,
                         "somatic_motor_update",
                         "ach result: %s", ach_result_to_string(rach) );

        if( (ACH_OK == rach || ACH_MISSED_FRAME == rach)  && state) {
            if( state->position &&
                somatic_d_check_msg( d, NULL != state->position->data,
                                     "motor_state", "no position->data" ) &&
                somatic_d_check_msg( d, state->position->n_data == m->n,
                                     "motor_state",
                                     "pos len, got %d, wanted %d",
                                     state->position->n_data, m->n)
                ) {
                double tmp[m->n];
                aa_la_vadd( m->n, state->position->data, m->pos_offset, tmp);
                if( somatic_d_check_msg_v(d, "motor_state-pos",
                                          tmp, sizeof(tmp)/sizeof(double),
                                          m->pos_valid_min, m->pos_valid_max,
                                          m->n) ) {
                    aa_fcpy( m->pos, tmp, m->n );
                }
            }
            if( state->velocity &&
                somatic_d_check_msg( d, NULL != state->velocity->data,
                                     "motor_state", "no velocity->data" ) &&
                somatic_d_check_msg( d, state->velocity->n_data == m->n,
                                     "motor_state-vel",
                                     "vel len, got %d, wanted %d",
                                     state->velocity->n_data, m->n) &&
                somatic_d_check_msg_v(d, "motor_state",
                                      state->velocity->data,
                                      state->velocity->n_data,
                                      m->vel_valid_min, m->vel_valid_max, m->n)
                ) {
                aa_fcpy( m->vel, state->velocity->data, m->n );
            }
            if( state->current &&
                somatic_d_check_msg( d, NULL != state->current->data,
                                     "motor_state", "no velocity->data" ) &&
                somatic_d_check_msg( d, state->current->n_data == m->n,
                                     "motor_state-cur",
                                     "cure len, got %d, wanted %d",
                                     state->velocity->n_data, m->n) 
                ) {
                aa_fcpy( m->cur, state->current->data, m->n );
            }


            somatic__motor_state__free_unpacked(state,
                                                &protobuf_c_system_allocator);
        }
    }
}
