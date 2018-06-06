/* -*- mode: C; c-basic-offset: 4; indent-tabs-mode: nil  -*- */
/* ex: set shiftwidth=4 expandtab: */
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

#include <stdio.h>
#include <stdarg.h>
#include <ntcan.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <amino.h>
#include "include/amccan.h"
#include "include/amcdrive.h"
#include "ntcanopen.h"

static uint16_t cob_base = 0x200;

#define eprintf(f, args...) fprintf(stderr, f, ## args)

#ifdef DEBUG
#define dprintf(f, args...) fprintf(stderr, f, ## args)
#else
#define dprintf(f, args...)
#endif

uint16_t uadd(uint16_t a, uint16_t b)
{
    assert(a + b < UINT16_MAX);
    return (uint16_t)(a + b);
}

uint16_t umult(uint16_t a, uint16_t b)
{
    assert(a * b < UINT16_MAX);
    return (uint16_t)(a * b);
}

static NTCAN_RESULT fail(NTCAN_RESULT result, const char *format, ...) {
    va_list ap;

    va_start(ap, format);
    vfprintf(stderr, format, ap);
    va_end(ap);

    return result != NTCAN_SUCCESS ? result : -1;
}

static NTCAN_RESULT try_ntcan(const char *op, NTCAN_RESULT ntr) {
    if (NTCAN_SUCCESS != ntr)
        return fail(ntr, "Failed %s: %s\n", op, canResultString(ntr));
    return ntr;
}

static NTCAN_RESULT try_ntcan_dl(const char *op, const uint8_t *rcmd, NTCAN_RESULT ntr) {
    if (NTCAN_SUCCESS != ntr)
        return fail(ntr, "Failed %s: %s\n", op, canResultString(ntr));

	// Checking the command byte of an SDO frame. If the client command specifier (ccs - the first 3 bits),
	// is set to 4, i.e. the server is aborting an SDO transfer, that's bad!
    if (rcmd && 0x80 == *rcmd)
        return fail(-1, "Bad SDO DL %s\n", op);
    return ntr;
}

static NTCAN_RESULT amcdrive_get_info(NTCAN_HANDLE handle, uint8_t id, servo_vars_t *drive_info) {
    NTCAN_RESULT status;
    uint8_t rcmd;

    status = try_ntcan_dl("read max current", &rcmd,
                          canOpenSDOWriteWait_ul_u16_AMC(handle, &rcmd, &drive_info->k_p, id,
                                                     AMCCAN_INDEX_BOARD_INFO, AMCCAN_SUBINDEX_MAX_PEAK_CURRENT,1));
    if (status != NTCAN_SUCCESS) 
        return status;

    uint32_t switchingFrequencyPBF;
    status = try_ntcan_dl("read switching frequency", &rcmd,
                          canOpenSDOWriteWait_ul_u32_AMC(handle, &rcmd, &switchingFrequencyPBF, id,
                                                     AMCCAN_INDEX_BOARD_INFO, AMCCAN_SUBINDEX_BOARD_SWITCH_FREQ,1));
    if (status != NTCAN_SUCCESS) 
        return status;

    drive_info->k_s = 1000*amcccan_decode_pbf(switchingFrequencyPBF);

    status = try_ntcan_dl("read feedback interpolation", &rcmd,
                          canOpenSDOWriteWait_ul_u16_AMC(handle, &rcmd, &drive_info->k_i, id,
                                                     AMCCAN_INDEX_FEEDBACK_PARM, AMCCAN_SUBINDEX_FEEDBACK_POS_INTERP,1));
    if (status != NTCAN_SUCCESS) 
        return status;

    return status;
}

static NTCAN_RESULT amcdrive_enable_pdos(NTCAN_HANDLE handle, uint8_t id, uint pdos, servo_vars_t *drive_info) {
    NTCAN_RESULT status;
    uint8_t rcmd;
    // What the heck is the 7 from? -ntd
    uint16_t cob_offset = umult(id, 7);

    drive_info->rpdo_position = uadd(cob_base, cob_offset);
    drive_info->rpdo_velocity = uadd(cob_base, uadd(cob_offset, 1));
    drive_info->rpdo_current  = uadd(cob_base, uadd(cob_offset, 2));

    drive_info->tpdo_position   = uadd(cob_base, uadd(cob_offset, 3));
    drive_info->tpdo_velocity   = uadd(cob_base, uadd(cob_offset, 4));
    drive_info->tpdo_current    = uadd(cob_base, uadd(cob_offset, 5));
    drive_info->tpdo_statusword = uadd(cob_base, uadd(cob_offset, 6));

    /*  eprintf(" %x\t%x\t%x\t%x\t%x\t%x\t%x\n",
        drive_info->rpdo_position, drive_info->rpdo_velocity, drive_info->rpdo_current,
        drive_info->tpdo_position, drive_info->tpdo_velocity, drive_info->tpdo_current,
        drive_info->tpdo_statusword);
    */

    // If they requested it, we assume they want it.
    canIdAdd(handle, drive_info->tpdo_position);
    canIdAdd(handle, drive_info->tpdo_velocity);
    canIdAdd(handle, drive_info->tpdo_current);
    canIdAdd(handle, drive_info->tpdo_statusword);

    /*
     * RPDO
     */
    dprintf("rpdo_position: %d\n", (pdos & ENABLE_RPDO_POSITION) == 0);
    status =
        try_ntcan_dl("rpdo_position", &rcmd,
                     amccan_dl_pdo_id(handle, &rcmd, id, AMCCAN_RPDO_3,
                                      drive_info->rpdo_position,
                                      (uint32_t)( (pdos & ENABLE_RPDO_POSITION) ==
                                                  0),
                                      0));
    if (status != NTCAN_SUCCESS)
        return status;

    dprintf("rpdo_velocity: %d\n", (pdos & ENABLE_RPDO_VELOCITY) == 0);
    status =
        try_ntcan_dl("rpdo_velocity", &rcmd,
                     amccan_dl_pdo_id(handle, &rcmd, id, AMCCAN_RPDO_4,
                                      drive_info->rpdo_velocity,
                                      (uint32_t)((pdos & ENABLE_RPDO_VELOCITY) == 0),
                                      0));
    if (status != NTCAN_SUCCESS)
        return status;

    dprintf("rpdo_current: %d\n", (pdos & ENABLE_RPDO_CURRENT) == 0);
    status =
        try_ntcan_dl("rpdo_current", &rcmd,
                     amccan_dl_pdo_id(handle, &rcmd, id, AMCCAN_RPDO_5,
                                      drive_info->rpdo_current,
                                      (uint32_t)((pdos & ENABLE_RPDO_CURRENT) == 0),
                                      0));
    if (status != NTCAN_SUCCESS)
        return status;

    /*
     * TPDO
     */
    dprintf("tpdo_statusword: %d\n", (pdos & REQUEST_TPDO_STATUSWORD) == 0);
    status =
        try_ntcan_dl("tpdo_current", &rcmd,
                     amccan_dl_pdo_id(handle, &rcmd, id, AMCCAN_TPDO_1,
                                      drive_info->tpdo_statusword,
                                      (uint32_t)((pdos & REQUEST_TPDO_STATUSWORD) == 0),
                                      0));
    if (status != NTCAN_SUCCESS)

        return status;

    dprintf("tpdo_position: %d\n", (pdos & REQUEST_TPDO_POSITION) == 0);
    status =
        try_ntcan_dl("tpdo_position", &rcmd,
                     amccan_dl_pdo_id(handle, &rcmd, id, AMCCAN_TPDO_3,
                                      drive_info->tpdo_position,
                                      (uint32_t)((pdos & REQUEST_TPDO_POSITION) == 0),
                                      0));
    if (status != NTCAN_SUCCESS)
        return status;

    dprintf("tpdo_velocity: %d\n", (pdos & REQUEST_TPDO_VELOCITY) == 0);
    status = try_ntcan_dl("tpdo_velocity", &rcmd,
                          amccan_dl_pdo_id(handle, &rcmd, id, AMCCAN_TPDO_4,
                                           drive_info->tpdo_velocity, (uint32_t)((pdos & REQUEST_TPDO_VELOCITY) == 0), 0));
    if (status != NTCAN_SUCCESS)
        return status;

    dprintf("tpdo_current: %d\n", (pdos & REQUEST_TPDO_CURRENT) == 0);
    status = try_ntcan_dl("tpdo_current", &rcmd,
                          amccan_dl_pdo_id(handle, &rcmd, id, AMCCAN_TPDO_5,
                                           drive_info->tpdo_current, (uint32_t)((pdos & REQUEST_TPDO_CURRENT) == 0), 0));
    if (status != NTCAN_SUCCESS)
        return status;

    return status;
}

NTCAN_RESULT amcdrive_enable_async_timer(NTCAN_HANDLE handle, uint8_t id, uint update_freq) {
    NTCAN_RESULT status;
    uint8_t rcmd;

    // Configure PDOs to function in ASYNC mode
    // RPDO
    status = try_ntcan_dl("rpdo_position_set", &rcmd,
                          amccan_dl_pdo_trans(handle, &rcmd, id,
                                              AMCCAN_RPDO_3, AMCCAN_PDO_TRANS_ASYNC, 0));
    if (status != NTCAN_SUCCESS)
        return status;

    status = try_ntcan_dl("rpdo_velocity_set", &rcmd,
                          amccan_dl_pdo_trans(handle, &rcmd, id,
                                              AMCCAN_RPDO_4, AMCCAN_PDO_TRANS_ASYNC, 0));
    if (status != NTCAN_SUCCESS)
        return status;

    status = try_ntcan_dl("rpdo_current_set", &rcmd,
                          amccan_dl_pdo_trans(handle, &rcmd, id,
                                              AMCCAN_RPDO_5, AMCCAN_PDO_TRANS_ASYNC, 0));
    if (status != NTCAN_SUCCESS)
        return status;

    // TPDO
    status = try_ntcan_dl("tpdo_statusword_transmission", &rcmd,
                          amccan_dl_pdo_trans(handle, &rcmd, id,
                                              AMCCAN_TPDO_1, AMCCAN_PDO_TRANS_ASYNC, 0));
    if (status != NTCAN_SUCCESS)
        return status;

    status = try_ntcan_dl("tpdo_position_transmission", &rcmd,
                          amccan_dl_pdo_trans(handle, &rcmd, id,
                                              AMCCAN_TPDO_3, AMCCAN_PDO_TRANS_ASYNC, 0));
    if (status != NTCAN_SUCCESS)
        return status;

    status = try_ntcan_dl("tpdo_velocity_transmission", &rcmd,
                          amccan_dl_pdo_trans(handle, &rcmd, id,
                                              AMCCAN_TPDO_4, AMCCAN_PDO_TRANS_ASYNC, 0));
    if (status != NTCAN_SUCCESS)
        return status;

    status = try_ntcan_dl("tpdo_current_transmission", &rcmd,
                          amccan_dl_pdo_trans(handle, &rcmd, id,
                                              AMCCAN_TPDO_5, AMCCAN_PDO_TRANS_ASYNC, 0));
    if (status != NTCAN_SUCCESS)
        return status;

    uint cycle_time = update_freq <= 1000 ? 1000 / update_freq : 1;
    uint32_t tpdos[] = {AMCCAN_TPDO_1 , AMCCAN_TPDO_3, AMCCAN_TPDO_4, AMCCAN_TPDO_5 };
    int tpdos_length = 4;    // length of tpdos[]
    status = try_ntcan_dl("enable_timer", &rcmd,
                          amccan_dl_timer1(handle, &rcmd, id, cycle_time, tpdos, tpdos_length));

    return status;
}

NTCAN_RESULT amcdrive_start(NTCAN_HANDLE handle, uint8_t id) {
    NTCAN_RESULT status;
    uint8_t rcmd;

    for( size_t i = 0; i < 3; i++ ) {
        // try this a few times since it doesn't always work...
        // Both of these reset any errors --
        // requires a transition on CW from 0->1
        status =
            try_ntcan_dl("control - shutdown", &rcmd,
                         amccan_dl_control(handle, &rcmd, id,
                                           AMCCAN_CONTROL_STATE_SHUTDOWN));
        if (status != NTCAN_SUCCESS)
            return status;

        status =
            try_ntcan_dl("control - reset error", &rcmd,
                         amccan_dl_control(handle, &rcmd, id,
                                           AMCCAN_CONTROL_STATE_RESET_FAULT));
        if (status != NTCAN_SUCCESS)
            return status;
    }


    // Now Start the Drive
    status = try_ntcan("NMT Start",
                       canOpenWriteNMT(handle, id,
                                       CANOPEN_NMT_START_REMOTE));
    if (status != NTCAN_SUCCESS)
        return status;


    status = try_ntcan_dl("control - shutdown", &rcmd,
                          amccan_dl_control(handle, &rcmd, id,
                                            AMCCAN_CONTROL_STATE_SHUTDOWN));
    if (status != NTCAN_SUCCESS)
        return status;

    status = try_ntcan_dl("control - switch on", &rcmd,
                          amccan_dl_control(handle, &rcmd, id,
                                            AMCCAN_CONTROL_STATE_SWITCH_ON));
    if (status != NTCAN_SUCCESS)
        return status;

    status = try_ntcan_dl("current mode", &rcmd,
                          amccan_dl_op_mode( handle, &rcmd, id,
                                             AMCCAN_OP_MODE_CURRENT));
    if (status != NTCAN_SUCCESS)
        return status;

    status =
        try_ntcan_dl("control - enable",&rcmd,
                     amccan_dl_control(handle, &rcmd, id,
                                       AMCCAN_CONTROL_STATE_ENABLE_OPERATION));
    if (status != NTCAN_SUCCESS)
        return status;

    return status;
}

NTCAN_RESULT amcdrive_stop_drive(servo_vars_t *drive) {
    NTCAN_RESULT status;
    uint8_t rcmd;

    status =
        try_ntcan_dl("control - shutdown", &rcmd,
                     amccan_dl_control(drive->handle, &rcmd,
                                       drive->canopen_id,
                                       AMCCAN_CONTROL_STATE_SHUTDOWN));
    return status;
}

NTCAN_RESULT amcdrive_start_drive(servo_vars_t *drive) {
    NTCAN_RESULT status;
    uint8_t rcmd;

    status =
        try_ntcan_dl("control - switch on", &rcmd,
                     amccan_dl_control(drive->handle, &rcmd,
                                       drive->canopen_id,
                                       AMCCAN_CONTROL_STATE_SWITCH_ON));
    if (status != NTCAN_SUCCESS)
        return status;

    status =
        try_ntcan_dl("current mode", &rcmd,
                     amccan_dl_op_mode( drive->handle, &rcmd,
                                        drive->canopen_id,
                                        AMCCAN_OP_MODE_CURRENT));
    if (status != NTCAN_SUCCESS)
        return status;

    status =
        try_ntcan_dl("control - enable",&rcmd,
                     amccan_dl_control(drive->handle, &rcmd,
                                       drive->canopen_id,
                                       AMCCAN_CONTROL_STATE_ENABLE_OPERATION));
    if (status != NTCAN_SUCCESS)
        return status;

    return status;
}

NTCAN_RESULT amcdrive_reset_drive(NTCAN_HANDLE handle, uint8_t identifier) {
    NTCAN_RESULT status;

    status = try_ntcan("reset",
                       canOpenWriteNMT(handle, identifier,
                                       CANOPEN_NMT_RESET_NODE));
    if (status != NTCAN_SUCCESS)
        return status;

    return status;
}

NTCAN_RESULT amcdrive_reset_drives(servo_vars_t *drives, size_t count) {
    NTCAN_RESULT status;

    uint i;
    for (i = 0; i < count; i++) {
        printf("resetting %d\n", i);
        status = amcdrive_reset_drive( drives[i].handle,
                                       drives[i].canopen_id );
        if (status != NTCAN_SUCCESS)
            return status;
    }

    return status;
}

NTCAN_RESULT amcdrive_init_drive( servo_vars_t *drive_info,
                                  uint pdos, uint update_freq ) {

    NTCAN_RESULT status;
    drive_info->current_sign = 1;
    uint8_t identifier = drive_info->canopen_id;
    NTCAN_HANDLE handle = drive_info->handle;


    // Put the drive into pre-operational state
    status = try_ntcan("pre-op",
                       canOpenWriteNMT(handle, identifier, CANOPEN_NMT_PRE_OP));
    if (status != NTCAN_SUCCESS) {
		fprintf(stderr, "pre-op failed\n");
        goto fail; 
	}

	// Yes, yes, goto is evil, but we don't have proper exceptions.
    // If you don't think this is what we should be doing, ask me
    // before changing. -- Jon Olson

    // Fetch motor controller constants (max current, switching frequency, etc)
    status = amcdrive_get_info(handle, identifier, drive_info);
    if (status != NTCAN_SUCCESS) {
        fprintf(stderr, "get info failed\n");
        goto fail;
    }

    // Enable process data objects:
    //   for RPDO, setting target position, velocity, current
    //   for TPDO, receiving actual position, velocity, current, status word
    status = amcdrive_enable_pdos(handle, identifier, pdos, drive_info);
    if (status != NTCAN_SUCCESS) {
        fprintf(stderr, "enable pdos failed\n");
        goto fail;
    }

    // Enable the asynchronous transmission of PDOs on a timed basis
    status = amcdrive_enable_async_timer(handle, identifier, update_freq);
    if (status != NTCAN_SUCCESS) {
        fprintf(stderr, "enable async timer failed\n");
        goto fail;
	}

    // Reset errors and go to start state
    status = amcdrive_start(handle, identifier);
    if (status != NTCAN_SUCCESS) {
        fprintf(stderr, "amcdrive start failed\n");
        goto fail;
	}

    return NTCAN_SUCCESS;

fail:
    // why do we close the handle, might want to reset other drives on bus? -ntd
    //if (handle != -1)
        //canClose(handle);

    // This error handling is incomplete. What we should really do is keep
    // track of the state of the drive as we initialize it and then have a
    // switch statement here with fallthrough to catch it in each state and
    // do any shutdown that is necessary from that state

    return status;
}

// ==============================================================================================
// Places the given identifiers into the canopen_id fields of the drive_infos
NTCAN_RESULT amcdrive_open_drives( int32_t network, uint8_t *identifiers,
                      uint count,
                      servo_vars_t *drive_infos) {
    // Open CAN device
    NTCAN_HANDLE handle;
    NTCAN_RESULT status;
    status = try_ntcan("canOpen",
                       canOpen(network,    //net
                               0,          //flags
                               10,         //txqueue
                               128,        //rxqueue
                               1000,       //txtimeout
                               2000,       //rxtimeout
                               &handle));   // handle

    // bind CAN MSG IDs
    for( size_t i = 0; i < count && NTCAN_SUCCESS == status; i ++ ) {
        drive_infos[i].handle = handle;
        drive_infos[i].canopen_id = identifiers[i];
        status = try_ntcan("add SDO response",
                           canOpenIdAddSDOResponse(handle, identifiers[i]));
    }
    // set baud
    if (status == NTCAN_SUCCESS) {
        status = try_ntcan("canSetBaudrate",
                           canSetBaudrate(handle, NTCAN_BAUD_1000));
    }
    // did it work?
    if( NTCAN_SUCCESS != status ) {
        canClose(handle);
    }

    return status;
}

static NTCAN_RESULT amcdrive_rpdo_cw_i32(NTCAN_HANDLE handle, uint16_t rpdo, int32_t value) {
    CMSG canMsg;

    canMsg.id = rpdo;
    canMsg.len = 4;

    uint16_t control_word = htocs(0x8f);
    value = htocs(value);
    // Configure control word
    memcpy(&canMsg.data[0], &control_word, sizeof(int16_t)); // Copy contol word
    memcpy(&canMsg.data[2], &value, sizeof(int32_t)); // Copy

    int count = 1;
    return try_ntcan("amcdrive_rpdo", canWrite(handle, &canMsg, &count, NULL));
}


double ticks_to_rad( servo_vars_t *s, double ticks ) {
    return (ticks * s->current_sign * 2 * M_PI) /
        (s->encoder_count * s->gear_ratio);
}

NTCAN_RESULT amcdrive_update_drives(servo_vars_t *drives, size_t count) {
    CMSG canMsgs[256]; // We can easily handle more than one message at a time

    int len = 256;
    int status = canRead(drives[0].handle, canMsgs, &len, NULL);

    if (status != NTCAN_SUCCESS)
        return status;
    // FIXME: there is a race condition here where we could miss
    //        multiple status word updates
    int m;
    for (m = 0; m < len; m++) {
        CMSG *canMsg = &canMsgs[m];
        // This '7' must be the same as appeared in amcdrive_enable_pdos()
        int32_t drive_id = (canMsg->id - 0x200) / 7;

        for (size_t i = 0; i < count; i++) {
            servo_vars_t *d = &drives[i];
            if (d->canopen_id == drive_id) {

                // ACTUAL POSITION
                if (d->tpdo_position == canMsg->id) {
                    int32_t pos0 = aa_endconv_ld_le_i32( &canMsg->data[2] );
                    d->act_pos = ticks_to_rad(d, pos0);
                }

                // ACTUAL VELOCITY
                else if (d->tpdo_velocity == canMsg->id) {
                    int32_t vel0 = aa_endconv_ld_le_i32( &canMsg->data[2] );
                    double ticks = amccan_decode_ds1(vel0, d->k_i, d->k_s);
                    d->act_vel = ticks_to_rad(d, ticks);
                }

                // ACTUAL CURRENT
                else if (d->tpdo_current == canMsg->id) {
                    int16_t cur0 = aa_endconv_ld_le_i16( &canMsg->data[2] );
                    d->act_cur = (d->current_sign) *
                        amccan_decode_dc1(cur0, d->k_p);
                }

                // STATUS WORD
                else if ( d->tpdo_statusword == canMsg->id) {
                    // This Status Word will be the same as reading from 6041h
                    d->prev_status = d->status;
                    d->status = aa_endconv_ld_le_i16( &canMsg->data[0] );
                    //memcpy(&d->status, &canMsg->data[0], sizeof(int16_t));
                    //printf("drive %x, status = %x\n", drive_id, d->status);
                }

                break;
            }
        }
    }
    return NTCAN_SUCCESS;
}

NTCAN_RESULT amcdrive_set_current(servo_vars_t *drive, double amps) {
    NTCAN_HANDLE handle = drive->handle;

    if (amps * 10 > drive->k_p)
        amps = drive->k_p / 10; // No, limit it.

    // AMC CANopen drives take current in a custom unit called DC2. This is
    // equal to amps times 2^15 / K_p (the peak current for the drive).
    float cm = (1 << 15) / (float)(drive->k_p / 10.0);
    int32_t i_dc2 = (int32_t)(amps * cm);
    i_dc2 *= drive->current_sign;
    dprintf("set_dc2_current: %04x\n", i_dc2);

    return try_ntcan("set_current",
                     amcdrive_rpdo_cw_i32(handle, drive->rpdo_current, i_dc2));
}

/*
 * Set the current measured position to zero
 */
NTCAN_RESULT amcdrive_reset_position( NTCAN_HANDLE h, uint8_t *rcmd, uint8_t node) {

    // TODO: add ability to reset the current measured position to any value 'pos'

    NTCAN_RESULT ntr;

    int32_t pos = 0;

    // Set Position Limit 2039.01h
    ntr = canOpenSDOWriteWait_dl_i32_AMC( h, rcmd, node, AMCCAN_INDEX_POS_LIMIT, AMCCAN_SUBINDEX_POS_LIMIT_MEASURED, pos,1);
    if( NTCAN_SUCCESS != ntr ) return ntr;

    int i= 0;
    for (i = 0; i < 10; i++)
    {// Kasemsit: I do loop 10 times to make sure the position is actually reset.

        // Set Digital Input Mask: Load Measured Position 2058.08h
        ntr = canOpenSDOWriteWait_dl_u16_AMC( h, rcmd, node, 0x2058, 0x08, 0x1,1);
        if( NTCAN_SUCCESS != ntr ) return ntr;

        // Reset Digital Input Mask: Load Measured Position 2058.08h
        ntr = canOpenSDOWriteWait_dl_u16_AMC( h, rcmd, node, 0x2058, 0x08, 0x0,1);
        if( NTCAN_SUCCESS != ntr ) return ntr;

        // Set Digital Input Mask: Load Target Position Command 2058.09h
        ntr = canOpenSDOWriteWait_dl_u16_AMC( h, rcmd, node, 0x2058, 0x09, 0x1,1);
        if( NTCAN_SUCCESS != ntr ) return ntr;

        // Reset Digital Input Mask: Load Target Position Command 2058.09h
        ntr = canOpenSDOWriteWait_dl_u16_AMC( h, rcmd, node, 0x2058, 0x09, 0x0,1);
        if( NTCAN_SUCCESS != ntr ) return ntr;
    }

    return 0;
}

void amcdrive_print_info(NTCAN_HANDLE handle, uint8_t id) {

    /* TODO: Print something....  */

    uint8_t rcmd;
    uint16_t k_p;

    // Print: Maximum Peak Current 20D8.0Ch
    canOpenSDOWriteWait_ul_u16_AMC(handle, &rcmd, &k_p, id, AMCCAN_INDEX_BOARD_INFO, AMCCAN_SUBINDEX_MAX_PEAK_CURRENT,1);
    printf("Maximum Peak Current (20D8.0Ch) = %u [PBC]\n", k_p);

}

void amcdrive_dump_status(FILE *f, int16_t statw) {
    fprintf(f, "Ready to Switch On:    %d\n",
            statw & AMCCAN_STATW_READY_ON );
    fprintf(f, "Switched On:           %d\n",
            statw & AMCCAN_STATW_SWITCHED_ON );
    fprintf(f, "Operation Enabled:     %d\n",
            statw & AMCCAN_STATW_OP_ENABLED);
    fprintf(f, "Fault:                 %d\n",
            statw & AMCCAN_STATW_FAULT);
    fprintf(f, "Voltage Enabled:       %d\n",
            statw & AMCCAN_STATW_VOLT_ENABLED);
    fprintf(f, "Quick Stop:            %d\n",
            statw & AMCCAN_STATW_QUICK_STOP);
    fprintf(f, "Switch On disabled:    %d\n",
            statw & AMCCAN_STATW_SW_ON_DISABLED);
    fprintf(f, "Warning:               %d\n",
            statw & AMCCAN_STATW_WARNING);
    fprintf(f, "Manufacture specific:  %d\n",
            statw & AMCCAN_STATW_MFCTR_SPECIFIC);
    fprintf(f, "Remote:                %d\n",
            statw & AMCCAN_STATW_REMOTE);
    fprintf(f, "Target Reached:        %d\n",
            statw & AMCCAN_STATW_TARGET_REACHED);
    fprintf(f, "Internal Limit Active: %d\n",
            statw & AMCCAN_STATW_INTERNAL_LIMIT);
    fprintf(f, "Homing Complete:       %d\n",
            statw & AMCCAN_STATW_HOMING_COMPLETE);

}
