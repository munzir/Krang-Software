/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2009-2011, Georgia Tech Research Corporation
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

#ifndef SOMATIC_H
#define SOMATIC_H

#include <amino.h>
#include <ach.h>
#include "somatic.pb-c.h"
#include "somatic/msg.h"
#include "somatic/util.h"

#ifdef __cplusplus
extern "C" {
#endif

//#include <genmsg.h>

/** \file somatic.h
 *  \author Neil T. Dantam
 */

/** \page main Overview
 *
 * Somatic provides facilities to develop robot-control software as a
 * set of communicating daemons.
 *
 * Each daemon runs as a separate unix process.  Interprocess
 * communication between daemons is implemented via message passing.
 * Messages are formatted using Google Protocol Buffers and
 * transported using Ach, a shared-memory circular buffer library.
 *
 * \section Legal
 *
 * \author Neil T. Dantam, Jon Scholz, Pushkar Kolhe
 * \author Developed at the Georgia Tech Humanoid Robotics Lab
 * \author Under Direction of Professor Mike Stilman
 *
 * \copyright
 * Copyright (c) 2009-2011, Georgia Tech Research Corporation.
 * All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   - Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   - Redistributions in binary form must reproduce the above
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
 */


/**
 * \param chan: ach channel to send over
 * \param type: protobuf message type string (i.e. somatic__vector,
 *  			NOT actual Somatic__Vector type)
 * \param msg: pointer to the protobuf message
 */
#define SOMATIC_PACK_SEND( chan, type, msg )                    \
    ({                                                          \
        size_t _somatic_private_n =                             \
            type ## __get_packed_size(msg);                     \
        uint8_t _somatic_private_buf[_somatic_private_n];       \
        type ## __pack( msg, &_somatic_private_buf[0] );        \
        ach_put( chan, _somatic_private_buf,                    \
                 _somatic_private_n );                          \
    })



/**
 * \param ret: ach return code
 * \param type: protobuf message type string (i.e. somatic__vector,
 *  			NOT actual Somatic__Vector type)
 * \param alloc: protobuf allocator (ie, &protobuf_c_system_allocator)
 * \param buf: buffer to store data in
 * \param size: size of buffer to give ach
 * \param chan: ach channel pointer
 */
#define SOMATIC_GET_LAST_UNPACK_BUF(ret, type, chan, buf, size, alloc)  \
    ({                                                                  \
        size_t _somatic_private_nread;                                  \
        ret = ach_get( (chan), (buf), (size),                           \
                            &_somatic_private_nread, NULL, ACH_O_LAST); \
        ( ACH_OK == (ret) || ACH_MISSED_FRAME == (ret) ) ?              \
            type ## __unpack( (alloc), _somatic_private_nread,          \
                              (buf) ) :                                 \
            NULL;                                                       \
    })

/**
 * \param ret: ach return code
 * \param type: protobuf message type string (i.e. somatic__vector,
 *  			NOT actual Somatic__Vector type)
 * \param alloc: protobuf allocator (ie, &protobuf_c_system_allocator)
 * \param size: size of buffer to give ach
 * \param chan: ach channel pointer
 */
#define SOMATIC_GET_LAST_UNPACK( ret, type, alloc, size, chan )         \
    ({                                                                  \
        uint8_t _somatic_private_buf[size];                             \
        size_t _somatic_private_nread;                                  \
        ret = ach_get( chan, _somatic_private_buf, size,                \
                            &_somatic_private_nread, NULL, ACH_O_LAST );\
        ( ACH_OK == ret || ACH_MISSED_FRAME == ret ) ?                  \
            type ## __unpack( alloc, _somatic_private_nread,            \
                              _somatic_private_buf ) :                  \
            NULL;                                                       \
    })


#define SOMATIC_DEC_GET_LAST_UNPACK( name, ftype, rtype )               \
    rtype *name( ach_channel_t *chan, int *ach_result, size_t msg_size, \
                 ProtobufCAllocator *alloc)



#define SOMATIC_DEF_GET_LAST_UNPACK( name, ftype, rtype )               \
    SOMATIC_DEC_GET_LAST_UNPACK( name, ftype, rtype ) {                 \
        return SOMATIC_GET_LAST_UNPACK( *ach_result, ftype, alloc, msg_size, chan ); \
    }

/**
 * \param ret: ach return code
 * \param type: protobuf message type string (i.e. somatic__vector,
 *  			NOT actual Somatic__Vector type)
 * \param alloc: protobuf allocator (ie, &protobuf_c_system_allocator)
 * \param size: size of buffer to give ach
 * \param chan: ach channel pointer
 * \param timeout: optional timespec
 */
#define SOMATIC_WAIT_LAST_UNPACK( ret, type, alloc, size, chan, abstime ) \
    ({                                                                  \
        uint8_t _somatic_private_buf[size];                             \
        size_t _somatic_private_nread;                                  \
        ret = ach_get( chan, _somatic_private_buf, size,                \
                             &_somatic_private_nread, abstime, ACH_O_WAIT); \
        ( ACH_OK == ret || ACH_MISSED_FRAME == ret ) ?                  \
            type ## __unpack( alloc, _somatic_private_nread,            \
                              _somatic_private_buf ) :                  \
            NULL;                                                       \
    })

#define SOMATIC_DEC_WAIT_LAST_UNPACK( name, ftype, rtype )              \
    rtype *name( ach_channel_t *chan, int *ach_result, size_t msg_size, \
                 struct timespec *abstime, ProtobufCAllocator *alloc)


#define SOMATIC_DEF_WAIT_LAST_UNPACK( name, ftype, rtype )              \
    SOMATIC_DEC_WAIT_LAST_UNPACK( name, ftype, rtype ) {                \
        return SOMATIC_WAIT_LAST_UNPACK( *ach_result, ftype, alloc, msg_size, chan, abstime ); \
    }


int somatic_beep( int fd, double freq, double dur );

/** A structure to define binary protocol data.
 */
typedef struct {
    const char *key;
    uint32_t value;
    const char *desc;
    int typecode;
} somatic_prototable_t;

const somatic_prototable_t *somatic_prototable_lookup_key( const somatic_prototable_t *table,
                                                     const char *key);
const somatic_prototable_t *somatic_prototable_lookup_value( const somatic_prototable_t *table,
                                                       uint32_t value);


int somatic_prototable_key2value( const somatic_prototable_t *table,
                                  const char *key,
                                  uint32_t *value);
const char* somatic_prototable_value2key( const somatic_prototable_t *table,
                                          uint32_t value);


#ifdef __cplusplus
} // extern C
#endif

#endif
