/*
 * Cypress Semiconductor Proprietary and Confidential. © 2016 Cypress Semiconductor.  All rights reserved.
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Cypress Semiconductor;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Cypress Semiconductor.
 */

/** @file
 *
 * A2DP support for HCI AV Source application
 */
#ifndef A2DP_APP_H
#define A2DP_APP_H

#include "bt_types.h"
#include "wiced_bt_avdt.h"

#define CASE_RETURN_STR(const) case const: return #const;

/* A2DP Audio Route */
#define AUDIO_ROUTE_I2S             0x00
#define AUDIO_ROUTE_UART            0x01
#define AUDIO_ROUTE_SINE            0x02

/*A2DP Sampling Frequencies */
#define AUDIO_SF_16K                0x00
#define AUDIO_SF_32K                0x01
#define AUDIO_SF_44_1K              0x02
#define AUDIO_SF_48K                0x03

/* Channel Configurations */
#define AUDIO_CHCFG_MONO            0x00
#define AUDIO_CHCFG_STEREO          0x01

#define AV_CTRL_MTU                 L2CAP_MTU_SIZE
#define AV_DATA_MTU                 L2CAP_MTU_SIZE
#define AV_RET_TOUT                 4
#define AV_SIG_TOUT                 4
#define AV_IDLE_TOUT                10
#define AV_SEC_MASK                 BTM_SEC_NONE

/* offset of media type in codec info byte array */
#define AV_MEDIA_TYPE_IDX           1

#define  AV_NUM_SEPS                3

typedef enum
{
    AV_STATE_IDLE,              /* Initial state (channel is unused) */
    AV_STATE_CONFIGURED,        /* Remote has sent configuration request */
    AV_STATE_CONNECTED,         /* Signaling Channel is connected and active */
    AV_STATE_STARTED,			/* Data streaming */
} AV_STATE;

typedef enum
{
    AV_STREAM_STATE_STOPPED,
    AV_STREAM_STATE_STARTING,
    AV_STREAM_STATE_STARTED,
    AV_STREAM_STATE_STOPPING
} AV_STREAM_STATE;

/* Feature mask definitions */
#define AV_FEAT_CONTROL         0x0001  /* controller role */
#define AV_FEAT_TARGET          0x0002  /* target role */
#define AV_FEAT_INT             0x0004  /* initiator */
#define AV_FEAT_ACP             0x0008  /* acceptor */
#define AV_FEAT_VENDOR          0x0010  /* vendor dependent commands */
#define AV_FEAT_METADATA        0x0020  /* metadata Transfer */
typedef uint32_t  tAV_APP_FEAT;

#define AUDIO_IDLE_SUSPEND_TIMEOUT_IN_SECONDS 3

/* Entry in peer_cb table reserved for acceptor connection */
#define AV_IDX_ACP              0

/*  BIT pool calculation
    ********************
    bit_rate = 8*frame_length*fs(in kHz)/nrof_subbands/nrof_blocks,

    where,
    frame_length = 4+(4*nrof_subbands*nrof_channels)/8
                    +[nrof_blocks*nrof_channels*bitpool/8]
    for the MONO and DUAL_CHANNEL channel modes
    frame_length = 4+(4*nrof_subbands*nrof_channels)/8
                   +[(join*nrof_subbands + nrof_blocks*nrof_channels*bitpool)/8]
    for the STEREO a and JOIN_STEREO channel modes
    join = 1 when join stereo is used else 0

    for fs = 16kHz, nrof_subbands = 8, nrof_blocks = 16, nrof_channels = 1
    and channel mode = MONO
    => bit_rate = frame_length.
    => frame_length = 8+2*bitpool.
    Therefore, bitpool = (bit_rate-8)/2
    For bit_rate of 128kbps, bitpool = 60

    reference : A2DP spec v12
*/

#endif  /* A2DP_APP_H */
