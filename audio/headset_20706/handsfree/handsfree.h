/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * This file provides the private interface definitions for handsfree
 *
 */
#pragma once
#include "bt_types.h"
#include "wiced_bt_hfp_hf_int.h"

// SDP Record for Hands-Free Unit
#define HFP_MAX_SLC_CONNECTION                  2
#define HDLR_HANDS_FREE_UNIT                    0x10004
#define HANDS_FREE_SCN                          0x01
#define HANDS_FREE_SCN1                         0x02
#define HANDS_FREE_VOLUME_MIN                 1
#define HANDS_FREE_VOLUME_MAX                 15
#define HANDS_FREE_INVALID_SCO_INDEX              0xFFFF

#define WICED_HS_EIR_BUF_MAX_SIZE               264
#define KEY_INFO_POOL_BUFFER_SIZE               145 //Size of the buffer used for holding the peer device key info

#ifndef BTM_SCO_PKT_TYPES_MASK_HV1
#define BTM_INVALID_SCO_INDEX           0xFFFF
#define BTM_SCO_LINK_ALL_PKT_MASK       0x003F
#define BTM_SCO_LINK_ONLY_MASK          0x0007
#define BTM_SCO_PKT_TYPES_MASK_EV3      0x0008
#define BTM_SCO_PKT_TYPES_MASK_NO_2_EV3 0x0040
#define BTM_SCO_PKT_TYPES_MASK_NO_3_EV3 0x0080
#define BTM_SCO_PKT_TYPES_MASK_NO_2_EV5 0x0100
#define BTM_SCO_PKT_TYPES_MASK_NO_3_EV5 0x0200
#define BTM_ESCO_RETRANS_POWER          1
#define BTM_ESCO_RETRANS_QUALITY        2
#endif

#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
#define HANDS_FREE_SUPPORTED_FEATURES      ( WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                           WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                           WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                           WICED_BT_HFP_HF_FEATURE_HF_INDICATORS | \
                                           WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                           WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION)
#else
#define HANDS_FREE_SUPPORTED_FEATURES  ( WICED_BT_HFP_HF_FEATURE_3WAY_CALLING | \
                                           WICED_BT_HFP_HF_FEATURE_CLIP_CAPABILITY | \
                                           WICED_BT_HFP_HF_FEATURE_REMOTE_VOLUME_CONTROL| \
                                           WICED_BT_HFP_HF_FEATURE_VOICE_RECOGNITION_ACTIVATION | \
                                           WICED_BT_HFP_HF_FEATURE_HF_INDICATORS)

#endif

/** HF device indicators. */
typedef enum
{
    WICED_BT_HFP_HF_SERVICE_IND     =   1,
    WICED_BT_HFP_HF_CALL_IND        =   2,
    WICED_BT_HFP_HF_CALL_SETUP_IND  =   3,
    WICED_BT_HFP_HF_CALL_HELD_IND   =   4,
    WICED_BT_HFP_HF_SIGNAL_IND      =   5,
    WICED_BT_HFP_HF_ROAM_IND        =   6,
    WICED_BT_HFP_HF_BATTERY_IND     =   7
}wiced_bt_hfp_hf_indicator_t;

typedef struct
{
    wiced_bt_device_address_t               peer_bd_addr;
    wiced_bt_hfp_hf_connection_state_t      connection_status;
    int                                     call_active;
    int                                     call_held;
    wiced_bt_hfp_hf_callsetup_state_t       call_setup;
    wiced_bt_hfp_hf_inband_ring_state_t     inband_ring_status;
    uint8_t                                 mic_volume;
    uint8_t                                 spkr_volume;
    uint16_t                                sco_index;
    uint16_t                                rfcomm_handle;
    wiced_bool_t                            init_sco_conn;
} bluetooth_hfp_context_t;

extern bluetooth_hfp_context_t handsfree_ctxt_data[HFP_MAX_SLC_CONNECTION];


/* data associated with HF_OPEN_EVT */
typedef struct
{
    BD_ADDR             bd_addr;
    uint8_t             status;
} hci_control_hf_open_t;

/* data associated with AT command response event */
typedef struct
{
    uint16_t            num;
    char                str[WICED_BT_HFP_HF_MAX_AT_CMD_LEN];
} hci_control_hf_value_t;

/* data associated with HF_CONNECTED_EVT */
typedef struct
{
    uint32_t           peer_features;
} hci_control_hf_connect_t;

/* union of data associated with HS callback */
typedef union
{
    hci_control_hf_open_t    open;
    hci_control_hf_connect_t conn;
    hci_control_hf_value_t   val;
} hci_control_hf_event_t;
