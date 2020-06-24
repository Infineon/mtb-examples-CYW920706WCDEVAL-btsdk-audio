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
 * Runtime Bluetooth stack configuration parameters
 *
 */
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_a2dp_sink.h"
#include "hci_control.h"
#include "wiced_bt_audio.h"

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
#define WICED_DEVICE_NAME                       "Headset"
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    ( uint8_t* )WICED_DEVICE_NAME,                                /**< Local device name ( NULL terminated ) */
    {0x24, 0x04, 0x18},                                           /**< Local device class */
    ( BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT ),
    /**< Security requirements mask ( BTM_SEC_NONE, or combination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT ( see #wiced_bt_sec_level_e ) ) */
    /**< Security requirements mask ( BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT ( see #wiced_bt_sec_level_e ) ) */

    3,                                                              /**< Maximum number simultaneous links to different devices */

    /* BR/EDR scan config */
    {
        BTM_SCAN_TYPE_STANDARD,                                     /**< Inquiry scan type ( BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED ) */
        WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                 /**< Inquiry scan interval  ( 0 to use default ) */
        WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                   /**< Inquiry scan window ( 0 to use default ) */

        BTM_SCAN_TYPE_STANDARD,                                     /**< Page scan type ( BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED ) */
        WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                    /**< Page scan interval  ( 0 to use default ) */
        WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW                       /**< Page scan window ( 0 to use default ) */
    },

    /* BLE scan settings  */
    {
        BTM_BLE_SCAN_MODE_ACTIVE,                                   /**< BLE scan mode ( BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE ) */

        /* Advertisement scan configuration */
        96,                 /**< High duty scan interval */
        48,                 /**< High duty scan window */
        30,                                                         /**< High duty scan duration in seconds ( 0 for infinite ) */

        2048,                /**< Low duty scan interval  */
        48,                  /**< Low duty scan window */
        30,                                                         /**< Low duty scan duration in seconds ( 0 for infinite ) */

        /* Connection scan configuration */
        96,            /**< High duty cycle connection scan interval */
        48,            /**< High duty cycle connection scan window */
        30,                                                         /**< High duty cycle connection duration in seconds ( 0 for infinite ) */

        2048,           /**< Low duty cycle connection scan interval */
        48,             /**< Low duty cycle connection scan window */
        30,                                                         /**< Low duty cycle connection duration in seconds ( 0 for infinite ) */

        /* Connection configuration */
        WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                     /**< Minimum connection interval */
        WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                     /**< Maximum connection interval */
        WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    /* BLE advertisement settings */
    {
        BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map ( mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39 ) */
        BTM_BLE_ADVERT_CHNL_38 |
        BTM_BLE_ADVERT_CHNL_39,

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,            /**< High duty undirected connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,            /**< High duty undirected connectable maximum advertising interval */
        0,                                                         /**< High duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,             /**< Low duty undirected connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,             /**< Low duty undirected connectable maximum advertising interval */
        0,                                                         /**< Low duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        30,                                                         /**< Low duty directed connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        30,                                                         /**< High duty non-connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        0                                                           /**< Low duty non-connectable advertising duration in seconds ( 0 for infinite ) */
    },

    /* GATT configuration */
    {
        APPEARANCE_GENERIC_TAG,                                     /**< GATT appearance ( see gatt_appearance_e ) */
        3,                                                          /**< Client config: maximum number of servers that local client can connect to  */
        1,                                                          /**< Server config: maximum number of remote clients connections allowed by the local */
        512                                                         /**< Clinet config: maximum number of bytes that local client can reciver over LE link  */
    },

    /* RFCOMM configuration */
    {
        2,                                                          /**< Maximum number of simultaneous RFCOMM ports */
        2                                                           /**< Maximum number of simultaneous RFCOMM connections */
    },

    /* Application managed l2cap protocol configuration */
    {
        6,                                                          /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        7,                                                          /**< Maximum number of application-managed BR/EDR PSMs */
        12,                                                          /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        0,                                                          /**< Maximum number of application-managed LE PSMs */
        0,                                                          /**< Maximum number of application-managed LE channels */
    },

    /* Audio/Video Distribution configuration */
    {
        4,                                                          /**< Maximum simultaneous audio/video links */
    },

    /* Audio/Video Remote Control configuration */
    {
        1,                                                           /**< 1 if AVRC_CONN_ACCEPTOR is supported */
        4,                                                           /**< Maximum simultaneous remote control links */
    },
    5,
    517                                                             /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5 ) */
};


/*****************************************************************************
 * SDP database for the hci_control application
 ****************************************************************************/
// SDP Record handle for AVDT Sink
#define HANDLE_AVDT_SINK                        0x10001
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_TARGET                      0x10002
// SDP Record handle for AVRC TARGET
#define HANDLE_AVRC_CONTROLLER                  0x10003
// SDP Record handle for SPP

const uint8_t headset_sdp_db[] =
{

    SDP_ATTR_SEQUENCE_2( 275 ),      // A2DP Sink       ==> 77 + 2
                                     // AVRC Target     ==> 56 + 2
                                     // AVRC Controller ==> 59 + 2
                                     // Handsfree       ==> 75 + 2
                                     // Total = 275 ( 77 + 2 + 56 + 2 + 59 + 2 + 75 +2 )

    // SDP Record for A2DP Sink
    SDP_ATTR_SEQUENCE_1(77),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVDT_SINK),
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_AUDIO_SINK),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVDTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVDTP),
                SDP_ATTR_VALUE_UINT2(0x103),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_ADV_AUDIO_DISTRIBUTION),
                SDP_ATTR_VALUE_UINT2(0x103),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x000B),
        SDP_ATTR_SERVICE_NAME(16),
        'W', 'I', 'C', 'E', 'D', ' ', 'A', 'u', 'd', 'i', 'o', ' ', 'S', 'i', 'n', 'k',

    // SDP Record for AVRC Target
    SDP_ATTR_SEQUENCE_1(56),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_TARGET),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(3),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_TARGET),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x0104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_5),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_TG_CAT2),

    // SDP Record for AVRC Controller
    SDP_ATTR_SEQUENCE_1(59),
        SDP_ATTR_RECORD_HANDLE(HANDLE_AVRC_CONTROLLER),
        SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REM_CTRL_CONTROL),
        SDP_ATTR_ID(ATTR_ID_PROTOCOL_DESC_LIST), SDP_ATTR_SEQUENCE_1(16),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),
                SDP_ATTR_VALUE_UINT2(BT_PSM_AVCTP),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_PROTOCOL_AVCTP),
                SDP_ATTR_VALUE_UINT2(0x104),
        SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
            SDP_ATTR_SEQUENCE_1(6),
                SDP_ATTR_UUID16(UUID_SERVCLASS_AV_REMOTE_CONTROL),
                SDP_ATTR_VALUE_UINT2(AVRC_REV_1_3),
        SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, AVRC_SUPF_CT_CAT1),

        // SDP Record for Hands-Free Unit
            SDP_ATTR_SEQUENCE_1(75),
                SDP_ATTR_RECORD_HANDLE(HDLR_HANDS_FREE_UNIT),
                SDP_ATTR_ID(ATTR_ID_SERVICE_CLASS_ID_LIST), SDP_ATTR_SEQUENCE_1(6),
                    SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                    SDP_ATTR_UUID16(UUID_SERVCLASS_GENERIC_AUDIO),
                SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST(HANDS_FREE_SCN),
                SDP_ATTR_ID(ATTR_ID_BT_PROFILE_DESC_LIST), SDP_ATTR_SEQUENCE_1(8),
                    SDP_ATTR_SEQUENCE_1(6),
                        SDP_ATTR_UUID16(UUID_SERVCLASS_HF_HANDSFREE),
                        SDP_ATTR_VALUE_UINT2(0x0107),
                SDP_ATTR_SERVICE_NAME(15),
                    'W', 'I', 'C', 'E', 'D', ' ', 'H', 'F', ' ', 'D', 'E', 'V', 'I', 'C', 'E',
                SDP_ATTR_UINT2(ATTR_ID_SUPPORTED_FEATURES, 0x0016),
};


/*****************************************************************************
 * wiced_bt  buffer pool configuration
 *
 * Configure buffer pools used by the stack  according to application's requirement
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_app_cfg_buf_pools[] =
{
/*  { buf_size, buf_count } */
    { 64,       16  },      /* Small Buffer Pool */
    { 272,      6   },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 1056,     9   },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1056,     0   },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};

/*****************************************************************************
 *   codec and audio tuning configurations
 ****************************************************************************/
/*  Recommended max_bitpool for high quality audio */
#define BT_AUDIO_A2DP_SBC_MAX_BITPOOL   53

/* Array of decoder capabilities information. */
wiced_bt_a2dp_codec_info_t bt_audio_codec_capabilities[] =
{
    {
        .codec_id = WICED_BT_A2DP_CODEC_SBC,
        .cie =
        {
            .sbc =
            {
                (A2D_SBC_IE_SAMP_FREQ_44 | A2D_SBC_IE_SAMP_FREQ_48 | A2D_SBC_IE_SAMP_FREQ_32 | A2D_SBC_IE_SAMP_FREQ_16),    /* samp_freq */
                (A2D_SBC_IE_CH_MD_MONO   | A2D_SBC_IE_CH_MD_STEREO |
                 A2D_SBC_IE_CH_MD_JOINT  | A2D_SBC_IE_CH_MD_DUAL),      /* ch_mode */
                (A2D_SBC_IE_BLOCKS_16    | A2D_SBC_IE_BLOCKS_12 |
                 A2D_SBC_IE_BLOCKS_8     | A2D_SBC_IE_BLOCKS_4),        /* block_len */
                (A2D_SBC_IE_SUBBAND_4    | A2D_SBC_IE_SUBBAND_8),       /* num_subbands */
                (A2D_SBC_IE_ALLOC_MD_L   | A2D_SBC_IE_ALLOC_MD_S),      /* alloc_mthd */
                BT_AUDIO_A2DP_SBC_MAX_BITPOOL,                          /* max_bitpool for high quality audio */
                A2D_SBC_IE_MIN_BITPOOL                                  /* min_bitpool */
            }
        }
    }
};

/** A2DP sink configuration data */
wiced_bt_a2dp_config_data_t bt_audio_config =
{
    .feature_mask = WICED_BT_A2DP_SINK_FEAT_DELAY_RPT,          /* feature mask */
    .codec_capabilities =
    {
        .count = sizeof(bt_audio_codec_capabilities) / sizeof(bt_audio_codec_capabilities[0]),
        .info  = bt_audio_codec_capabilities,                   /* codec configuration */
    },
    .p_param =
    {
        .buf_depth_ms                   = 300,                                          /* in msec */
        .start_buf_depth                = 50,                                           /* start playback percentage of the buffer depth */
        .target_buf_depth               = 50,                                           /* target level percentage of the buffer depth */
        .overrun_control                = WICED_BT_A2DP_SINK_OVERRUN_CONTROL_FLUSH_DATA,/* overrun flow control flag */
        .adj_ppm_max                    = +300,                                         /* Max PPM adjustment value */
        .adj_ppm_min                    = -300,                                         /* Min PPM adjustment value */
        .adj_ppb_per_msec               = 200,                                          /* PPM adjustment per milli second */
        .lvl_correction_threshold_high  = +2000,                                        /* Level correction threshold high value */
        .lvl_correction_threshold_low   = -2000,                                        /* Level correction threshold low value */
        .adj_proportional_gain          = 20,                                           /* Proportional component of total PPM adjustment */
        .adj_integral_gain              = 2,                                            /* Integral component of total PPM adjustment */
    },
};

/**  Audio buffer configuration configuration */
const wiced_bt_audio_config_buffer_t wiced_bt_audio_buf_config = {
    .role                       =   WICED_AUDIO_SINK_ROLE | WICED_HF_ROLE,
#ifdef CHANGE_ROUTE_SUPPORT
    .audio_tx_buffer_size       =   6720,
#else
    .audio_tx_buffer_size       =   0,
#endif
    .audio_codec_buffer_size    =   0x5000
};

/*
 * wiced_app_cfg_sdp_record_get_size
 */
uint16_t wiced_app_cfg_sdp_record_get_size(void)
{
    return (uint16_t)sizeof(headset_sdp_db);
}

/*
 * wiced_app_cfg_buf_pools_get_num
 */
const wiced_bt_cfg_settings_t *wiced_app_cfg_get_settings(void)
{
    return &wiced_bt_cfg_settings;
}


/*
 * wiced_app_cfg_buf_pools_get_num
 */
int wiced_app_cfg_buf_pools_get_num(void)
{
    return (int)sizeof(wiced_app_cfg_buf_pools)/sizeof(wiced_app_cfg_buf_pools[0]);
}

/*
 * wiced_app_cfg_buf_pools_get
 */
const wiced_bt_cfg_buf_pool_t *wiced_app_cfg_buf_pools_get(void)
{
    return wiced_app_cfg_buf_pools;
}
