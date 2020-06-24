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
 * Handsfree Plus Device Sample Application for 2070X/20739B1 devices.
 *
 * This file implements 2070x/20739B1 embedded application controlled over UART.
 * Current version of the application exposes Handsfree Device
 *
 * MCU connected over UART can send commands to execute certain functionality
 * while configuration is local in the application including SDP
 * databases, as well as configuration of different activities like inquiry
 * advertisements or scanning.
 *
 * To demonstrate the app, work through the following steps.
 * 1. Plug the WICED Bluetooth ( 2070x ) evaluation board into your computer
 * 2. Build and download the application ( to the 2070x board )
 * 3. Use ClientControl application to send various commands
 *
 * The sample app performs as a Bluetooth HF device
 *
 * The sample Windows ClientControl application is provided to show sample
 * MCU implementation on Windows platform.
 *
 * Features demonstrated
 *  - WICED BT Handsfree (Device) APIs
 *  - Handling of the UART WICED protocol
 *  - SDP configuration
 *  - Setting of the Local Bluetooth Device address from the host MCU
 *
 * On startup this demo:
 *  - Initializes the Bluetooth sub system
 *  - Receive NVRAM information from the host
 *
 * Application Instructions
 *  - Connect a PC terminal to the serial port of the WICED Eval board.
 *  - Start ClientControl application.
 *  - Select the COM port assigned to the WICED Eval board.
 *  - Download the application
 *
 * BR/EDR
 * - To find BR/EDR devices: Click on "Start BR/EDR Discovery"
 *
 * Handsfree Connection
 * - To create handsfree connection to remote AG device , choose the bluetooth
 *   address of the remote AG device from the BR/EDR combo box
 * - Click "Connect" button under Handsfree
 * - OR Put the device in discoverable and connectable mode and serach for the device from AG device and connect
 * - The following HF operations can be performed using the client control application
 *      Connect / Disconnect HF or SCO connection
 *      Answer / Hangup the call
 *      Dial / Redial the number
 *      Control Held call (ex. release all held calls, add held to conversion etc.)
 *      Mic / Speaker gain control
 */

#include "sparcommon.h"

#include "wiced_bt_cfg.h"
#include "wiced_memory.h"
#include "wiced_bt_sdp.h"
#include "wiced_transport.h"

#include "wiced_platform.h"
#include "wiced_bt_sco.h"
#include "handsfree.h"
#include "wiced_bt_dev.h"
#include "string.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#if defined(CYW20721B2) || defined(CYW43012C0)
#include "wiced_audio_manager.h"
#endif

#if defined(CYW20721B2) || defined(CYW43012C0)
static int32_t stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
static audio_config_t audio_config =
    {
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        .sr = AM_PLAYBACK_SR_16K,
#else
        .sr = AM_PLAYBACK_SR_8K,
#endif
       .channels = 1,
       .bits_per_sample = DEFAULT_BITSPSAM,
       .volume = AM_VOL_LEVEL_HIGH-2,
       .mic_gain = AM_VOL_LEVEL_HIGH-2,
       .sink = AM_HEADPHONES,
    };
#endif
static void hci_control_transport_status( wiced_transport_type_t type );
static void hfp_timer_expiry_handler( uint32_t param );

const wiced_transport_cfg_t  transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 2
    },
    .p_status_handler    = hci_control_transport_status,
    .p_data_handler      = hci_control_proc_rx_cmd,
    .p_tx_complete_cback = NULL
};

wiced_bt_sco_params_t handsfree_esco_params =
{
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        0x000D,             /* Latency: 13 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( T2 ) */
#else
        0x000C,             /* Latency: 12 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( S4 ) */
#endif
        HANDS_FREE_SCO_PKT_TYPES,
        BTM_ESCO_RETRANS_POWER, /* Retrans Effort ( At least one retrans, opt for power ) ( S4 ) */
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        WICED_TRUE
#else
        WICED_FALSE
#endif
};

#ifdef WICED_ENABLE_BT_HSP_PROFILE
wiced_bt_sco_params_t headset_sco_params =
{
        0x000A,             /* Latency: 10 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( S3 ) */
        HANDS_FREE_SCO_PKT_TYPES,
        BTM_ESCO_RETRANS_POWER, /* Retrans Effort ( At least one retrans, opt for power ) ( S3 ) */
        WICED_FALSE
};
#endif

bluetooth_hfp_context_t handsfree_ctxt_data;
handsfrees_app_globals handsfree_app_states;

void hci_control_send_hf_event(uint16_t evt, uint16_t handle, hci_control_hf_event_t *p_data)
{
    uint8_t   tx_buf[300];
    uint8_t  *p = tx_buf;
    int       i;

    WICED_BT_TRACE("[%u]hci_control_send_hf_event: Sending Event: %u  to UART\n", handle, evt);

    *p++ = (uint8_t)(handle);
    *p++ = (uint8_t)(handle >> 8);

    switch (evt)
    {
        case HCI_CONTROL_HF_EVENT_OPEN:                 /* HS connection opened or connection attempt failed  */
            for (i = 0; i < BD_ADDR_LEN; i++)
                *p++ = p_data->open.bd_addr[BD_ADDR_LEN - 1 - i];
            *p++ = p_data->open.status;
            break;

        case HCI_CONTROL_HF_EVENT_CLOSE:                /* HS connection closed */
            break;

        case HCI_CONTROL_HF_EVENT_AUDIO_OPEN:           /* Audio connection open */
            break;

        case HCI_CONTROL_HF_EVENT_AUDIO_CLOSE:          /* Audio connection closed */
            break;

        case HCI_CONTROL_HF_EVENT_CONNECTED:            /* HS Service Level Connection is UP */
            UINT32_TO_STREAM(p,p_data->conn.peer_features);
            break;

        case HCI_CONTROL_HF_EVENT_PROFILE_TYPE:
            UINT8_TO_STREAM(p,p_data->conn.profile_selected);
            break;
        default:                                        /* AT response */
            if (p_data)
            {
                *p++ = (uint8_t)(p_data->val.num);
                *p++ = (uint8_t)(p_data->val.num >> 8);
                utl_strcpy((char *)p, p_data->val.str);
                p += strlen(p_data->val.str) + 1;
            }
            else
            {
                *p++ = 0;               // val.num
                *p++ = 0;
                *p++ = 0;               // empty val.str
            }
            break;
    }
    wiced_transport_send_data(evt, tx_buf, (int)(p - tx_buf));
}

static void handsfree_connection_event_handler(wiced_bt_hfp_hf_event_data_t* p_data)
{
    wiced_bt_dev_status_t status;

    if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_CONNECTED)
    {
        hci_control_hf_open_t    open;
        wiced_bt_hfp_hf_scb_t *p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr (p_data->conn_data.remote_address);
        memcpy(open.bd_addr,p_data->conn_data.remote_address,BD_ADDR_LEN);
        open.status = WICED_BT_SUCCESS;
        handsfree_ctxt_data.rfcomm_handle = p_scb->rfcomm_handle;
        hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_OPEN, p_scb->rfcomm_handle, (hci_control_hf_event_t *) &open);

        if( p_data->conn_data.connected_profile == WICED_BT_HFP_PROFILE )
        {
            handsfree_app_states.connect.profile_selected = WICED_BT_HFP_PROFILE;
            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_PROFILE_TYPE, p_scb->rfcomm_handle, (hci_control_hf_event_t *) &handsfree_app_states.connect);
        }
        else
        {
            handsfree_app_states.connect.profile_selected = WICED_BT_HSP_PROFILE;
            memcpy( handsfree_ctxt_data.peer_bd_addr, p_data->conn_data.remote_address, sizeof(wiced_bt_device_address_t));
            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_PROFILE_TYPE, p_scb->rfcomm_handle, (hci_control_hf_event_t *) &handsfree_app_states.connect);
        }

        status = wiced_bt_sco_create_as_acceptor(&handsfree_ctxt_data.sco_index);
        WICED_BT_TRACE("%s: status [%d] SCO INDEX [%d] \n", __func__, status, handsfree_ctxt_data.sco_index);
    }
    else if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_SLC_CONNECTED)
    {
        WICED_BT_TRACE("%s: Peer BD Addr [%B]\n", __func__,p_data->conn_data.remote_address);

        memcpy( handsfree_ctxt_data.peer_bd_addr, p_data->conn_data.remote_address, sizeof(wiced_bt_device_address_t));
    }
    else if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_DISCONNECTED)
    {
        memset(handsfree_ctxt_data.peer_bd_addr, 0, sizeof(wiced_bt_device_address_t));
        if(handsfree_ctxt_data.sco_index != BT_AUDIO_INVALID_SCO_INDEX)
        {
            status = wiced_bt_sco_remove(handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.sco_index = BT_AUDIO_INVALID_SCO_INDEX;
            WICED_BT_TRACE("%s: remove sco status [%d] \n", __func__, status);
        }
        hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_CLOSE, handsfree_ctxt_data.rfcomm_handle, NULL);
    }
    UNUSED_VARIABLE(status);
}


static void handsfree_call_setup_event_handler(wiced_bt_hfp_hf_call_data_t* call_data)
{
    switch (call_data->setup_state)
    {
        case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING:
            WICED_BT_TRACE("%s: Call(incoming) setting-up\n", __func__);
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:
            if(call_data->active_call_present == 0)
            {
                if(handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING ||
                        handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING ||
                        handsfree_ctxt_data.call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING )
                {
                    WICED_BT_TRACE("Call: Inactive; Call Set-up: IDLE\n");
                    break;
                }
                /* If previous context has an active-call and active_call_present is 0 */
                if(handsfree_ctxt_data.call_active == 1)
                {
                    WICED_BT_TRACE("Call Terminated\n");
                    break;
                }
            }
            else if( call_data->active_call_present == 1)
            {
                WICED_BT_TRACE("Call: Active; Call-setup: DONE\n");
            }
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:
            WICED_BT_TRACE("Call(outgoing) setting-up\n");
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING:
            WICED_BT_TRACE("Remote(outgoing) ringing\n");
            break;

        default:
            break;
    }
    handsfree_ctxt_data.call_active = call_data->active_call_present;
    handsfree_ctxt_data.call_setup  = call_data->setup_state;
    handsfree_ctxt_data.call_held   = call_data->held_call_present;
}

static void handsfree_send_ciev_cmd (uint16_t handle, uint8_t ind_id,uint8_t ind_val,hci_control_hf_value_t *p_val)
{
    wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(handle);
    p_val->str[0] = '0'+ind_id;
    p_val->str[1] = ',';
    p_val->str[2] = '0'+ind_val;
    p_val->str[3] = '\0';
    hci_control_send_hf_event( HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CIEV, p_scb->rfcomm_handle, (hci_control_hf_event_t *)p_val );
}

static void handsfree_send_clcc_evt (uint16_t handle, wiced_bt_hfp_hf_active_call_t *active_call,hci_control_hf_value_t *p_val)
{
    wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(handle);
    int i = 0;

    p_val->str[i++] = '0'+active_call->idx;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->dir;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->status;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->mode;
    p_val->str[i++] = ',';
    p_val->str[i++] = '0'+active_call->is_conference;

    if(active_call->type)
    {
        p_val->str[i++] = ',';
        memcpy(&p_val->str[i],active_call->num,strlen(active_call->num));
        i +=  strlen(active_call->num);
        p_val->str[i++] = ',';
        i += utl_itoa (active_call->type,&p_val->str[i]);
    }
    p_val->str[i++] = '\0';
    hci_control_send_hf_event( HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CLCC, p_scb->rfcomm_handle, (hci_control_hf_event_t *)p_val );
}

static void handsfree_event_callback( wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data)
{
    hci_control_hf_event_t     p_val;
    int res = 0;

    memset(&p_val,0,sizeof(hci_control_hf_event_t));

    switch(event)
    {
        case WICED_BT_HFP_HF_CONNECTION_STATE_EVT:
            handsfree_connection_event_handler(p_data);
            break;

        case WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT:
            res = HCI_CONTROL_HF_EVENT_CONNECTED;
            p_val.conn.peer_features = p_data->ag_feature_flags;

            if(p_data->ag_feature_flags & WICED_BT_HFP_AG_FEATURE_INBAND_RING_TONE_CAPABILITY)
            {
                handsfree_ctxt_data.inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_ENABLED;
            }
            else
            {
                handsfree_ctxt_data.inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_DISABLED;
            }
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
            {
                wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->handle);
                if( (p_data->ag_feature_flags & WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION) &&
                        (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION) )
                {
                    handsfree_esco_params.use_wbs = WICED_TRUE;
                }
                else
                {
                    handsfree_esco_params.use_wbs = WICED_FALSE;
                }
            }
#endif
            break;

        case WICED_BT_HFP_HF_SERVICE_STATE_EVT:
            handsfree_send_ciev_cmd (p_data->handle,WICED_BT_HFP_HF_SERVICE_IND,p_data->service_state,&p_val.val);
            break;

        case WICED_BT_HFP_HF_CALL_SETUP_EVT:
        {
            if (handsfree_ctxt_data.call_active != p_data->call_data.active_call_present)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_IND,p_data->call_data.active_call_present,&p_val.val);

            if (handsfree_ctxt_data.call_held != p_data->call_data.held_call_present)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_HELD_IND,p_data->call_data.held_call_present,&p_val.val);

            if (handsfree_ctxt_data.call_setup != p_data->call_data.setup_state)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_SETUP_IND,p_data->call_data.setup_state,&p_val.val);

            handsfree_call_setup_event_handler(&p_data->call_data);
        }
            break;

        case WICED_BT_HFP_HF_RSSI_IND_EVT:
            handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_SIGNAL_IND,p_data->rssi,&p_val.val);
            break;

        case WICED_BT_HFP_HF_SERVICE_TYPE_EVT:
            handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_ROAM_IND,p_data->service_type,&p_val.val);
            break;

        case WICED_BT_HFP_HF_BATTERY_STATUS_IND_EVT:
            handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_BATTERY_IND,p_data->battery_level,&p_val.val);
            break;

        case WICED_BT_HFP_HF_RING_EVT:
            WICED_BT_TRACE("%s: RING \n", __func__);
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_RING;
            break;

        case WICED_BT_HFP_HF_INBAND_RING_STATE_EVT:
            handsfree_ctxt_data.inband_ring_status = p_data->inband_ring;
            break;

        case WICED_BT_HFP_HF_OK_EVT:
            WICED_BT_TRACE("%s: OK \n", __func__);
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_OK;
            break;

        case WICED_BT_HFP_HF_ERROR_EVT:
            WICED_BT_TRACE("%s: Error \n", __func__);
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_ERROR;
            break;

        case WICED_BT_HFP_HF_CME_ERROR_EVT:
            WICED_BT_TRACE("%s: CME Error \n", __func__);
            p_val.val.num = p_data->error_code;
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CMEE;
            break;

        case WICED_BT_HFP_HF_CLIP_IND_EVT:
            p_val.val.num = p_data->clip.type;
            strncpy( p_val.val.str, p_data->clip.caller_num, sizeof( p_val.val.str ) );
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CLIP;
            WICED_BT_TRACE("%s: CLIP - number %s, type %d\n", __func__, p_data->clip.caller_num, p_data->clip.type);
            break;

        case WICED_BT_HFP_HF_BINP_EVT:
            p_val.val.num = p_data->binp_data.type;
            strncpy( p_val.val.str, p_data->binp_data.caller_num, sizeof( p_val.val.str ) );
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BINP;
            WICED_BT_TRACE("%s: BINP - number %s, type %d\n", __func__, p_data->binp_data.caller_num, p_data->binp_data.type);
            break;

        case WICED_BT_HFP_HF_VOLUME_CHANGE_EVT:
            WICED_BT_TRACE("%s: %s VOLUME - %d \n", __func__, (p_data->volume.type == WICED_BT_HFP_HF_SPEAKER)?"SPK":"MIC",  p_data->volume.level);
            if (p_data->volume.type == WICED_BT_HFP_HF_MIC )
            {
                res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_VGM;
            }
            else
            {
                res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_VGS;
            }
            p_val.val.num = p_data->volume.level;
            break;

        case WICED_BT_HFP_HFP_CODEC_SET_EVT:
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BCS;
            if ( p_data->selected_codec == WICED_BT_HFP_HF_MSBC_CODEC )
                handsfree_esco_params.use_wbs = WICED_TRUE;
            else
                handsfree_esco_params.use_wbs = WICED_FALSE;
            p_val.val.num = p_data->selected_codec;


            if (handsfree_ctxt_data.init_sco_conn == WICED_TRUE)
            {
                /* timer started here to check if the sco has been created as an acceptor*/
                wiced_start_timer(&handsfree_app_states.hfp_timer,SCO_CONNECTION_WAIT_TIMEOUT);

                handsfree_ctxt_data.init_sco_conn = WICED_FALSE;
            }
#if defined(CYW20721B2) || defined(CYW43012C0)
            WICED_BT_TRACE("%s - CODEC_SET: %d\n", __func__, p_data->selected_codec);
            if ( p_data->selected_codec == WICED_BT_HFP_HF_MSBC_CODEC ) {
                handsfree_esco_params.use_wbs = WICED_TRUE;
                audio_config.sr = 16000;
            }
            else {
                handsfree_esco_params.use_wbs = WICED_FALSE;
                audio_config.sr = 8000;
            }

            audio_config.channels =  1;
            audio_config.bits_per_sample = DEFAULT_BITSPSAM;
            audio_config.volume = AM_VOL_LEVEL_HIGH-2;
            if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                stream_id = wiced_am_stream_open(HFP);
            }

            if( WICED_SUCCESS != wiced_am_stream_set_param(stream_id,AM_AUDIO_CONFIG, &audio_config))
                WICED_BT_TRACE("wiced_am_set_param failed\n");
#endif
            break;

        case WICED_BT_HFP_HFP_ACTIVE_CALL_EVT:
            handsfree_send_clcc_evt(p_data->handle,&p_data->active_call,&p_val.val);
            break;

        case WICED_BT_HFP_HF_CNUM_EVT:
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CNUM;
            memcpy(p_val.val.str, p_data->cnum_data, strlen(p_data->cnum_data));
            break;

        case WICED_BT_HFP_HF_BIND_EVT:
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BIND;
            p_val.val.str[0] = p_data->bind_data.ind_id + '0';
            p_val.val.str[1] = ',';
            p_val.val.str[2] = p_data->bind_data.ind_value + '0';
            break;

        default:
            break;
    }
    if ( res && (res <= (HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_MAX)) )
    {
        wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->handle);
        hci_control_send_hf_event( res, p_scb->rfcomm_handle, (hci_control_hf_event_t *)&p_val );
    }
}

void handsfree_init_context_data(void)
{
    handsfree_ctxt_data.call_active         = 0;
    handsfree_ctxt_data.call_held           = 0;
    handsfree_ctxt_data.call_setup          = WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE;
    handsfree_ctxt_data.connection_status   = WICED_BT_HFP_HF_STATE_DISCONNECTED;
    handsfree_ctxt_data.spkr_volume         = 8;
    handsfree_ctxt_data.mic_volume          = 8;
    handsfree_ctxt_data.sco_index           = BT_AUDIO_INVALID_SCO_INDEX;
    handsfree_ctxt_data.init_sco_conn       = WICED_FALSE;
}

wiced_bt_voice_path_setup_t handsfree_sco_path = {
#ifdef CYW20706A2
    .path = WICED_BT_SCO_OVER_I2SPCM,
#else
    .path = WICED_BT_SCO_OVER_PCM,
#endif
};

void handsfree_hfp_init(void)
{
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_hfp_hf_config_data_t config;

    handsfree_init_context_data();

    /* Perform the rfcomm init before hf and spp start up */
#if defined(CYW20719B2) || defined(CYW20721B2) || defined (CYW43012C0)
    result = wiced_bt_rfcomm_set_buffer_pool( 700, 4);
#else
    result = wiced_bt_rfcomm_init( 700, 4 );
#endif
    if( (wiced_bt_rfcomm_result_t)result != WICED_BT_RFCOMM_SUCCESS )
    {
        WICED_BT_TRACE("Error Initializing RFCOMM - HFP failed\n");
        return;
    }

    config.feature_mask     = BT_AUDIO_HFP_SUPPORTED_FEATURES;
    config.speaker_volume   = handsfree_ctxt_data.spkr_volume;
    config.mic_volume       = handsfree_ctxt_data.mic_volume;
#ifdef WICED_ENABLE_BT_HSP_PROFILE
    config.num_server       = 2;
#else
    config.num_server       = 1;
#endif
    config.scn[0]           = HANDS_FREE_SCN;
    config.uuid[0]          = UUID_SERVCLASS_HF_HANDSFREE;
#ifdef WICED_ENABLE_BT_HSP_PROFILE
    config.scn[1]           = HEADSET_SCN;
    config.uuid[1]          = UUID_SERVCLASS_HEADSET;
#endif

    result = wiced_bt_hfp_hf_init(&config, handsfree_event_callback);
    WICED_BT_TRACE("[%s] SCO Setting up voice path = %d\n",__func__, result);
}

void handsfree_write_eir()
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;

    pBuf = (uint8_t*)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );
    WICED_BT_TRACE( "hci_control_write_eir %x\n", pBuf );

    if ( !pBuf )
    {
        return;
    }
    p = pBuf;

    //p = ( uint8_t * )( pBuf + 1 );
    //p += 4;

    length = strlen( (char *)handsfree_cfg_settings.device_name );

    *p++ = length + 1;
    *p++ = 0x09;            // EIR type full name
    memcpy( p, handsfree_cfg_settings.device_name, length );
    p += length;
    *p++ = ( 1 * 2 ) + 1;     // length of services + 1
    *p++ =   0x02;            // EIR type full list of 16 bit service UUIDs
#ifdef WICED_ENABLE_BT_HSP_PROFILE
    *p++ =   UUID_SERVCLASS_HEADSET         & 0xff;
    *p++ = ( UUID_SERVCLASS_HEADSET >> 8 ) & 0xff;
#endif
    *p++ =   UUID_SERVCLASS_HF_HANDSFREE        & 0xff;
    *p++ = ( UUID_SERVCLASS_HF_HANDSFREE >> 8 ) & 0xff;
    *p++ =   UUID_SERVCLASS_GENERIC_AUDIO        & 0xff;
    *p++ = ( UUID_SERVCLASS_GENERIC_AUDIO >> 8 ) & 0xff;
    *p++ = 0;

    // print EIR data
    wiced_bt_trace_array( "EIR :", ( uint8_t* )( pBuf+1 ), MIN( p-( uint8_t* )pBuf,100 ) );
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    return;
}

extern wiced_bt_buffer_pool_t* p_key_info_pool;//Pool for storing the  key info
extern void hci_control_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );

void handsfree_post_bt_init(wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_BT_ERROR;
    if(p_event_data->enabled.status == WICED_BT_SUCCESS)
    {
        WICED_BT_TRACE("Bluetooth stack initialized\n");

        handsfree_app_states.pairing_allowed = WICED_FALSE;
        wiced_init_timer( &handsfree_app_states.hfp_timer, hfp_timer_expiry_handler, 0,
                        WICED_MILLI_SECONDS_TIMER );

        /* Set-up EIR data */
        handsfree_write_eir();
        /* Set-up SDP database */
        wiced_bt_sdp_db_init((uint8_t *)handsfree_sdp_db, wiced_app_cfg_sdp_record_get_size());

        handsfree_hfp_init();
    }
    else
    {
        WICED_BT_TRACE("Bluetooth stack initialization failure!!\n");
        return;
    }
}

/*
 * Process SCO management callback
 */
void hf_sco_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_hfp_hf_scb_t *p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr (handsfree_ctxt_data.peer_bd_addr);
    int status;

    WICED_BT_TRACE("hf_sco_management_callback: event=%d\n", event);

    switch ( event )
    {
        case BTM_SCO_CONNECTED_EVT:             /**< SCO connected event. Event data: #wiced_bt_sco_connected_t */
#if defined(CYW20721B2) || defined(CYW43012C0)
            /* setup audio path */
            if (stream_id == WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                stream_id = wiced_am_stream_open(HFP);
                WICED_BT_TRACE("wiced_am_stream_open completed stream_id: %d\n", stream_id);
            }

            /* Set sample rate. */
            if (handsfree_esco_params.use_wbs == WICED_TRUE)
            {
                audio_config.sr = AM_PLAYBACK_SR_16K;
            }
            else
            {
                audio_config.sr = AM_PLAYBACK_SR_8K;
            }

            audio_config.volume = AM_VOL_LEVEL_HIGH - 2;
            audio_config.mic_gain = AM_VOL_LEVEL_HIGH - 2;

            if( WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_AUDIO_CONFIG, &audio_config))
                WICED_BT_TRACE("wiced_am_set_param failed\n");

            if( WICED_SUCCESS != wiced_am_stream_start(stream_id))
                WICED_BT_TRACE("wiced_am_stream_start failed stream_id : %d \n", stream_id);

            /* Set speaker volume and MIC gain to make the volume consistency between call
             * sessions. */
            if (WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_SPEAKER_VOL_LEVEL, (void *) &audio_config.volume))
                WICED_BT_TRACE("wiced_am_set_param failed\n");

            if (WICED_SUCCESS != wiced_am_stream_set_param(stream_id, AM_MIC_GAIN_LEVEL, (void *) &audio_config.mic_gain))
                WICED_BT_TRACE("wiced_am_set_param failed\n");
#endif /* CYW20721B2 || CYW43012C0 */

            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_AUDIO_OPEN, p_scb->rfcomm_handle, NULL );
            WICED_BT_TRACE("%s: SCO Audio connected, sco_index = %d [in context sco index=%d]\n", __func__, p_event_data->sco_connected.sco_index, handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.is_sco_connected = WICED_TRUE;

            break;

        case BTM_SCO_DISCONNECTED_EVT:          /**< SCO disconnected event. Event data: #wiced_bt_sco_disconnected_t */
            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_AUDIO_CLOSE, p_scb->rfcomm_handle, NULL );
            WICED_BT_TRACE("%s: SCO disconnection change event handler\n", __func__);

            status = wiced_bt_sco_create_as_acceptor(&handsfree_ctxt_data.sco_index);
            WICED_BT_TRACE("%s: status [%d] SCO INDEX [%d] \n", __func__, status, handsfree_ctxt_data.sco_index);
            handsfree_ctxt_data.is_sco_connected = WICED_FALSE;
            break;

        case BTM_SCO_CONNECTION_REQUEST_EVT:    /**< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t */
            WICED_BT_TRACE("%s: SCO connection request event handler \n", __func__);

            if( wiced_is_timer_in_use(&handsfree_app_states.hfp_timer) )
            {
                wiced_stop_timer(&handsfree_app_states.hfp_timer);
            }

            if(handsfree_app_states.connect.profile_selected == WICED_BT_HFP_PROFILE)
            {
                wiced_bt_sco_accept_connection(p_event_data->sco_connection_request.sco_index, HCI_SUCCESS, (wiced_bt_sco_params_t *) &handsfree_esco_params);
            }
#ifdef WICED_ENABLE_BT_HSP_PROFILE
            else
            {
                wiced_bt_sco_accept_connection(p_event_data->sco_connection_request.sco_index, HCI_SUCCESS, (wiced_bt_sco_params_t *) &headset_sco_params);
            }
#endif
            break;

        case BTM_SCO_CONNECTION_CHANGE_EVT:     /**< SCO connection change event. Event data: #wiced_bt_sco_connection_change_t */
            WICED_BT_TRACE("%s: SCO connection change event handler\n", __func__);
            break;
    }
    UNUSED_VARIABLE(status);
}

static void hfp_timer_expiry_handler( uint32_t param )
{
    /* if sco is not created as an acceptor then remove the sco and create it as initiator. */
    if( handsfree_ctxt_data.call_active && !handsfree_ctxt_data.is_sco_connected )
    {
        wiced_bt_sco_remove( handsfree_ctxt_data.sco_index );
        wiced_bt_sco_create_as_initiator( handsfree_ctxt_data.peer_bd_addr, &handsfree_ctxt_data.sco_index, (wiced_bt_sco_params_t *) &handsfree_esco_params );
    }
}

/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
int handsfree_write_nvram( int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    int             bytes_written = wiced_hal_write_nvram( nvram_id, data_len, (uint8_t*)p_data, &result );

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
int handsfree_read_nvram( int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if (data_len >= sizeof(wiced_bt_device_link_keys_t))
    {
        read_bytes = wiced_hal_read_nvram( nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result );
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(wiced_bt_device_link_keys_t), read_bytes, result );
    }
    return (read_bytes);
}



wiced_result_t handsfree_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    int nvram_id;
    int bytes_written, bytes_read;
    wiced_result_t result = WICED_BT_SUCCESS;
    wiced_bt_dev_pairing_cplt_t *p_pairing_cmpl;
    uint8_t                      pairing_result;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    const uint8_t *link_key;

    WICED_BT_TRACE( "Bluetooth management callback event: 0x%02x\n", event );

    switch(event)
    {

        case BTM_ENABLED_EVT:
            //disable pairing
            wiced_bt_set_pairable_mode(0,0);

            handsfree_post_bt_init(p_event_data);

            //Creating a buffer pool for holding the peer devices's key info
            p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE, KEY_INFO_POOL_BUFFER_COUNT );
            WICED_BT_TRACE( "wiced_bt_create_pool %x\n", p_key_info_pool );

            wiced_bt_dev_register_hci_trace( hci_control_hci_trace_cback );

#ifdef CYW20706A2
            hci_control_send_device_started_evt( );
#endif

#if defined(CYW20721B2) || defined(CYW43012C0)
            wiced_am_init();
#endif
            break;

        case BTM_DISABLED_EVT:
            break;

        case BTM_SCO_CONNECTED_EVT:
        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
            hf_sco_management_callback(event, p_event_data);
            break;

        case BTM_SCO_DISCONNECTED_EVT:
#if defined(CYW20721B2) || defined(CYW43012C0)
            if (stream_id != WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
            {
                if( WICED_SUCCESS != wiced_am_stream_stop(stream_id))
                    WICED_BT_TRACE("wiced_am_stream_stop failed stream_id : %d \n", stream_id);

                if( WICED_SUCCESS != wiced_am_stream_close(stream_id))
                    WICED_BT_TRACE("wiced_am_stream_close failed stream_id : %d \n", stream_id);

                stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;
            }
#endif
            hf_sco_management_callback(event, p_event_data);
            break;

        case BTM_SECURITY_REQUEST_EVT:
            if ( handsfree_app_states.pairing_allowed )
            {
                wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            }
            else
            {
                // Pairing not allowed, return error
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            p_pairing_cmpl = &p_event_data->pairing_complete;

            if(p_pairing_cmpl->transport == BT_TRANSPORT_BR_EDR)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
            }
            hci_control_send_pairing_completed_evt( pairing_result, p_event_data->pairing_complete.bd_addr );
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            /* Check if we already have information saved for this bd_addr */
            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
            {
                // This is the first time, allocate id for the new memory chunk
                nvram_id = hci_control_alloc_nvram_id( );
                WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
            }
            bytes_written = hci_control_write_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), &p_event_data->paired_device_link_keys_update, WICED_FALSE );

            WICED_BT_TRACE("NVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);
            link_key = p_event_data->paired_device_link_keys_update.key_data.br_edr_key;
            WICED_BT_TRACE(" LinkKey:%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
                    link_key[0], link_key[1], link_key[2], link_key[3], link_key[4], link_key[5], link_key[6], link_key[7],
                    link_key[8], link_key[9], link_key[10], link_key[11], link_key[12], link_key[13], link_key[14], link_key[15]);
            break;


        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            /* read existing key from the NVRAM  */

            WICED_BT_TRACE("\t\tfind device %B\n", p_event_data->paired_device_link_keys_request.bd_addr);

            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_request.bd_addr, BD_ADDR_LEN ) ) != 0)
            {
                 bytes_read = hci_control_read_nvram( nvram_id, &p_event_data->paired_device_link_keys_request, sizeof( wiced_bt_device_link_keys_t ) );

                 result = WICED_BT_SUCCESS;
                 WICED_BT_TRACE("Read:nvram_id:%d bytes:%d\n", nvram_id, bytes_read);
            }
            else
            {
                result = WICED_BT_ERROR;
                WICED_BT_TRACE("Key retrieval failure\n");
            }
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            /* Use the default security for BLE */
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT bda %B\n",
                    p_event_data->pairing_io_capabilities_ble_request.bd_addr);
            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data      = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req      = BTM_LE_AUTH_REQ_SC_MITM_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size  = 16;
            p_event_data->pairing_io_capabilities_ble_request.init_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys     = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
            //            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_ALL_PROFILES_NO;
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;
            WICED_BT_TRACE( "Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr, p_encryption_status->result );
            break;

        default:
            break;
    }
    UNUSED_VARIABLE(p_encryption_status);
    UNUSED_VARIABLE(bytes_read);
    UNUSED_VARIABLE(bytes_written);
    return result;
}

static void hci_control_transport_status( wiced_transport_type_t type )
{
    WICED_BT_TRACE( " hci_control_transport_status %x \n", type );
    hci_control_send_device_started_evt();
}
/*
 *  Application Start, ie, entry point to the application.
 */
APPLICATION_START()
{
#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
    wiced_transport_init( &transport_cfg );

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

#ifdef NO_PUART_SUPPORT
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#else
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if ( defined(CYW20706A2) || defined(CYW20735B0) || defined(CYW20719B0) || defined(CYW43012C0) )
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif
#endif
    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE( "Starting Hands-free Application...\n" );

    /* Initialize Bluetooth stack */
    wiced_bt_stack_init( handsfree_management_callback , &handsfree_cfg_settings, handsfree_cfg_buf_pools);

    /* Configure Audio buffer */
    wiced_audio_buffer_initialize (handsfree_audio_buf_config);
}
