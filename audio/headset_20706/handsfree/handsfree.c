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
 *  This file implements the handsfree functions
 *
 */
#include "wiced_bt_sco.h"
#include "../hci_control.h"
#include "wiced_timer.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_rfcomm.h"
#include "wiced_transport.h"
#include "string.h"

static void handsfree_event_callback( wiced_bt_hfp_hf_event_t event, wiced_bt_hfp_hf_event_data_t* p_data);
static void handsfree_init_context_data(void);
static void hci_control_send_hf_event(uint16_t evt, uint16_t handle, hci_control_hf_event_t *p_data);

void hf_sco_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
wiced_bt_dev_status_t wiced_bt_sco_create_as_acceptor (uint16_t *p_sco_index);
void hci_control_send_sco_confirmation_request_evt( BD_ADDR bda, uint16_t sco_index );

extern wiced_bt_sco_params_t handsfree_esco_params;

bluetooth_hfp_context_t handsfree_ctxt_data[HFP_MAX_SLC_CONNECTION];

bluetooth_hfp_context_t* get_context_ptr_from_handle(uint16_t handle)
{
    int i = 0;
    bluetooth_hfp_context_t* tmp_ctxt = NULL;

    for( ; i< HFP_MAX_SLC_CONNECTION; i++ )
    {
        if( handsfree_ctxt_data[i].rfcomm_handle == handle )
        {
            return &handsfree_ctxt_data[i];
        }
        if( handsfree_ctxt_data[i].connection_status == WICED_BT_HFP_HF_STATE_DISCONNECTED && (!tmp_ctxt) )
        {
            tmp_ctxt = &handsfree_ctxt_data[i];
        }
    }
    return tmp_ctxt;
}

bluetooth_hfp_context_t* get_context_ptr_from_sco_index(uint16_t sco_index)
{
    int i = 0;
    bluetooth_hfp_context_t* tmp_ctxt = NULL;

    if( sco_index == HANDS_FREE_INVALID_SCO_INDEX )
        return NULL;

    for( ; i< HFP_MAX_SLC_CONNECTION; i++ )
    {
        if( sco_index == handsfree_ctxt_data[i].sco_index && handsfree_ctxt_data[i].connection_status != WICED_BT_HFP_HF_STATE_DISCONNECTED)
        {
            return &handsfree_ctxt_data[i];
        }
    }
    return tmp_ctxt;
}

static void handsfree_init_context_data(void)
{
    int i = 0;
    for( ; i < HFP_MAX_SLC_CONNECTION; i++)
    {
        handsfree_ctxt_data[i].call_active         = 0;
        handsfree_ctxt_data[i].call_held           = 0;
        handsfree_ctxt_data[i].call_setup          = WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE;
        handsfree_ctxt_data[i].connection_status   = WICED_BT_HFP_HF_STATE_DISCONNECTED;
        handsfree_ctxt_data[i].spkr_volume         = 8;
        handsfree_ctxt_data[i].mic_volume          = 8;
        handsfree_ctxt_data[i].sco_index           = HANDS_FREE_INVALID_SCO_INDEX;
        handsfree_ctxt_data[i].rfcomm_handle       = 0;
        handsfree_ctxt_data[i].init_sco_conn       = WICED_FALSE;
    }
}

wiced_bt_voice_path_setup_t handsfree_sco_path = {
    .path = WICED_BT_SCO_OVER_I2SPCM
};

void handsfree_hfp_init(void)
{
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bt_hfp_hf_config_data_t config;

    handsfree_init_context_data();

    /* Perform the rfcomm init before hf and spp start up */
    result = wiced_bt_rfcomm_init( 700, 4 );
    if( result != WICED_BT_SUCCESS )
    {
        WICED_BT_TRACE("Error Initializing RFCOMM - HFP failed\n");
        return;
    }

    config.feature_mask     = HANDS_FREE_SUPPORTED_FEATURES;
    config.speaker_volume   = handsfree_ctxt_data[0].spkr_volume;
    config.mic_volume       = handsfree_ctxt_data[0].mic_volume;
    config.num_server       = 2;
    config.scn[0]           = HANDS_FREE_SCN;
    config.scn[1]           = HANDS_FREE_SCN1;
    config.uuid[0]          = UUID_SERVCLASS_HF_HANDSFREE;
    config.uuid[1]          = UUID_SERVCLASS_HF_HANDSFREE;

    result = wiced_bt_hfp_hf_init(&config, handsfree_event_callback);
    WICED_BT_TRACE("[%s] wiced_bt_hfp_hf_init %d\n",__func__, result);

//    Default mode is set to I2S Master so no need to call this API
//    To change the mode please call below API and to update PCM configuration use wiced_hal_set_pcm_config API
//    result = wiced_bt_sco_setup_voice_path(&handsfree_sco_path);
//    WICED_BT_TRACE("[%s] SCO Setting up voice path = %d\n",__func__, result);

}

/*
 * Process SCO management callback
 */
void hf_sco_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    uint16_t sco_index = 0xFFFF;
    uint16_t handle = 0xFFFF;
    bluetooth_hfp_context_t* hfp_ptr = NULL;
    int status;
    switch ( event )
    {
        case BTM_SCO_CONNECTED_EVT:             /**< SCO connected event. Event data: #wiced_bt_sco_connected_t */
            sco_index = p_event_data->sco_connected.sco_index;
            hfp_ptr = get_context_ptr_from_sco_index(sco_index);
            if( !hfp_ptr )
            {
                WICED_BT_TRACE("Invalid sco index in BTM_SCO_CONNECTED_EVT\n");
                break;
            }
            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_AUDIO_OPEN, hfp_ptr->rfcomm_handle, NULL );
            WICED_BT_TRACE("%s: SCO Audio connected, sco_index = %d [rfcomm_handle :%d]\n", __func__, sco_index, hfp_ptr->rfcomm_handle);
            break;

        case BTM_SCO_DISCONNECTED_EVT:          /**< SCO disconnected event. Event data: #wiced_bt_sco_disconnected_t */
            sco_index = p_event_data->sco_disconnected.sco_index;
            hfp_ptr = get_context_ptr_from_sco_index(sco_index);
            if( !hfp_ptr )
            {
                WICED_BT_TRACE("Invalid sco index in BTM_SCO_DISCONNECTED_EVT\n");
                break;
            }

            hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_AUDIO_CLOSE, hfp_ptr->rfcomm_handle, NULL );
            WICED_BT_TRACE("%s: SCO disconnection change event handler\n", __func__);

            status = wiced_bt_sco_create_as_acceptor(&hfp_ptr->sco_index);
            WICED_BT_TRACE("%s: status [%d] SCO INDEX [%d] \n", __func__, status, hfp_ptr->sco_index);
            break;

        case BTM_SCO_CONNECTION_REQUEST_EVT:    /**< SCO connection request event. Event data: #wiced_bt_sco_connection_request_t */
            hfp_ptr = get_context_ptr_from_sco_index(p_event_data->sco_connection_request.sco_index);
            if( !hfp_ptr )
            {
                WICED_BT_TRACE("Can't handle SCO_CONNECTION_REQUEST from <%B>\n", p_event_data->sco_connection_request.bd_addr);
                break;
            }

            hfp_ptr->sco_index = p_event_data->sco_connection_request.sco_index;

            WICED_BT_TRACE("%s: SCO connection request event handler \n", __func__);

            /*
             * The check below is a work around for the issue where outgoing Ring tone was not heard,
             * when a call is initiated from Android Phones(Nexus 5,Nexus 6,Pixel).
             * It is expected that AG/MP will resume streaming after the Call is Terminated.
             */
            if( app_state_cb.audio_stream_state == APP_STREAM_STATE_PLAYING )
            {
                wiced_bt_a2dp_sink_suspend( app_state_cb.peer_conn_handle );
            }
            else
            {
#ifdef STANDALONE_HEADSET_APP
        /* accept the sco connection in case of stand alone mode */
            wiced_bt_sco_accept_connection(hfp_ptr->sco_index, HCI_SUCCESS, (wiced_bt_sco_params_t *) &handsfree_esco_params);
#else
            hci_control_send_sco_confirmation_request_evt (p_event_data->sco_connection_request.bd_addr,p_event_data->sco_connection_request.sco_index);
#endif
            }
            break;

        case BTM_SCO_CONNECTION_CHANGE_EVT:     /**< SCO connection change event. Event data: #wiced_bt_sco_connection_change_t */
            WICED_BT_TRACE("%s: SCO connection change event handler\n", __func__);
            break;
    }
}

static void hci_control_send_hf_event(uint16_t evt, uint16_t handle, hci_control_hf_event_t *p_data)
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
    bluetooth_hfp_context_t* hfp_ptr = NULL;

    hfp_ptr = get_context_ptr_from_handle(p_data->handle);
    if( !hfp_ptr )
    {
        WICED_BT_TRACE("Can't get HFP context pointer for address: <%B>", p_data->conn_data.remote_address);
        return;
    }

    if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_CONNECTED)
    {
        hci_control_hf_open_t    open;
        wiced_bt_hfp_hf_scb_t *p_scb = wiced_bt_hfp_hf_get_scb_by_bd_addr (p_data->conn_data.remote_address);
        memcpy(open.bd_addr,p_data->conn_data.remote_address,BD_ADDR_LEN);
        open.status = WICED_BT_SUCCESS;
        memcpy(hfp_ptr->peer_bd_addr,p_data->conn_data.remote_address,BD_ADDR_LEN);
        hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_OPEN, p_scb->rfcomm_handle, (hci_control_hf_event_t *) &open);
        hfp_ptr->rfcomm_handle = p_scb->rfcomm_handle;
        status = wiced_bt_sco_create_as_acceptor(&hfp_ptr->sco_index);
        WICED_BT_TRACE("%s: status [%d] SCO INDEX [%d] \n", __func__, status, hfp_ptr->sco_index);
    }
    else if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_SLC_CONNECTED)
    {
        WICED_BT_TRACE("%s: Peer BD Addr [%B]\n", __func__,p_data->conn_data.remote_address);

        memcpy( hfp_ptr->peer_bd_addr, p_data->conn_data.remote_address, sizeof(wiced_bt_device_address_t));
#ifdef STANDALONE_HEADSET_APP
        if (app_state_cb.is_originator)
        {
            a2dp_app_hci_control_connect(p_data->conn_data.remote_address,BD_ADDR_LEN);
        }
#endif
    }
    else if(p_data->conn_data.conn_state == WICED_BT_HFP_HF_STATE_DISCONNECTED)
    {
        memset(hfp_ptr->peer_bd_addr, 0, sizeof(wiced_bt_device_address_t));
        if(hfp_ptr->sco_index != HANDS_FREE_INVALID_SCO_INDEX)
        {
            status = wiced_bt_sco_remove(hfp_ptr->sco_index);
            hfp_ptr->sco_index = HANDS_FREE_INVALID_SCO_INDEX;
            WICED_BT_TRACE("%s: remove sco status [%d] \n", __func__, status);
        }
        hci_control_send_hf_event( HCI_CONTROL_HF_EVENT_CLOSE, hfp_ptr->rfcomm_handle, NULL);
    }

    hfp_ptr->connection_status =  p_data->conn_data.conn_state;
}

static void handsfree_call_setup_event_handler(uint16_t handle, wiced_bt_hfp_hf_call_data_t* call_data)
{
    bluetooth_hfp_context_t* hfp_ptr = NULL;
    hfp_ptr = get_context_ptr_from_handle(handle);
    app_state_cb.hands_free_handle = handle;

    if( !hfp_ptr )
    {
        WICED_BT_TRACE("Invalid remote-address or Can't handle call-setup\n");
        return;
    }

    switch (call_data->setup_state)
    {
        case WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING:
            WICED_BT_TRACE("%s: Call(incoming) setting-up\n", __func__);
            /* maintain local app state */
            app_state_cb.state = HFP;
            app_state_cb.call_state = HFP_STATE_INCOMING_CALL;

            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_IDLE:
            if(call_data->active_call_present == 0)
            {
                if(hfp_ptr->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_INCOMING ||
                        hfp_ptr->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING ||
                        hfp_ptr->call_setup == WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING )
                {
                    WICED_BT_TRACE("Call: Inactive; Call Set-up: IDLE\n");
                    /* maintain local app state */
                    app_state_cb.state = IDLE;
                    app_state_cb.call_state = HFP_STATE_IDLE;
                    break;
                }
                /* If previous context has an active-call and active_call_present is 0 */
                if(hfp_ptr->call_active == 1)
                {
                    WICED_BT_TRACE("Call Terminated\n");
                    /* maintain local app state */
                    app_state_cb.state = IDLE;
                    app_state_cb.call_state = HFP_STATE_IDLE;
                    break;
                }
            }
            else if( call_data->active_call_present == 1)
            {
                WICED_BT_TRACE("Call: Active; Call-setup: DONE\n");
                /* maintain local app state */
                app_state_cb.state = HFP;
                app_state_cb.call_state = HFP_STATE_CALL_IN_PROGRESS;
            }
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_DIALING:
            WICED_BT_TRACE("Call(outgoing) setting-up\n");
            /* maintain local app state */
            app_state_cb.state = HFP;
            app_state_cb.call_state = HFP_STATE_OUTGOING_CALL;
            break;

        case WICED_BT_HFP_HF_CALLSETUP_STATE_ALERTING:
            WICED_BT_TRACE("Remote(outgoing) ringing\n");
            break;

        default:
            break;
    }
    hfp_ptr->call_active = call_data->active_call_present;
    hfp_ptr->call_setup  = call_data->setup_state;
    hfp_ptr->call_held   = call_data->held_call_present;
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
        {
            bluetooth_hfp_context_t* hfp_ptr = NULL;
            hfp_ptr = get_context_ptr_from_handle(p_data->handle);

            if( !hfp_ptr )
            {
                WICED_BT_TRACE("Invalid remote-address for AG_FEATURE_SUPPORT_EVENT\n");
                return;
            }
            res = HCI_CONTROL_HF_EVENT_CONNECTED;
            p_val.conn.peer_features = p_data->ag_feature_flags;

            if(hfp_ptr && (p_data->ag_feature_flags & WICED_BT_HFP_AG_FEATURE_INBAND_RING_TONE_CAPABILITY) )
            {
                hfp_ptr->inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_ENABLED;
            }
            else
            {
                hfp_ptr->inband_ring_status = WICED_BT_HFP_HF_INBAND_RING_DISABLED;
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
        }

        case WICED_BT_HFP_HF_SERVICE_STATE_EVT:
            handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_SERVICE_IND,p_data->service_state,&p_val.val);
            break;

        case WICED_BT_HFP_HF_CALL_SETUP_EVT:
        {
            bluetooth_hfp_context_t* hfp_ptr = NULL;
            hfp_ptr = get_context_ptr_from_handle(p_data->handle);

            if (hfp_ptr->call_active != p_data->call_data.active_call_present)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_IND,p_data->call_data.active_call_present,&p_val.val);

            if (hfp_ptr->call_held != p_data->call_data.held_call_present)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_HELD_IND,p_data->call_data.held_call_present,&p_val.val);

            if (hfp_ptr->call_setup != p_data->call_data.setup_state)
                handsfree_send_ciev_cmd(p_data->handle,WICED_BT_HFP_HF_CALL_SETUP_IND,p_data->call_data.setup_state,&p_val.val);

            handsfree_call_setup_event_handler(p_data->handle, &p_data->call_data);
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

        case WICED_BT_HFP_HF_INBAND_RING_STATE_EVT:
        {
            bluetooth_hfp_context_t* hfp_ptr = NULL;
            hfp_ptr = get_context_ptr_from_handle(p_data->handle);

            if( !hfp_ptr )
            {
                WICED_BT_TRACE("Invalid remote-address for INBAND_RING_STATE_EVENT\n");
                return;
            }

            hfp_ptr->inband_ring_status = p_data->inband_ring;
            break;
        }
        case WICED_BT_HFP_HF_CLIP_IND_EVT:
            p_val.val.num = p_data->clip.type;
            BCM_STRNCPY_S( p_val.val.str, sizeof( p_val.str ), p_data->clip.caller_num, sizeof( p_val.val.str ) );
            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_CLIP;
            WICED_BT_TRACE("%s: CLIP - number %s, type %d\n", __func__, p_data->clip.caller_num, p_data->clip.type);
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
        {
            bluetooth_hfp_context_t* hfp_ptr = get_context_ptr_from_handle(p_data->handle);

            res = HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_BCS;
            if ( p_data->selected_codec == WICED_BT_HFP_HF_MSBC_CODEC )
                handsfree_esco_params.use_wbs = WICED_TRUE;
            else
                handsfree_esco_params.use_wbs = WICED_FALSE;
            p_val.val.num = p_data->selected_codec;

            if (hfp_ptr && (hfp_ptr->init_sco_conn == WICED_TRUE) )
            {
                wiced_bt_sco_create_as_initiator( hfp_ptr->peer_bd_addr, &hfp_ptr->sco_index, (wiced_bt_sco_params_t *) &handsfree_esco_params );
                hfp_ptr->init_sco_conn = WICED_FALSE;
            }
            break;
        }

        default:
            break;
    }
    if ( res && (res <= (HCI_CONTROL_HF_AT_EVENT_BASE + HCI_CONTROL_HF_AT_EVENT_MAX)) )
    {
        wiced_bt_hfp_hf_scb_t    *p_scb = wiced_bt_hfp_hf_get_scb_by_handle(p_data->handle);
        hci_control_send_hf_event( res, p_scb->rfcomm_handle, (hci_control_hf_event_t *)&p_val );
    }
}
