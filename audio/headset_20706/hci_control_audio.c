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
 * This file implements audio application controlled over UART.
 *
 */
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_bt_trace.h"
#include "hci_control_audio.h"
#include "hci_control_api.h"
#include "hci_control.h"
#include "wiced_timer.h"
#include "string.h"
#include "wiced_bt_avrc_ct.h"
#include "wiced_bt_utils.h"

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/
#define MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS 2

/******************************************************************************
 *                         Type Definitions
 ******************************************************************************/

typedef struct
{
    wiced_bt_device_address_t       peerBda;             /* Peer bd address */
    AV_STATE                        state;               /* AVDT State machine state */
    AV_STREAM_STATE                 audio_stream_state;  /* Audio Streaming to host state */
    wiced_bt_a2dp_codec_info_t      codec_config;       /* Codec configuration information */
    uint16_t                        handle;             /* connection handle */
} tAV_APP_CB;

/******************************************************************************
 *                         Variable Definitions
 ******************************************************************************/

/* A2DP module control block */
static tAV_APP_CB                   av_app_cb[MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS];
extern wiced_bt_a2dp_config_data_t  bt_audio_config;
wiced_timer_t avrc_timer;

static void a2dp_sink_control_cback( wiced_bt_a2dp_sink_event_t event, wiced_bt_a2dp_sink_event_data_t* p_data );

extern wiced_result_t hci_control_audio_send_codec_configured( wiced_bt_a2dp_codec_info_t codec_info);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
static const char *dump_control_event_name(wiced_bt_a2dp_sink_event_t event)
{
    switch((int)event)
    {
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_CONNECT_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_DISCONNECT_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_START_IND_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_START_CFM_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_SUSPEND_EVT)
        CASE_RETURN_STR(WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT)
    }

    return NULL;
}

static const char *dump_state_name(AV_STATE state)
{
    switch((int)state)
    {
        CASE_RETURN_STR(AV_STATE_IDLE)
        CASE_RETURN_STR(AV_STATE_CONFIGURED)
        CASE_RETURN_STR(AV_STATE_CONNECTED)
        CASE_RETURN_STR(AV_STATE_STARTED)
    }

    return NULL;
}

/*******************************************************************************
 * A2DP Application HCI Control handlers
 *******************************************************************************/

static uint16_t get_index_from_address(wiced_bt_device_address_t bd_addr)
{
    int i = 0;
    uint16_t idle_index = 0xFFFF;

    /* loop-in to see if there is any matching BD_ADDR */
    for( ; i < MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS; i++)
    {
        /* If found the matching address, return the handle */
        if( !utl_bdcmp(bd_addr, av_app_cb[i].peerBda) )
        {
            return i;
        }
        /* else check if this non-matching address/handle is IDLE */
        if( av_app_cb[i].state == AV_STATE_IDLE && idle_index == 0xFFFF)
        {
            /* Got the first IDLE handle */
            idle_index = i;
        }
    }

    return idle_index;
}

static uint16_t get_index_from_handle(uint16_t handle)
{
    int i = 0;

    for( ; i < MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS; i++)
    {
        /* If found the matching address, return the handle */
        if( av_app_cb[i].handle == handle )
        {
            return i;
        }
    }

    return 0xFFFF;
}

/*
 * Handles the audio connect command
 */
wiced_result_t a2dp_app_hci_control_connect(uint8_t* p_data, uint32_t len)
{
    wiced_result_t status = WICED_ERROR;
    wiced_bt_device_address_t bd_addr;
    int i;
    uint16_t idx;

    for ( i = 0; i < BD_ADDR_LEN; i++ )
            bd_addr[i] = p_data[BD_ADDR_LEN - i - 1];

    idx = get_index_from_address(bd_addr);

    WICED_BT_TRACE( "[%s]  <%B> idx:%d\n\r", __FUNCTION__, bd_addr, idx);

    if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS )
    {
        WICED_BT_TRACE("Invalid index(no idle index for Connecting A2DP)\n\r");
        return WICED_BADARG;
    }

    /* First make sure that there is not already a connection */
    if (av_app_cb[idx].state == AV_STATE_IDLE)
    {
        status = wiced_bt_a2dp_sink_connect(bd_addr);
    }
    else
    {
        WICED_BT_TRACE( "[%s]: Bad state: (%d) %s", __FUNCTION__, av_app_cb[idx].state, dump_state_name(av_app_cb[idx].state));
    }
    return status;
}

/*
 * Handles the audio disconnect
 */
wiced_result_t a2dp_app_hci_control_disconnect( uint8_t* p_data, uint32_t len )
{
    uint16_t       handle = p_data[0] + (p_data[1] << 8);
    wiced_result_t status = WICED_ERROR;
    uint16_t idx;

    WICED_BT_TRACE( "[%s] Handle %d\n\r", __FUNCTION__, handle );

    idx = get_index_from_handle(handle);

    if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS )
    {
        WICED_BT_TRACE("Invalid Index(no idle index for Connecting A2DP)\n\r");
        return WICED_BADARG;
    }

    /* if already idle there is no connection to disconnect */
    /* TODO: Bad use of an enumerated type as an integer. Fix later */
    if ( av_app_cb[idx].state >= AV_STATE_CONNECTED )
    {
        if (WICED_BT_PENDING == wiced_bt_dev_cancel_sniff_mode (av_app_cb[idx].peerBda))
        {
            WICED_BT_TRACE( "[%s] sniff canceling for <%B>\n\r", __FUNCTION__, av_app_cb[idx].peerBda );
        }

        /* attempt to disconnect directly */
        status = wiced_bt_a2dp_sink_disconnect(handle);
    }
    else
    {
        WICED_BT_TRACE( "[%s]: Bad state: (%d) %s", __FUNCTION__, av_app_cb[idx].state, dump_state_name(av_app_cb[idx].state));
    }

    return status;
}

/*
 * Handles the audio start (AVDT START)
 */
wiced_result_t a2dp_app_hci_control_start( uint8_t* p_data, uint32_t len )
{
    uint16_t       handle = p_data[0] + (p_data[1] << 8);
    wiced_result_t status = WICED_ERROR;
    uint16_t idx;

    WICED_BT_TRACE( "[%s] Handle %d\n\r", __FUNCTION__, handle );

    idx = get_index_from_handle(handle);

    if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS )
    {
        WICED_BT_TRACE("Invalid Index for Audio start\n\r");
        return WICED_BADARG;
    }

    if (av_app_cb[idx].state == AV_STATE_CONNECTED)
    {
        status = wiced_bt_a2dp_sink_start(handle);
    }
    else
    {
        WICED_BT_TRACE( "[%s]: Bad state: (%d) %s", __FUNCTION__, av_app_cb[idx].state, dump_state_name(av_app_cb[idx].state));
    }

    return status;
}

/*
 * Handles the audio stop (AVDT SUSPEND)
 */
wiced_result_t a2dp_app_hci_control_stop( uint8_t* p_data, uint32_t len )
{
    uint16_t       handle = p_data[0] + (p_data[1] << 8);
    wiced_result_t status = WICED_ERROR;
    uint16_t idx;

    WICED_BT_TRACE( "[%s] Handle %d\n\r", __FUNCTION__, handle );

    idx = get_index_from_handle(handle);

    if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS )
    {
        WICED_BT_TRACE("Invalid Index for AUDIO_STOP\n\r");
        return WICED_BADARG;
    }

    if (av_app_cb[idx].state == AV_STATE_STARTED)
    {
        status = wiced_bt_a2dp_sink_suspend(handle);
    }
    else
    {
        WICED_BT_TRACE( "[%s]: Bad state: (%d) %s", __FUNCTION__, av_app_cb[idx].state, dump_state_name(av_app_cb[idx].state));
    }

    return status;
}

/*
 * Handles the audio stop (AVDT SUSPEND)
 */
wiced_result_t a2dp_app_hci_control_start_rsp( uint8_t* p_data, uint32_t len )
{
    uint16_t       handle   = p_data[0] + (p_data[1] << 8);
    uint8_t        label    = p_data[2];
    uint8_t        response = p_data[3];//Eg: A2D_SUCCESS           0     /**< Success */
    wiced_result_t status   = WICED_ERROR;
    uint16_t idx;

    if (len != 4 )
        return WICED_BADARG;

    idx = get_index_from_handle(handle);

    if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS )
    {
        WICED_BT_TRACE("Invalid Index for START_RSP\n\r");
        return WICED_BADARG;
    }

    WICED_BT_TRACE(" [%s] Label:%x response:%x \n",__FUNCTION__, label, response);

    status = wiced_bt_a2dp_sink_send_start_response( handle, label, response );
    if ( status == WICED_SUCCESS )
    {
        /* Maintain State */
        av_app_cb[idx].state = AV_STATE_STARTED;
        WICED_BT_TRACE(" a2dp sink streaming accepted \n");
    }
    return status;
}

#ifdef CHANGE_ROUTE_SUPPORT
/*
 * Handles the change route command from the host
 */
wiced_result_t a2dp_app_hci_control_change_route( uint8_t* p_data, uint32_t len )
{
    uint16_t       handle = p_data[0] + (p_data[1] << 8);
    wiced_result_t status = WICED_ERROR;
    uint16_t idx;
    wiced_bt_a2dp_sink_route_config route_config;

    WICED_BT_TRACE( "[%s] Handle %d\n\r", __FUNCTION__, handle );

    idx = get_index_from_handle(handle);

    if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS )
    {
       WICED_BT_TRACE("Invalid Index for AUDIO_STOP\n\r");
       return WICED_BADARG;
    }

    route_config.route = p_data[2];
    if (  wiced_bt_a2dp_sink_update_route_config( handle, &route_config) )
    {
        status = WICED_SUCCESS;
    }
    return status;
}
#endif

uint8_t hci_control_audio_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t len )
{
    uint8_t  status      = HCI_CONTROL_STATUS_FAILED;
    uint32_t send_status = 1;

    switch ( cmd_opcode )
    {

    case HCI_CONTROL_AUDIO_SINK_COMMAND_CONNECT:
        status = a2dp_app_hci_control_connect( p_data, len ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AUDIO_SINK_COMMAND_DISCONNECT:
        status = a2dp_app_hci_control_disconnect( p_data, len ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AUDIO_SINK_COMMAND_START:
        status = a2dp_app_hci_control_start( p_data, len ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AUDIO_SINK_COMMAND_STOP:
        status = a2dp_app_hci_control_stop( p_data, len ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;

    case HCI_CONTROL_AUDIO_SINK_COMMAND_START_RSP:
        status = a2dp_app_hci_control_start_rsp( p_data, len ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;
#ifdef CHANGE_ROUTE_SUPPORT
    case HCI_CONTROL_AUDIO_SINK_COMMAND_CHANGE_ROUTE:
        status = a2dp_app_hci_control_change_route( p_data, len ) == WICED_SUCCESS ? HCI_CONTROL_STATUS_SUCCESS : HCI_CONTROL_STATUS_FAILED;
        break;
#endif
    default:
        status = HCI_CONTROL_STATUS_UNKNOWN_COMMAND;
        break;
    }

    if ( send_status )
        hci_control_send_command_status_evt( HCI_CONTROL_AUDIO_SINK_EVENT_COMMAND_STATUS, status );


    return status;
}

void avrc_timer_expiry_handler(  uint32_t param )
{
    uint16_t idx;

    WICED_BT_TRACE("Timer Expired \n");

    idx = get_index_from_handle(app_state_cb.peer_conn_handle);
    if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS)
    {
        WICED_BT_TRACE("No available Handle for SINK_CONNECT event  handle:%x\n\r", app_state_cb.peer_conn_handle);
        return;
    }

    /* Make an AVRC connection, if AVRC is not connected earlier and A2DP is connected */
    if( (app_state_cb.avrc_state == 0) && (av_app_cb[idx].state == AV_STATE_CONNECTED) )
    {
        wiced_bt_avrc_ct_connect( av_app_cb[idx].peerBda );
    }
}

/* ****************************************************************************
 * Function: a2dp_sink_control_cback
 *
 * Parameters:
 *          event - control event called back
 *          p_data - event data
 *
 * Description:
 *          Control callback supplied by  the a2dp sink profile code.
 * ***************************************************************************/

void a2dp_sink_control_cback( wiced_bt_a2dp_sink_event_t event,  wiced_bt_a2dp_sink_event_data_t* p_data )
{
    uint16_t idx = 0xFFFF;
    WICED_BT_TRACE( "[%s] Event: (%d) %s\n\r", __FUNCTION__, event, dump_control_event_name(event) );

    switch(event)
    {
        case WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT: /**< Codec config event, received when codec config for a streaming session is updated */
        {
            static wiced_bt_a2dp_codec_info_t   parsed_codec;
            /* Save the configuration to setup the decoder if necessary. */
            memcpy( &parsed_codec, &p_data->codec_config.codec, sizeof( wiced_bt_a2dp_codec_info_t ) );
            hci_control_audio_send_codec_configured(parsed_codec);
            WICED_BT_TRACE(" a2dp sink codec configuration done\n\r");
            break;
        }

        case WICED_BT_A2DP_SINK_CONNECT_EVT:      /**< Connected event, received on establishing connection to a peer device. Ready to stream. */
            if (p_data->connect.result == WICED_SUCCESS)
            {
                uint16_t settings = HCI_ENABLE_MASTER_SLAVE_SWITCH;// HCI_DISABLE_ALL_LM_MODES;

                idx = get_index_from_address(p_data->connect.bd_addr);
                if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS)
                {
                    WICED_BT_TRACE("No available Handle for SINK_CONNECT event  handle:%x\n\r", p_data->connect.handle);
                    break;
                }


                /* Save the address of the remote device on remote connection */
                memcpy(av_app_cb[idx].peerBda, p_data->connect.bd_addr, sizeof(wiced_bt_device_address_t));
                av_app_cb[idx].handle = p_data->connect.handle;
                app_state_cb.peer_conn_handle = p_data->connect.handle;

//                BTM_SetLinkPolicy(av_app_cb[idx].peerBda, &settings);

                /* Notify MCU of connection state change */
                hci_control_audio_send_connect_complete( av_app_cb[idx].peerBda, WICED_SUCCESS, av_app_cb[idx].handle );

                /* Maintain State */
                av_app_cb[idx].state = AV_STATE_CONNECTED;

                WICED_BT_TRACE( "[%s] A2DP sink connected to addr: <%B> Handle:%d idx:%d\n\r", __FUNCTION__, p_data->connect.bd_addr,
                    p_data->connect.handle, idx );
#ifdef STANDALONE_HEADSET_APP
         if (app_state_cb.is_originator)
         {
             wiced_bt_avrc_ct_connect( p_data->connect.bd_addr );
         }
#endif


         /* run a timer for a certain time to check if Peer already initiated AVRCP connection */
         /* If we are Originator, timer should not be started */
#ifdef STANDALONE_HEADSET_APP
               if( (!app_state_cb.is_originator) && (app_state_cb.avrc_state == 0) )
#else
               if( app_state_cb.avrc_state == 0 )
#endif
               {
                     WICED_BT_TRACE("starting a timer for AVRC\n");
                     /* start a timer */
                     wiced_start_timer( &avrc_timer, 500 );
               }
            }
            else
            {
                /* Notify MCU of connection state change */
                hci_control_audio_send_connect_complete( NULL, WICED_ERROR, 0 );

                WICED_BT_TRACE(" a2dp sink connection failed %d \n", p_data->connect.result );
            }
            break;

        case WICED_BT_A2DP_SINK_DISCONNECT_EVT:   /**< Disconnected event, received on disconnection from a peer device */

            idx = get_index_from_handle(p_data->disconnect.handle);

            if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS)
            {
                WICED_BT_TRACE("No available Handle for SINK_DISCONNECT event  handle:%x\n\r", p_data->disconnect.handle);
                break;
            }

            /* Maintain State */
            av_app_cb[idx].state = AV_STATE_IDLE;
            /* maintain local app state */
            app_state_cb.state = IDLE;
            app_state_cb.audio_stream_state = APP_STREAM_STATE_STOPPED;
            WICED_BT_TRACE(" a2dp sink disconnected  idx:%d handle:%d\n\r", idx, p_data->disconnect.handle);
            hci_control_audio_send_disconnect_complete( p_data->disconnect.handle, 0, 0 );
            break;

        case WICED_BT_A2DP_SINK_START_IND_EVT:        /**< Start stream event, received when audio streaming is about to start */

            idx = get_index_from_handle(p_data->start_ind.handle);

            if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS)
            {
                WICED_BT_TRACE("No available Handle for SINK_START_IND event  handle:%x\n\r", p_data->start_ind.handle);
                break;
            }
            WICED_BT_TRACE("app state:%d",app_state_cb.audio_stream_state);
            /* Maintain State */
            av_app_cb[idx].state = AV_STATE_STARTED;
            /* maintain local app state */
            app_state_cb.state = A2DP;
            app_state_cb.audio_stream_state = APP_STREAM_STATE_PLAYING;

            WICED_BT_TRACE(" WICED_BT_A2DP_SINK_START_IND_EVT handle %x label:%x\n", p_data->start_ind.handle, p_data->start_ind.label );
            hci_control_audio_send_start_indication( p_data->start_ind.handle, p_data->start_ind.label);
            break;

        case WICED_BT_A2DP_SINK_START_CFM_EVT:        /**< Start stream event, received when audio streaming is about to start */

            idx = get_index_from_handle(p_data->start_cfm.handle);

            if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS)
            {
                WICED_BT_TRACE("No available Handle for SINK_START_CFM event  handle:%x\n\r", p_data->start_cfm.handle);
                break;
            }

            /* Maintain State */
            av_app_cb[idx].state = AV_STATE_STARTED;
            /* maintain local app state */
            app_state_cb.state = A2DP;
            app_state_cb.audio_stream_state = APP_STREAM_STATE_PLAYING;

            hci_control_audio_send_started_stopped( p_data->start_cfm.handle, TRUE );

            WICED_BT_TRACE(" WICED_BT_A2DP_SINK_START_CFM_EVT handle: %x \n", p_data->start_cfm.handle );

            break;

        case WICED_BT_A2DP_SINK_SUSPEND_EVT:      /**< Suspend stream event, received when audio streaming is suspended */

            idx = get_index_from_handle(p_data->suspend.handle);

            if( idx >= MAX_NUM_OF_SIMULTANEOUS_A2DP_CONNECTIONS)
            {
                WICED_BT_TRACE("No available index for SINK_SUSPEND event handle:%x\n\r", p_data->suspend.handle);
                break;
            }

            /* Maintain State */
            av_app_cb[idx].state = AV_STATE_CONNECTED;
            /* maintain local app state */
            app_state_cb.audio_stream_state = APP_STREAM_STATE_STOPPED;
            WICED_BT_TRACE(" a2dp sink streaming suspended Handle:%d\n\r", p_data->suspend.handle);
            /* Send Audio stopped event to the MCU */
            hci_control_audio_send_started_stopped( p_data->suspend.handle, FALSE );
            break;

        default:
            break;
    }
}

/******************************************************************************
 *                     Application Initialization
 ******************************************************************************/

wiced_result_t headset_a2dp_init (void)
{
    wiced_result_t result;

    memset(av_app_cb, 0, sizeof(av_app_cb));

    /* Register with the A2DP sink profile code */
    result = wiced_bt_a2dp_sink_init( &bt_audio_config,
                                      a2dp_sink_control_cback );
    WICED_BT_TRACE( "[%s] A2DP sink initialized result:%x\n", __FUNCTION__, result );

    wiced_init_timer( &avrc_timer, avrc_timer_expiry_handler, 0,
               WICED_MILLI_SECONDS_TIMER );
    return result;
}
