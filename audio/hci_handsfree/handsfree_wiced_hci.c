/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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

#include "wiced_bt_trace.h"
#include "handsfree.h"
#include "wiced_bt_sco.h"
#include "wiced_memory.h"
#include "wiced_transport.h"
#include "string.h"
#include "wiced_platform.h"
#ifdef CYW43012C0
#include "wiced_hal_watchdog.h"
#else
#include "wiced_hal_wdog.h"
#endif

void hci_control_send_command_status_evt( uint16_t code, uint8_t status );
void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
void hci_control_hf_handle_command(uint16_t opcode, uint8_t* p_data, uint32_t length);
void hci_control_hf_at_command (uint16_t handle, uint8_t command, int num, uint8_t* p_data);
void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
void hci_control_misc_handle_get_version( void );
void hci_control_hf_send_at_cmd (uint16_t handle,char *cmd, uint8_t arg_type, uint8_t arg_format, const char *p_arg, int16_t int_arg);

extern wiced_bt_buffer_pool_t* p_key_info_pool;//Pool for storing the  key info
extern wiced_bt_sco_params_t handsfree_esco_params;

/*
 * handle reset command from UART
 */
void hci_control_handle_reset_cmd( void )
{
    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}


/*
 *  Pass protocol traces up through the UART
 */
void hci_control_hci_trace_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    //Enable below to receive traces over HCI UART
    //send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        wiced_bt_dev_register_hci_trace( hci_control_hci_trace_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }
    wiced_set_debug_uart( route_debug );
}

/*
 * handle command to set local Bluetooth device address
 */
void hci_control_handle_set_local_bda( uint8_t *p_bda)
{
    BD_ADDR bd_addr;
    STREAM_TO_BDADDR( bd_addr,p_bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle Inquiry result callback from teh stack, format and send event over UART
 */
void hci_control_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    int       i;
    uint8_t   len;
    uint8_t   tx_buf[300];
    uint16_t  code;
    uint8_t   *p = tx_buf;

    if ( p_inquiry_result == NULL )
    {
        code = HCI_CONTROL_EVENT_INQUIRY_COMPLETE;
    }
    else
    {
        code = HCI_CONTROL_EVENT_INQUIRY_RESULT;
        WICED_BT_TRACE( "inquiry result %B\n", p_inquiry_result->remote_bd_addr );
        for ( i = 0; i < 6; i++ )
            *p++ = p_inquiry_result->remote_bd_addr[5 - i];
        for ( i = 0; i < 3; i++ )
            *p++ = p_inquiry_result->dev_class[2 - i];
        *p++ = p_inquiry_result->rssi;

        // currently callback does not pass the data of the adv data, need to go through the data
        // zero len in the LTV means that there is no more data
        while ( ( len = *p_eir_data ) != 0 )
        {
            // In the HCI event all parameters should fit into 255 bytes
            if ( p + len + 1 > tx_buf + 255 )
            {
                WICED_BT_TRACE( "Bad data\n" );
                break;
            }
            for ( i = 0; i < len + 1; i++ )
                *p++ = *p_eir_data++;
        }
    }
    wiced_transport_send_data( code, tx_buf, ( int )( p - tx_buf ) );
}

/*
 *  Handle Inquiry command received over UART
 */
void hci_control_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {

        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = 5;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &hci_control_inquiry_result_cback );
        WICED_BT_TRACE( "inquiry started:%d\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "cancel inquiry:%d\n", result );
    }
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    UNUSED_VARIABLE(result);
}

/*
 *  Handle Set Visibility command received over UART
 */
void hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability )
{
    // we cannot be discoverable and not connectable
    if ( ( ( discoverability != 0 ) && ( connectability == 0 ) ) ||
           ( discoverability > 1 ) ||
           ( connectability > 1 ) )
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_INVALID_ARGS );
    }
    else
    {
        wiced_bt_dev_set_discoverability( ( discoverability != 0 ) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE ,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability( ( connectability != 0 ) ? WICED_TRUE : WICED_FALSE ,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);

        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
}

/*
 *  Handle Set Pairability command received over UART
 */
void hci_control_handle_set_pairability ( uint8_t allowed )
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;
    void                      *p1;

    if ( handsfree_app_states.pairing_allowed != allowed )
    {
        if ( allowed )
        {
            // Check if key buffer pool has buffer available. If not, cannot enable pairing until nvram entries are deleted
            if ( ( p1 = ( void * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
            {
                allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
            else
            {
                wiced_bt_free_buffer( p1 );
            }
        }

        handsfree_app_states.pairing_allowed = allowed;
        wiced_bt_set_pairable_mode( handsfree_app_states.pairing_allowed, 0 );
        WICED_BT_TRACE( " Set the pairing allowed to %d \n", handsfree_app_states.pairing_allowed );
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status );
}

/*
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt( void )
{
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );

    WICED_BT_TRACE( "maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
                handsfree_cfg_settings.l2cap_application.max_links,
                handsfree_cfg_settings.l2cap_application.max_channels,
                handsfree_cfg_settings.l2cap_application.max_psm,
                handsfree_cfg_settings.rfcomm_cfg.max_links,
                handsfree_cfg_settings.rfcomm_cfg.max_ports );
}

/*
* transfer command status event to UART
*/
void hci_control_send_command_status_evt( uint16_t code, uint8_t status )
{
    wiced_transport_send_data( code, &status, 1 );
}

/*
 *  Send Pairing Completed event through UART
 */
void hci_control_send_pairing_completed_evt( uint8_t status , wiced_bt_device_address_t bdaddr )
{
    const int cmd_size = BD_ADDR_LEN + sizeof(uint8_t);
    uint8_t event_data[cmd_size];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    {
        int i;
        for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];
    }

    WICED_BT_TRACE(" sending the pairing complete evt: %B as %B status %d\n", bdaddr, &event_data[1], status);

    wiced_transport_send_data(HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, cmd_bytes );
}

/*
 *  Send Encryption Changed event through UART
 */
void hci_control_send_encryption_changed_evt( uint8_t encrypted ,  wiced_bt_device_address_t bdaddr )
{
    const int cmd_size = BD_ADDR_LEN + sizeof(uint8_t);
    uint8_t event_data[cmd_size];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = encrypted;

    {
        int i;
        for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];
    }

    wiced_transport_send_data( HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, event_data, cmd_bytes );
}

/*
 * Handle received command over UART.
 */
uint32_t hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t* p_rx_buf = p_data;

    WICED_BT_TRACE( "hci_control_proc_rx_cmd:%d\n", length );

    if ( !p_rx_buf )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    //Expected minimum 4 byte as the wiced header
    if( length < 4 )
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer( p_rx_buf );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }


    STREAM_TO_UINT16(opcode, p_data);     // Get opcode
    STREAM_TO_UINT16(payload_len, p_data); // Get len

    WICED_BT_TRACE("cmd_opcode 0x%02x\n", opcode);

    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_HF:
        hci_control_hf_handle_command ( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_MISC:
        hci_control_misc_handle_command(opcode, p_data, payload_len);
        break;

    default:
        WICED_BT_TRACE( "unknown class code\n");
        break;
    }

    //Freeing the buffer in which data is received
    wiced_transport_free_buffer( p_rx_buf );
    return status;
}

void hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    uint8_t bytes_written;
    switch( cmd_opcode )
    {
    case HCI_CONTROL_COMMAND_RESET:
        hci_control_handle_reset_cmd( );
        break;

    case HCI_CONTROL_COMMAND_TRACE_ENABLE:
        hci_control_handle_trace_enable( p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_BDA:
        hci_control_handle_set_local_bda( p_data );
        break;

    case HCI_CONTROL_COMMAND_PUSH_NVRAM_DATA:
        bytes_written = hci_control_write_nvram( p_data[0] | ( p_data[1] << 8 ), data_len - 2, &p_data[2], TRUE );
        WICED_BT_TRACE( "NVRAM write: %d\n", bytes_written );
        break;

    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_delete_nvram( p_data[0] | ( p_data[1] << 8 ) ,WICED_TRUE);
        WICED_BT_TRACE( "NVRAM delete: %d\n", p_data[0] | ( p_data[1] << 8 ) );
        break;

    case HCI_CONTROL_COMMAND_INQUIRY:
        hci_control_inquiry( *p_data );
        break;

    case HCI_CONTROL_COMMAND_SET_VISIBILITY:
        hci_control_handle_set_visibility( p_data[0], p_data[1] );
        break;

    case  HCI_CONTROL_COMMAND_SET_PAIRING_MODE:
        hci_control_handle_set_pairability( p_data[0] );
        break;
    default:
        WICED_BT_TRACE( "??? Unknown command code\n" );
        break;
    }
    UNUSED_VARIABLE(bytes_written);
}

/*
 * Handle Handsfree commands received over UART.
 */
void hci_control_hf_handle_command(uint16_t opcode, uint8_t* p_data, uint32_t length)
{
    uint16_t                  handle;
    uint8_t                   hs_cmd;
    int                       num;
    uint8_t                  *p = (uint8_t *)p_data;
    BD_ADDR bd_addr;
    wiced_bt_hfp_hf_scb_t    *p_scb = NULL;

    switch (opcode)
    {
    case HCI_CONTROL_HF_COMMAND_CONNECT:
        STREAM_TO_BDADDR(bd_addr,p);
        wiced_bt_hfp_hf_connect(bd_addr);
        break;

    case HCI_CONTROL_HF_COMMAND_DISCONNECT:
        handle = p[0] | (p[1] << 8);
        wiced_bt_hfp_hf_disconnect(handle);
        break;

    case HCI_CONTROL_HF_COMMAND_OPEN_AUDIO:
        handle = p[0] | (p[1] << 8);
        p_scb = wiced_bt_hfp_hf_get_scb_by_handle (handle);
        if( p_scb )
        {
            // For all HF initiated audio connection establishments for which both sides support the Codec Negotiation feature,
            // the HF shall trigger the AG to establish a Codec Connection. ( Ref HFP Spec 1.7 : Section 4.11.2 )
            if ( (p_scb->peer_feature_mask & WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION) &&
                    (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION) )
            {
                wiced_bt_hfp_hf_at_send_cmd( p_scb, WICED_BT_HFP_HF_CMD_BCC,
                        WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0 );

                // As per spec on receiving AT+BCC command, AG will respond with OK and initiate Codec Connection Setup procedure.
                // While responding AT+BCS=<codec_id> to AG, from profile we will get "WICED_BT_HFP_HFP_CODEC_SET_EVT" event,
                // and based on init_sco_conn flag we will initiate SCO connection request.
                handsfree_ctxt_data.init_sco_conn = WICED_TRUE;
            }
            else
            {
                wiced_bt_sco_remove( handsfree_ctxt_data.sco_index );
                wiced_bt_sco_create_as_initiator( p_scb->peer_addr, &handsfree_ctxt_data.sco_index, (wiced_bt_sco_params_t *) &handsfree_esco_params );
            }
        }
        break;

    case HCI_CONTROL_HF_COMMAND_CLOSE_AUDIO:
        handle = p[0] | (p[1] << 8);
        //handle is not used
        wiced_bt_sco_remove( handsfree_ctxt_data.sco_index );
        break;

    case HCI_CONTROL_HF_COMMAND_TURN_OFF_PCM_CLK:
        wiced_bt_sco_turn_off_pcm_clock();
        break;

    case HCI_CONTROL_HF_COMMAND_BUTTON_PRESS:
        /* send a corresponding AT command */
#ifdef WICED_ENABLE_BT_HSP_PROFILE
        WICED_BT_TRACE("Send AT+CKPD=200\n");
        hci_control_hf_send_at_cmd(handsfree_ctxt_data.rfcomm_handle,"+CKPD",
                WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 200);
#endif
        break;

    default:
        {
            uint8_t *data_ptr = (uint8_t *) wiced_bt_get_buffer(length+1);
            hs_cmd = opcode - HCI_CONTROL_HF_AT_COMMAND_BASE;

            memcpy (data_ptr, p, length);
            data_ptr[length] = 0;                      /* NULL terminate the AT string */
            handle = data_ptr[0] | (data_ptr[1] << 8);
            num = data_ptr[2] | (data_ptr[3] << 8);
            hci_control_hf_at_command (handle,hs_cmd, num, data_ptr+4);
            wiced_bt_free_buffer((void *)data_ptr);
        }
        break;
    }
}

void hci_control_hf_send_at_cmd (uint16_t handle,char *cmd, uint8_t arg_type, uint8_t arg_format, const char *p_arg, int16_t int_arg)
{
    char    buf[WICED_BT_HFP_HF_AT_MAX_LEN + 16];
    char    *p = buf;

    memset (buf, 0, (WICED_BT_HFP_HF_AT_MAX_LEN+16));

    *p++ = 'A';
    *p++ = 'T';

    /* copy result code string */
    memcpy(p,cmd, strlen(cmd));
    p += strlen(cmd);

    if(arg_type == WICED_BT_HFP_HF_AT_SET)
    {
        *p++ = '=';

    }
    else if(arg_type == WICED_BT_HFP_HF_AT_READ)
    {
        *p++ = '?';

    }
    else if(arg_type == WICED_BT_HFP_HF_AT_TEST)
    {
        *p++ = '=';
        *p++ = '?';

    }

    /* copy argument if any */
    if (arg_format == WICED_BT_HFP_HF_AT_FMT_INT)
    {
        p += util_itoa((uint16_t) int_arg, p);
    }
    else if (arg_format == WICED_BT_HFP_HF_AT_FMT_STR)
    {
        utl_strcpy(p, (char *)p_arg);
        p += strlen(p_arg);
    }

    /* finish with \r*/
    *p++ = '\r';

    wiced_bt_hfp_hf_send_at_cmd(handle,buf);
}
void hci_control_hf_at_command (uint16_t handle, uint8_t command, int num, uint8_t* p_data)
{
    switch ( command )
    {
        case HCI_CONTROL_HF_AT_COMMAND_SPK:
            wiced_bt_hfp_hf_notify_volume (handle,
                    WICED_BT_HFP_HF_SPEAKER, num);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_MIC:
            wiced_bt_hfp_hf_notify_volume (handle,
                    WICED_BT_HFP_HF_MIC, num);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_BINP:
            hci_control_hf_send_at_cmd( handle, "+BINP",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 1 );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CHLD:
            wiced_bt_hfp_hf_perform_call_action(handle,
                    WICED_BT_HFP_HF_CALL_ACTION_HOLD_0 + num,(char *)p_data);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_BVRA:
            hci_control_hf_send_at_cmd( handle, "+BVRA",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CMEE:
            hci_control_hf_send_at_cmd( handle, "+CMEE",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_A:
            wiced_bt_hfp_hf_perform_call_action(handle,
                        WICED_BT_HFP_HF_CALL_ACTION_ANSWER,(char *)p_data);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CHUP:
            wiced_bt_hfp_hf_perform_call_action(handle,
                        WICED_BT_HFP_HF_CALL_ACTION_HANGUP,(char *)p_data);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CNUM:
            hci_control_hf_send_at_cmd(handle, "+CNUM",
                        WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CLCC:
            hci_control_hf_send_at_cmd(handle, "+CLCC",
                    WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CIND:
            hci_control_hf_send_at_cmd(handle, "+CIND",
                    WICED_BT_HFP_HF_AT_READ, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0 );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_D:
        case HCI_CONTROL_HF_AT_COMMAND_BLDN:
            wiced_bt_hfp_hf_perform_call_action (handle ,
                                        WICED_BT_HFP_HF_CALL_ACTION_DIAL ,(char *)p_data);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_NREC:
            hci_control_hf_send_at_cmd( handle, "+NREC",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 0 );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_VTS:
            hci_control_hf_send_at_cmd( handle, "+VTS",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0 );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_BTRH:
            hci_control_hf_send_at_cmd(handle, "+BTRH",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, num);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_BIEV:
            hci_control_hf_send_at_cmd(handle, "+BIEV",
                            WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;
        case HCI_CONTROL_HF_AT_COMMAND_BIA:
            hci_control_hf_send_at_cmd(handle, "+BIA",
                            WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char *)p_data, 0);
            break;
    }
}

/* Handle misc command group */
void hci_control_misc_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len )
{
    switch( cmd_opcode )
    {
    case HCI_CONTROL_MISC_COMMAND_GET_VERSION:
        hci_control_misc_handle_get_version();
        break;
    }
}


/* Handle get version command */
void hci_control_misc_handle_get_version( void )
{
    uint8_t   tx_buf[15];
    uint8_t   cmd = 0;

// If this is 20819 or 20820, we do detect the device from hardware
#define RADIO_ID    0x006007c0
#define RADIO_20820 0x80
#define CHIP_20820  20820
#define CHIP_20819  20819
#if (CHIP==CHIP_20819) || (CHIP==CHIP_20820)
    uint32_t chip = CHIP_20819;
    if (*(UINT32*) RADIO_ID & RADIO_20820)
    {
        chip = CHIP_20820;
    }
#else
    uint32_t  chip = CHIP;
#endif

    tx_buf[cmd++] = WICED_SDK_MAJOR_VER;
    tx_buf[cmd++] = WICED_SDK_MINOR_VER;
    tx_buf[cmd++] = WICED_SDK_REV_NUMBER;
    tx_buf[cmd++] = WICED_SDK_BUILD_NUMBER & 0xFF;
    tx_buf[cmd++] = (WICED_SDK_BUILD_NUMBER>>8) & 0xFF;
    tx_buf[cmd++] = chip & 0xFF;
    tx_buf[cmd++] = (chip>>8) & 0xFF;
    tx_buf[cmd++] = (chip>>24) & 0xFF;
    tx_buf[cmd++] = 0; // not used

    /* Send MCU app the supported features */
    tx_buf[cmd++] = HCI_CONTROL_GROUP_HF;

    wiced_transport_send_data( HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd );
}
