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
 * This file implements the HCI command for handsfree
 */

#include "../hci_control.h"
#include "wiced_bt_sco.h"
#include "wiced_timer.h"
#include "string.h"
#include "wiced_transport.h"

void hci_control_hf_handle_command(uint16_t opcode, uint8_t* p_data, uint32_t length);
void hci_control_hf_at_command (uint16_t handle, uint8_t command, int num, uint8_t* p_data);
extern bluetooth_hfp_context_t* get_context_ptr_from_handle(uint16_t handle);

wiced_bt_sco_params_t handsfree_esco_params =
{
        0x000A,             /* Latency: 10 ms ( HS/HF can use EV3, 2-EV3, 3-EV3 ) ( S3 ) */
        HANDS_FREE_SCO_PKT_TYPES,
        BTM_ESCO_RETRANS_POWER, /* Retrans Effort ( At least one retrans, opt for power ) ( S3 ) */
#if (WICED_BT_HFP_HF_WBS_INCLUDED == TRUE)
        WICED_TRUE
#else
        WICED_FALSE
#endif
};

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
    bluetooth_hfp_context_t  *hfp_ptr = NULL;

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
        hfp_ptr = (bluetooth_hfp_context_t *) get_context_ptr_from_handle(handle);
        p_scb = wiced_bt_hfp_hf_get_scb_by_handle(handle);

        if( hfp_ptr && p_scb )
        {
            // For all HF initiated audio connection establishments for which both sides support the Codec Negotiation feature,
            // the HF shall trigger the AG to establish a Codec Connection. ( Ref HFP Spec 1.7 : Section 4.11.2 )
            if ( (p_scb->peer_feature_mask & WICED_BT_HFP_AG_FEATURE_CODEC_NEGOTIATION) &&
                    (p_scb->feature_mask & WICED_BT_HFP_HF_FEATURE_CODEC_NEGOTIATION) )
            {
                // Default we are SCO accepter so before initiating sco connection first we have to remove existing SCO index
                wiced_bt_sco_remove( hfp_ptr->sco_index );

                wiced_bt_hfp_hf_at_send_cmd( p_scb, WICED_BT_HFP_HF_CMD_BCC,
                        WICED_BT_HFP_HF_AT_NONE, WICED_BT_HFP_HF_AT_FMT_NONE, NULL, 0 );

                // As per spec on receiving AT+BCC command, AG will respond with OK and initiate Codec Connection Setup procedure.
                // While responding AT+BCS=<codec_id> to AG, from profile we will get "WICED_BT_HFP_HFP_CODEC_SET_EVT" event,
                // and based on init_sco_conn flag we will initiate SCO connection request.
                hfp_ptr->init_sco_conn = WICED_TRUE;
            }
            else
            {
                wiced_bt_sco_create_as_initiator( hfp_ptr->peer_bd_addr, &hfp_ptr->sco_index, (wiced_bt_sco_params_t *) &handsfree_esco_params );
            }
        }
        break;

    case HCI_CONTROL_HF_COMMAND_CLOSE_AUDIO:
        handle = p[0] | (p[1] << 8);
        {
            bluetooth_hfp_context_t* hfp_ptr = NULL;
            hfp_ptr = (bluetooth_hfp_context_t *) get_context_ptr_from_handle(handle);
            if( hfp_ptr )
            {
                wiced_bt_sco_remove( hfp_ptr->sco_index );
            }
        }
        break;

    case HCI_CONTROL_HF_COMMAND_AUDIO_ACCEPT_CONN:
        {
            int sco_index = p[0] | (p[1] << 8);
            wiced_bool_t flag = p[2];
            if ( flag )
            {
                wiced_bt_sco_accept_connection(sco_index, HCI_SUCCESS, (wiced_bt_sco_params_t *) &handsfree_esco_params);
            }
            else
            {
                WICED_BT_TRACE ("User Reject connection request \n");
            }
        }
        break;

    case HCI_CONTROL_HF_COMMAND_TURN_OFF_PCM_CLK:
        wiced_bt_sco_turn_off_pcm_clock();
        break;

    default:
        hs_cmd = opcode - HCI_CONTROL_HF_AT_COMMAND_BASE;
        p[length] = 0;                              /* NULL terminate the AT string */
        handle = p[0] | (p[1] << 8);
        p += 2;
        num = p[0] | (p[1] << 8);
        p += 2;
        hci_control_hf_at_command (handle,hs_cmd, num, p);

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
        p += utl_itoa((uint16_t) int_arg, p);
    }
    else if (arg_format == WICED_BT_HFP_HF_AT_FMT_STR)
    {
        utl_strcpy(p,(char *)p_arg);
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
                    WICED_BT_HFP_HF_CALL_ACTION_HOLD_0 + num, (char*)p_data);
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
                        WICED_BT_HFP_HF_CALL_ACTION_ANSWER,(char*)p_data);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_CHUP:
            wiced_bt_hfp_hf_perform_call_action(handle,
                        WICED_BT_HFP_HF_CALL_ACTION_HANGUP,(char*)p_data);
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
                                        WICED_BT_HFP_HF_CALL_ACTION_DIAL ,(char*)p_data);
            break;

        case HCI_CONTROL_HF_AT_COMMAND_NREC:
            hci_control_hf_send_at_cmd( handle, "+NREC",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_INT, NULL, 0 );
            break;

        case HCI_CONTROL_HF_AT_COMMAND_VTS:
            hci_control_hf_send_at_cmd( handle, "+VTS",
                    WICED_BT_HFP_HF_AT_SET, WICED_BT_HFP_HF_AT_FMT_STR, (char*)p_data, 0 );
            break;

    }
}

/*
 *  Send User Confirmation Request event through UART
 */
void hci_control_send_sco_confirmation_request_evt( BD_ADDR bda, uint16_t sco_index )
{
    uint8_t buf[8];
    uint8_t *p = &buf[0];

    BDADDR_TO_STREAM(p,bda);

    *(p)++ = sco_index & 0xff;
    *(p)++ = (sco_index >> 8) & 0xff;

    wiced_transport_send_data( HCI_CONTROL_HF_EVENT_AUDIO_CONN_REQ, buf, 8 );
}
