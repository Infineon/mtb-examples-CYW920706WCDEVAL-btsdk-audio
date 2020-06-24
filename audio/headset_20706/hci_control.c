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
 * Headset Sample Application for 20706A2 devices.
 * The sample app performs as a Bluetooth A2DP sink and AVRCP Controller (and Target for absolute volume control).
 * Features demonstrated
 *  - WICED BT AV (A2DP/AVRCP) APIs
 *  - WICED BT GATT APIs
 *  - Handling of the UART WICED protocol
 *  - SDP and GATT descriptor/attribute configuration
 *
 * This application can run as a standalone Headset or in association with ClientControl.
 * When running in standalone mode, it is required that 20706 controls the codec chip, Currently this is not implemented.
 * In association with an ClientControl (apps processor emulation), this app serves to abstract the details of Bluetooth protocols and profiles while allowing ClientControl to deal with the business logic.
 * ClientControl processor is typically connected over UART and can send commands and receive notifications.
 *
 * On startup this demo:
 *  - Initializes the Bluetooth sub system
 *  - Receive NVRAM information from the host
 *
 * Application Instructions
 * *->To use this application in conjunction with ClientControl:
 *  - Build application to produce a downloadable hcd file.
 *  - The HCD file thus generated is downloaded into 20706 RAM.
 *  - Ensure that STANDALONE_HEADSET_APP flag is disabled.
 *
 * *->To use this in stand alone mode:
 *  - Ensure that STANDALONE_HEADSET_APP flag is enabled.
 *  - Button configured to demonstrate multiple functionality.
 *  - Button has 2 states, short press and long press. Both of them have certain features configured.
 *  - Button Short Press Cases:
 *     * Play/Pause music streaming.
 *     * Accept/End incoming call/ongoing call.
 *     * End an outgoing call.
 *  - Button Long Press Cases:
 *     * Last number redialing when in IDLE state.
 *     * Reject an incoming call.
 *     * Put an existing call on hold.
 */

#include "wiced_bt_gatt.h"
#include "wiced_bt_ble.h"
#include "wiced_app_cfg.h"
#include "wiced_memory.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_avrc_defs.h"
#include "wiced_bt_avrc.h"
#include "wiced_power_save.h"
#include "wiced_timer.h"
#include "hci_control.h"
#include "wiced_hal_nvram.h"

#include "wiced_platform.h"
#include "wiced_transport.h"

#include "wiced_app.h"
#include "string.h"
#include "wiced_bt_a2dp_sink.h"
#include "wiced_hal_mia.h"
#include "wiced_bt_stack.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_app_common.h"
#include "wiced_bt_app_hal_common.h"
#include "remote_controller/hci_control_rc.h"
#include "wiced_hal_wdog.h"

/*****************************************************************************
**  Constants
*****************************************************************************/
#define WICED_HS_EIR_BUF_MAX_SIZE               264
#define KEY_INFO_POOL_BUFFER_SIZE   145 //Size of the buffer used for holding the peer device key info
#define KEY_INFO_POOL_BUFFER_COUNT  8  //Correspond's to the number of peer devices

#ifdef STANDALONE_HEADSET_APP
/* Headset App Timer Timeout in seconds  */
#define HEADSET_APP_TIMEOUT_IN_SECONDS                 1

#define BUTTON_PRESSED                         WICED_BUTTON_PRESSED_VALUE


uint32_t app_timer_count = 0;
#endif

/*****************************************************************************
**  Structures
*****************************************************************************/
typedef struct
{
    void    *p_next;
    uint16_t nvram_id;
    uint8_t  chunk_len;
    uint8_t  data[1];
} hci_control_nvram_chunk_t;

/******************************************************
 *               Variables Definitions
 ******************************************************/

uint8_t device_name[255];

wiced_bool_t avrcp_profile_role = AVRCP_TARGET_ROLE;

hci_control_nvram_chunk_t *p_nvram_first = NULL;

/* HS control block */
#if BTA_DYNAMIC_MEMORY == FALSE
hci_control_cb_t  hci_control_cb;
#endif

wiced_transport_buffer_pool_t* transport_pool;   // Trans pool for sending the RFCOMM data to host
wiced_bt_buffer_pool_t*        p_key_info_pool;  //Pool for storing the  key info

static uint32_t  hci_control_proc_rx_cmd( uint8_t *p_data, uint32_t length );
static void      hci_control_transport_tx_cplt_cback(wiced_transport_buffer_pool_t* p_pool);

const wiced_transport_cfg_t transport_cfg =
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
		.buffer_size = 0,
		.buffer_count = 0
	},
	.p_status_handler = NULL,
	.p_data_handler = hci_control_proc_rx_cmd,
	.p_tx_complete_cback = hci_control_transport_tx_cplt_cback
};

extern const uint8_t headset_sdp_db[];

app_context_t app_state_cb;

/******************************************************
 *               Function Declarations
 ******************************************************/
static void     hci_control_handle_bt_stack_init(uint8_t* data, uint32_t length);
static void     hci_control_handle_set_local_name(uint8_t* data, uint32_t length);
static void     hci_control_handle_sdp_db_init(uint8_t* data, uint32_t length);
static void     hci_control_handle_reset_cmd( void );
static void     hci_control_handle_trace_enable( uint8_t *p_data );
static void     hci_control_device_handle_command( uint16_t cmd_opcode, uint8_t* p_data, uint32_t data_len );
static void     hci_control_handle_set_local_bda( uint8_t *p_bda );
static void     hci_control_inquiry( uint8_t enable );
static void     hci_control_handle_set_visibility( uint8_t discoverability, uint8_t connectability );
static void     hci_control_handle_set_pairability ( uint8_t pairing_allowed );
static void     hci_control_handle_enable_disable_coex ( wiced_bool_t enable );
static void     hci_control_handle_read_local_bda( void );
static void     hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing );
static void     hci_control_handle_read_buffer_stats( void );
static void     hci_control_send_device_started_evt( wiced_result_t fw_status, uint8_t status );
static void     hci_control_send_device_error_evt( uint8_t fw_error_code, uint8_t app_error_code );
static void     hci_control_send_pairing_completed_evt( uint8_t status, wiced_bt_device_address_t bdaddr );
static void     hci_control_send_user_confirmation_request_evt( BD_ADDR bda, uint32_t numeric_value );
static void     hci_control_send_encryption_changed_evt( uint8_t encrypted, wiced_bt_device_address_t bdaddr );
static wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

#ifdef STANDALONE_HEADSET_APP
static void                     headset_load_keys_to_addr_resolution_db(void);
static wiced_bool_t             headset_save_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t             headset_read_link_keys(wiced_bt_device_link_keys_t *p_keys);
static wiced_bool_t             headset_delete_link_keys(void);
static wiced_bool_t             headset_get_paired_host_info(uint8_t* bda);

static void                     headset_interrupt_handler( void *user_data, uint8_t value );
static void                     headset_app_timer( uint32_t arg );
#endif

extern void wiced_bt_rc_target_initiate_close( void );
extern wiced_result_t wiced_bt_avrc_ct_cleanup( void );
extern void avrc_app_init( void );

/******************************************************
 *               Function Definitions
 ******************************************************/

/*
 *  hci_control_init
 */
void hci_control_init( void )
{
    wiced_result_t ret = WICED_BT_ERROR;
    memset( &hci_control_cb, 0, sizeof( hci_control_cb ) );

    wiced_transport_init( &transport_cfg );

#ifdef WICED_BT_TRACE_ENABLE

#ifdef STANDALONE_HEADSET_APP
    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);

#else
    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
     wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif
#endif

    WICED_BT_TRACE( "HEADSET APP START\n" );
    ret = wiced_bt_stack_init(hci_control_management_callback, &wiced_bt_cfg_settings, wiced_app_cfg_buf_pools);
    if( ret != WICED_BT_SUCCESS )
    {
        WICED_BT_TRACE("wiced_bt_stack_init returns error: %d\n", ret);
        return;
    }

    /* Configure Audio buffer */
    wiced_audio_buffer_initialize (wiced_bt_audio_buf_config);

}

#ifdef STANDALONE_HEADSET_APP
void state_button_action( wiced_bool_t button_press_state );
void headset_set_input_interrupt(void)
{

    wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_BUTTON, headset_interrupt_handler, NULL );
    wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_BOTH_EDGE ), WICED_GPIO_BUTTON_DEFAULT_STATE );

    /* Starting the app timers, seconds timer and milliseconds timer  */
    wiced_bt_app_start_timer( HEADSET_APP_TIMEOUT_IN_SECONDS, 0, headset_app_timer, NULL );
}

/* The function invoked on timeout of app seconds timer. */
void headset_app_timer( uint32_t arg )
{
    app_timer_count++;
}

void headset_interrupt_handler( void *user_data, uint8_t value )
{
    static uint32_t button_pushed_time = 0;
    uint8_t data[2];

    if ( wiced_hal_gpio_get_pin_input_status(WICED_GPIO_BUTTON) == BUTTON_PRESSED )
        {
            button_pushed_time = app_timer_count;
        }
        else if ( button_pushed_time != 0 )
        {
            if ( app_timer_count - button_pushed_time > 5 )
            {
                 WICED_BT_TRACE( "long press\n");
                 state_button_action( BUTTON_STATE_LONG_PRESS );

            }
            else
            {
                 WICED_BT_TRACE( "short press\n" );
                 state_button_action( BUTTON_STATE_SHORT_PRESS );
            }
        }
}

void state_button_action( uint8_t button_press_state )
{
    uint8_t data[2];

    switch( app_state_cb.state )
    {
        case IDLE:
        {
            if( button_press_state == BUTTON_STATE_LONG_PRESS )
            {
                WICED_BT_TRACE("Re-dial last number\n");
                hci_control_hf_at_command ( app_state_cb.hands_free_handle, HCI_CONTROL_HF_AT_COMMAND_BLDN, (int)NULL, NULL );
            }
            else //short press
            {
                /* check the connection status also. if not connected do nothing */
                if( app_state_cb.audio_stream_state == APP_STREAM_STATE_STOPPED )
                {
                     WICED_BT_TRACE("A2DP streaming started!\n");
                     data[0] = app_state_cb.remote_control_handle & 0xff;                        //handle
                     data[1] = ( app_state_cb.remote_control_handle >> 8 ) & 0xff;
                     hci_control_avrc_handle_ctrlr_command( HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY, data, sizeof(uint16_t) );
                     /* maintain local app state */
                     app_state_cb.state = A2DP;
                     app_state_cb.audio_stream_state = APP_STREAM_STATE_PLAYING;
                }
            }
        }
        break;

        case A2DP:
        {
            if( button_press_state == BUTTON_STATE_LONG_PRESS )
            {
                 // do nothing
            }
            else // short press
            {
                if( app_state_cb.audio_stream_state == APP_STREAM_STATE_PLAYING )
                {
                     WICED_BT_TRACE("A2DP streaming paused!\n");
                     data[0] = app_state_cb.remote_control_handle & 0xff;                        //handle
                     data[1] = ( app_state_cb.remote_control_handle >> 8 ) & 0xff;
                     hci_control_avrc_handle_ctrlr_command( HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE, data, sizeof(uint16_t) );
                     /* maintain local app state */
                     app_state_cb.state = IDLE;
                     app_state_cb.audio_stream_state = APP_STREAM_STATE_STOPPED;
                }
            }
        }
        break;

        case HFP:
        {
            if( button_press_state == BUTTON_STATE_LONG_PRESS )
            {
                if( app_state_cb.call_state == HFP_STATE_INCOMING_CALL )
                {
                     WICED_BT_TRACE("End an incoming call\n");
                     hci_control_hf_at_command ( app_state_cb.hands_free_handle, HCI_CONTROL_HF_AT_COMMAND_CHUP, (int)NULL, NULL );
                }
                else if( app_state_cb.call_state == HFP_STATE_CALL_IN_PROGRESS )
                {
                     WICED_BT_TRACE("Put the active call on hold\n");
                     hci_control_hf_at_command ( app_state_cb.hands_free_handle, HCI_CONTROL_HF_AT_COMMAND_CHLD, 2, NULL );
                }
            }
            else //short press
            {
                if( app_state_cb.call_state == HFP_STATE_INCOMING_CALL )
                {
                     WICED_BT_TRACE("Incoming call- Accept\n");
                     hci_control_hf_at_command ( app_state_cb.hands_free_handle, HCI_CONTROL_HF_AT_COMMAND_A, (int)NULL, NULL );
                }
                else if( app_state_cb.call_state == HFP_STATE_CALL_IN_PROGRESS )
                {
                     WICED_BT_TRACE("End the on going call\n");
                     hci_control_hf_at_command ( app_state_cb.hands_free_handle, HCI_CONTROL_HF_AT_COMMAND_CHUP, (int)NULL, NULL );
                }
                else if( app_state_cb.call_state == HFP_STATE_OUTGOING_CALL )
                {
                     WICED_BT_TRACE("End an out going call\n");
                     hci_control_hf_at_command ( app_state_cb.hands_free_handle, HCI_CONTROL_HF_AT_COMMAND_CHUP, (int)NULL, NULL );
                }
            }
        }
        break;
    }
}
#endif

/*
 *  Process all HCI packet from
 */

void hci_control_hci_packet_cback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
    // send the trace
    wiced_transport_send_hci_trace( NULL, type, length, p_data  );
#endif

    if ( !test_command.test_executing )
        return;

    // If executing test command, need to send Command Complete event back to host app
    if( ( type == HCI_TRACE_EVENT ) && ( length >= 6 ) )
    {
       // hci_control_handle_hci_test_event( p_data, length );
    }
}

/*
 *  Prepare extended inquiry response data.  Current version publishes audio sink
 *  services.
 */
void headset_write_eir( char* headset_device_name, uint8_t name_length  )
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;
    uint8_t *p_tmp;
    char* device_name;
    uint8_t nb_uuid = 0;

    pBuf = (uint8_t*)wiced_bt_get_buffer( WICED_HS_EIR_BUF_MAX_SIZE );
    memset(pBuf, 0, WICED_HS_EIR_BUF_MAX_SIZE);
    WICED_BT_TRACE( "headset_write_eir %x\n", pBuf );

    if ( !pBuf )
    {
        return;
    }
    p = pBuf;

    if( headset_device_name != NULL)
    {
        device_name = headset_device_name;
        length = name_length;
    }
    else
    {
        device_name = (char*) wiced_bt_cfg_settings.device_name;
        length = strlen( (char *)device_name);
    }

    UINT8_TO_STREAM(p, length + 1);
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_LOCAL_NAME_TYPE); // EIR type full name
    memcpy( p, device_name, length );
    p += length;

    p_tmp = p;      // We don't now the number of UUIDs for the moment
    p++;
    UINT8_TO_STREAM(p, BT_EIR_COMPLETE_16BITS_UUID_TYPE);  // EIR type full list of 16 bit service UUIDs

    UINT16_TO_STREAM(p, UUID_SERVCLASS_AUDIO_SINK);       nb_uuid++;

    UINT16_TO_STREAM(p, UUID_SERVCLASS_HF_HANDSFREE);     nb_uuid++;

    /* Now, we can update the UUID Tag's length */
    UINT8_TO_STREAM(p_tmp, (nb_uuid * LEN_UUID_16) + 1);

    // Last Tag
    UINT8_TO_STREAM(p, 0x00);

    // print EIR data
    wiced_bt_trace_array( "EIR :", ( uint8_t* )( pBuf+1 ), MIN( p-( uint8_t* )pBuf,100 ) );
    if(wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) ) != WICED_SUCCESS)
        WICED_BT_TRACE("dev_write_eir failed\n");

    return;
}

/*
 *  Connection status callback function triggered from the stack on ACL connection/disconnection.
 *  This function sends HCI_CONTROL_EVENT_CONNECTION_STATUS event to UART
 */
void hci_control_connection_status_callback (wiced_bt_device_address_t bd_addr, uint8_t *p_features, wiced_bool_t is_connected, uint16_t handle, wiced_bt_transport_t transport, uint8_t reason)
{
    wiced_result_t result = WICED_ERROR;
    uint8_t event_data[2];

    //Build event payload
    event_data[0] = is_connected;
    event_data[1] = reason;

#ifdef STANDALONE_HEADSET_APP
    if (!is_connected)
    {
        app_state_cb.is_originator = WICED_FALSE;
    }
#endif

    result = wiced_transport_send_data( HCI_CONTROL_EVENT_CONNECTION_STATUS, event_data, 2 );

    WICED_BT_TRACE("%s  is_connected:%d reason:%x result:%d\n", __FUNCTION__, is_connected, reason, result );
}

wiced_result_t headset_post_bt_init(void)
{
    wiced_result_t result = WICED_BT_ERROR;
    wiced_bool_t ret = WICED_FALSE;

    ret = wiced_bt_sdp_db_init( ( uint8_t * )headset_sdp_db, wiced_app_cfg_sdp_record_get_size());
    if( ret != TRUE )
    {
        WICED_BT_TRACE("%s Failed to Initialize SDP databse\n", __FUNCTION__);
        return WICED_BT_ERROR;
    }

    result = wiced_bt_dev_register_connection_status_change( hci_control_connection_status_callback );
    WICED_BT_TRACE ("bt_audio_management_callback registering acl_change callback result: 0x%x\n", result);

    /* start a2dp*/
    result = headset_a2dp_init();
    if(result != WICED_BT_SUCCESS )
    {
        WICED_BT_TRACE("%s Failed to Initialize A2DP\n", __FUNCTION__);
        return result;
    }

    headset_remote_control_init();

    handsfree_hfp_init();

#if (WICED_APP_LE_INCLUDED == TRUE)
    hci_control_le_enable();
#endif

    return result;
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t hci_control_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t                     result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t              dev_status;
    wiced_bt_dev_ble_pairing_info_t   *p_pairing_info;
    wiced_bt_dev_encryption_status_t  *p_encryption_status;
    int                                bytes_written, bytes_read;
    int                                nvram_id;
    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    uint8_t                             pairing_result;
    wiced_bt_local_identity_keys_t     *p_identity_keys;

#if (WICED_HCI_TRANSPORT == WICED_HCI_TRANSPORT_UART)
    WICED_BT_TRACE( "headset bluetooth management callback event: 0x%02x\n", event );
#endif

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:
            if( p_event_data->enabled.status != WICED_BT_SUCCESS )
            {
                WICED_BT_TRACE("arrived with failure\n");
            }
            else
            {
                headset_post_bt_init();

#ifdef HCI_TRACE_OVER_TRANSPORT
            // Disable while streaming audio over the uart.
            wiced_bt_dev_register_hci_trace( hci_control_hci_packet_cback );
#endif

            // Creating a buffer pool for holding the peer devices's key info
            p_key_info_pool = wiced_bt_create_pool( KEY_INFO_POOL_BUFFER_SIZE,
                    KEY_INFO_POOL_BUFFER_COUNT );

            if (p_key_info_pool == NULL)
                {
                    WICED_BT_TRACE("Err: wiced_bt_create_pool failed\n");
                }
            }

#ifdef STANDALONE_HEADSET_APP
        {
            wiced_bt_device_address_t bda;
            /* to work with stand alone */
            wiced_bt_dev_set_local_name((char *)wiced_bt_cfg_settings.device_name);
            wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE, wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_window, wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval);
            wiced_bt_dev_set_connectability(BTM_CONNECTABLE, wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_window, wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_interval);

            /* maintain local application state */
            app_state_cb.state = IDLE;
            app_state_cb.audio_stream_state = APP_STREAM_STATE_STOPPED;
            app_state_cb.call_state = HFP_STATE_IDLE;

            wiced_bt_app_hal_init();

            headset_set_input_interrupt();

            if (headset_get_paired_host_info(bda))
            {
                app_state_cb.is_originator = WICED_TRUE;
                wiced_bt_hfp_hf_connect(bda);
            }
        }
#endif

            wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, NULL, 0 );
            break;

        case BTM_DISABLED_EVT:
            hci_control_send_device_error_evt( p_event_data->disabled.reason, 0 );
            break;

        case BTM_PIN_REQUEST_EVT:
            WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
            //wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr, WICED_BT_SUCCESS, WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            // If this is just works pairing, accept. Otherwise send event to the MCU to confirm the same value.
            if (p_event_data->user_confirmation_request.just_works)
            {
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr );
            }
            else
            {
                hci_control_send_user_confirmation_request_evt(p_event_data->user_confirmation_request.bd_addr, p_event_data->user_confirmation_request.numeric_value );
            }
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            WICED_BT_TRACE("PassKey Notification. BDA %B, Key %d \n", p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            hci_control_send_user_confirmation_request_evt(p_event_data->user_passkey_notification.bd_addr, p_event_data->user_passkey_notification.passkey );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            /* Use the default security for BR/EDR*/
            WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data     = WICED_FALSE;
//            p_event_data->pairing_io_capabilities_br_edr_request.auth_req     = BTM_AUTH_ALL_PROFILES_NO;
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
            WICED_BT_TRACE( "Pairing Result: %d\n", pairing_result );
            hci_control_send_pairing_completed_evt( pairing_result, p_event_data->pairing_complete.bd_addr );
            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            p_encryption_status = &p_event_data->encryption_status;

            WICED_BT_TRACE( "Encryption Status:(%B) res:%d\n", p_encryption_status->bd_addr, p_encryption_status->result );

#if (WICED_APP_LE_SLAVE_CLIENT_INCLUDED == TRUE)
            if (p_encryption_status->transport == BT_TRANSPORT_LE)
                le_slave_encryption_status_changed(p_encryption_status);
#endif
            hci_control_send_encryption_changed_evt( p_encryption_status->result, p_encryption_status->bd_addr );
            break;

        case BTM_SECURITY_REQUEST_EVT:
            if ( hci_control_cb.pairing_allowed )
            {
                wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
            }
            else
            {
                // Pairing not allowed, return error
                result = WICED_BT_ERROR;
            }
            WICED_BT_TRACE( "Security Request Event, Pairing allowed %d\n", hci_control_cb.pairing_allowed );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        {
#ifdef STANDALONE_HEADSET_APP

            WICED_BT_TRACE("link keys update:%d\n", p_event_data->paired_device_link_keys_update.key_data.le_keys_available_mask);
            headset_save_link_keys(&p_event_data->paired_device_link_keys_update);
            wiced_bt_dev_add_device_to_address_resolution_db(&p_event_data->paired_device_link_keys_update, p_event_data->paired_device_link_keys_update.key_data.ble_addr_type);

#else
            /* Check if we already have information saved for this bd_addr */
            if ( ( nvram_id = hci_control_find_nvram_id( p_event_data->paired_device_link_keys_update.bd_addr, BD_ADDR_LEN ) ) == 0)
            {
                // This is the first time, allocate id for the new memory chunk
                nvram_id = hci_control_alloc_nvram_id( );
                WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
            }
            bytes_written = hci_control_write_nvram( nvram_id, sizeof( wiced_bt_device_link_keys_t ), &p_event_data->paired_device_link_keys_update, WICED_FALSE );

            WICED_BT_TRACE("\nNVRAM write:id:%d bytes:%d dev: [%B]\n", nvram_id, bytes_written, p_event_data->paired_device_link_keys_update.bd_addr);
#endif
        }
            break;

        case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        {
            /* read existing key from the NVRAM  */
#ifdef STANDALONE_HEADSET_APP

           WICED_BT_TRACE("link keys request:%d\n", p_event_data->paired_device_link_keys_request.key_data.le_keys_available_mask);

           if (headset_read_link_keys(&p_event_data->paired_device_link_keys_request))
           {
               result = WICED_BT_SUCCESS;
               WICED_BT_TRACE("Key retrieval success\n");
           }
           else
           {
               result = WICED_BT_ERROR;
               WICED_BT_TRACE("Key retrieval failure\n");
           }

#else
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
#endif
         }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
        /* Request to store newly generated local identity keys to NVRAM */
        /* Store the keys only if used in Stand alone mode */
#ifdef STANDALONE_HEADSET_APP
           /* save keys to NVRAM */
           p_identity_keys = &p_event_data->local_identity_keys_update;
           wiced_hal_write_nvram (HEADSET_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), (uint8_t *)p_identity_keys, &result);
           WICED_BT_TRACE("local keys save to NVRAM result: %d \n", result);
#endif

            break;


        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            /*
             * Request to restore local identity keys from NVRAM
             * (requested during Bluetooth start up)
             * */
        /* Read the keys only in stand alone mode */
#ifdef STANDALONE_HEADSET_APP
            /* read keys from NVRAM */
           p_identity_keys = &p_event_data->local_identity_keys_request;
           wiced_hal_read_nvram(HEADSET_LOCAL_KEYS_VS_ID, sizeof(wiced_bt_local_identity_keys_t), (uint8_t *)p_identity_keys, &result);
           WICED_BT_TRACE("local keys read from NVRAM result: %d \n",  result);

#else
            result = WICED_BT_NO_RESOURCES;
#endif
            break;

        case BTM_BLE_SCAN_STATE_CHANGED_EVT:
            //hci_control_le_scan_state_changed( p_event_data->ble_scan_state_changed );
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            //hci_control_le_advert_state_changed( p_event_data->ble_advert_state_changed );
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
            WICED_BT_TRACE( "Power mgmt status event: bd ( %B ) status:%d hci_status:%d\n", p_power_mgmt_notification->bd_addr, \
                    p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
            break;

        case BTM_SCO_CONNECTED_EVT:
        case BTM_SCO_DISCONNECTED_EVT:
        case BTM_SCO_CONNECTION_REQUEST_EVT:
        case BTM_SCO_CONNECTION_CHANGE_EVT:
            hf_sco_management_callback(event, p_event_data);
            break;

        default:
            result = WICED_BT_USE_DEFAULT_SECURITY;
            break;
    }
    return result;
}

void hci_control_handle_get_version(void)
{
    uint8_t   tx_buf[20];
    uint8_t   cmd = 0;
    uint32_t  chip = CHIP;

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
    tx_buf[cmd++] = HCI_CONTROL_GROUP_GATT;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AUDIO_SINK;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_HF;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AVRC_CONTROLLER;
    tx_buf[cmd++] = HCI_CONTROL_GROUP_AVRC_TARGET;

    wiced_transport_send_data(HCI_CONTROL_MISC_EVENT_VERSION, tx_buf, cmd);
}
/*
 * Handle received command over UART. Please refer to the WICED Smart Ready
 * Software User Manual (WICED-Smart-Ready-SWUM100-R) for details on the
 * HCI UART control protocol.
*/
static uint32_t hci_control_proc_rx_cmd( uint8_t *p_buffer, uint32_t length )
{
    uint16_t opcode;
    uint16_t payload_len;
    uint8_t *p_data = p_buffer;
    uint8_t  buffer_processed = WICED_TRUE;

    if ( !p_buffer )
    {
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    //Expected minimum 4 byte as the wiced header
    if(( length < 4 ) || (p_data == NULL))
    {
        WICED_BT_TRACE("invalid params\n");
        wiced_transport_free_buffer( p_buffer );
        return HCI_CONTROL_STATUS_INVALID_ARGS;
    }

    STREAM_TO_UINT16(opcode, p_data);       // Get OpCode
    STREAM_TO_UINT16(payload_len, p_data);  // Gen Payload Length

    WICED_BT_TRACE("[%s] cmd_opcode 0x%02x\n", __FUNCTION__, opcode);

    if (opcode == HCI_CONTROL_MISC_EVENT_VERSION)
    {
        hci_control_handle_get_version();
    }
    switch((opcode >> 8) & 0xff)
    {
    case HCI_CONTROL_GROUP_DEVICE:
        hci_control_device_handle_command( opcode, p_data, payload_len );
        break;

#if (WICED_APP_LE_INCLUDED == TRUE)
    case HCI_CONTROL_GROUP_LE:
    case HCI_CONTROL_GROUP_GATT:
        hci_control_le_handle_command( opcode, p_data, payload_len );
        break;
#endif

#if (WICED_APP_AUDIO_SINK_INCLUDED == TRUE)
    case HCI_CONTROL_GROUP_AUDIO_SINK:
        hci_control_audio_handle_command( opcode, p_data, payload_len );
        break;
#endif

    case HCI_CONTROL_GROUP_AVRC_TARGET:
        //hci_control_avrc_handle_command( opcode, p_data, payload_len );
        break;

    case HCI_CONTROL_GROUP_AVRC_CONTROLLER:
        hci_control_avrc_handle_ctrlr_command(opcode, p_data, payload_len);
        break;

    case HCI_CONTROL_GROUP_HF:
        hci_control_hf_handle_command ( opcode, p_data, payload_len );
        break;

    default:
        WICED_BT_TRACE( "unknown class code (opcode:%x)\n", opcode);
        break;
    }
    if (buffer_processed)
    {
        // Freeing the buffer in which data is received
        wiced_transport_free_buffer( p_buffer );
    }

    return HCI_CONTROL_STATUS_SUCCESS;
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
        {
            int nvram_id = 0;
            BD_ADDR bd_addr;
            uint8_t* p = p_data;

            STREAM_TO_UINT16(nvram_id, p);
            memcpy(bd_addr, p, sizeof(uint8_t) * 6);

            WICED_BT_TRACE("[%s] bd_addr = %B\n",__func__,bd_addr);

            if ((nvram_id = hci_control_find_nvram_id( bd_addr, BD_ADDR_LEN)) == 0)
            {
                // This is the first time, allocate id for the new memory chunk
                nvram_id = hci_control_alloc_nvram_id();
                WICED_BT_TRACE( "Allocated NVRAM ID:%d\n", nvram_id );
            }

            bytes_written = hci_control_write_nvram(nvram_id, data_len - 2, &p_data[2], WICED_TRUE);
            WICED_BT_TRACE( "NVRAM write: %d dev: [%B]\n", bytes_written , &p_data[2]);
            break;
        }
    case HCI_CONTROL_COMMAND_DELETE_NVRAM_DATA:
        hci_control_delete_nvram( p_data[0] | ( p_data[1] << 8 ), WICED_TRUE );
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

    case HCI_CONTROL_COMMAND_ENABLE_COEX:
        hci_control_handle_enable_disable_coex( TRUE );
        break;

    case HCI_CONTROL_COMMAND_DISABLE_COEX:
        hci_control_handle_enable_disable_coex( FALSE );
        break;

    case HCI_CONTROL_COMMAND_READ_LOCAL_BDA:
        hci_control_handle_read_local_bda();
        break;

    case HCI_CONTROL_COMMAND_USER_CONFIRMATION:
        hci_control_handle_user_confirmation( p_data, p_data[6] );
        break;

    case HCI_CONTROL_COMMAND_READ_BUFF_STATS:
        hci_control_handle_read_buffer_stats ();
        break;

    case HCI_CONTROL_COMMAND_SET_LOCAL_NAME:
        hci_control_handle_set_local_name(p_data, data_len);
        break;

    default:
        WICED_BT_TRACE( "??? Unknown command code\n" );
        break;
    }
}


/*
 * handle reset command from UART
 */
void hci_control_handle_reset_cmd( void )
{
    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );

    // trip watch dog now.
    wiced_hal_wdog_reset_system( );
}

/*
 * handle command from UART to configure traces
 */
void hci_control_handle_trace_enable( uint8_t *p_data )
{
    uint8_t hci_trace_enable = *p_data++;
    wiced_debug_uart_types_t route_debug = (wiced_debug_uart_types_t)*p_data;

    WICED_BT_TRACE("HCI Traces:%d DebugRoute:%d\n", hci_trace_enable, route_debug);

    if ( hci_trace_enable )
    {
        /* Register callback for receiving hci traces */
        // Disable while streaming audio over the uart.
        wiced_bt_dev_register_hci_trace( hci_control_hci_packet_cback );
    }
    else
    {
        wiced_bt_dev_register_hci_trace( NULL);
    }
    wiced_set_debug_uart( route_debug );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 * handle command to set local Bluetooth device address
 */
void hci_control_handle_set_local_bda( uint8_t *p_bda)
{
    BD_ADDR bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_set_local_bdaddr( bd_addr, BLE_ADDR_PUBLIC );

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle read buffer statistics
 */
void hci_control_handle_read_buffer_stats( void )
{
    wiced_bt_buffer_statistics_t buff_stats[ wiced_bt_get_number_of_buffer_pools() ];
    wiced_result_t result;
    uint8_t i;

    result = wiced_bt_get_buffer_usage( buff_stats, sizeof( buff_stats ) );

    if( result == WICED_BT_SUCCESS )
    {
        // Print out the stats to trace
        WICED_BT_TRACE( "Buffer usage statistics:\n");

        for( i=0; i < wiced_bt_get_number_of_buffer_pools(); i++) {
            WICED_BT_TRACE("pool_id:%d size:%d curr_cnt:%d max_cnt:%d total:%d\n",
                           buff_stats[i].pool_id, buff_stats[i].pool_size,
                           buff_stats[i].current_allocated_count, buff_stats[i].max_allocated_count,
                           buff_stats[i].total_count);
        }

        // Return the stats via WICED-HCI
        wiced_transport_send_data( HCI_CONTROL_EVENT_READ_BUFFER_STATS, (uint8_t*)&buff_stats, sizeof( buff_stats ) );
    }
    else {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_FAILED );
    }
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
        WICED_BT_TRACE( "inquiry complete \n");
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
        while ( ( p_eir_data != NULL ) && ( len = *p_eir_data ) != 0 )
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
}

/*
 * Set local name
 *
 */
void hci_control_handle_set_local_name(uint8_t* p_data, uint32_t data_len)
{
    /* the local name is only from the second byte
     * The first bytes provides the length.
     */
    uint8_t name[250];

    memset(name, 0, sizeof(name));
    memcpy(name, &p_data[1], p_data[0] - 1);
    wiced_bt_dev_set_local_name((char*)name);
    headset_write_eir((char*)&p_data[1], p_data[0] - 1);
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
void hci_control_handle_set_pairability ( uint8_t pairing_allowed )
{
    uint8_t                   status = HCI_CONTROL_STATUS_SUCCESS;
    hci_control_nvram_chunk_t *p1;

    if ( hci_control_cb.pairing_allowed != pairing_allowed )
    {
        if ( pairing_allowed )
        {
            // Check if key buffer pool has buffer available. If not, cannot enable pairing until nvram entries are deleted
            if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
            {
                pairing_allowed = 0; //The key buffer pool is full therefore we cannot allow pairing to be enabled
                status = HCI_CONTROL_STATUS_OUT_OF_MEMORY;
            }
            else
            {
                wiced_bt_free_buffer( p1 );
            }
        }

    hci_control_cb.pairing_allowed = pairing_allowed;
        wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
        WICED_BT_TRACE( " Set the pairing allowed to %d \n", hci_control_cb.pairing_allowed );
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, status );
}

/*
 *  Handle Enable/Disable Coex command received over UART
 */
static void hci_control_handle_enable_disable_coex ( wiced_bool_t enable )
{
    if ( enable )
    {
        WICED_BT_TRACE( "Enabling Coex\n" );
        wiced_bt_coex_enable();
    }
    else
    {
        WICED_BT_TRACE( "Disabling Coex\n" );
        wiced_bt_coex_disable();
    }

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Handle Get Local BDA command received over UART
 */
void hci_control_handle_read_local_bda( void )
{
    wiced_bt_device_address_t bda = { 0 };

    wiced_bt_dev_read_local_addr(bda);
    WICED_BT_TRACE("Local Bluetooth Address: [%B]\n", bda);

    wiced_transport_send_data( HCI_CONTROL_EVENT_READ_LOCAL_BDA, (uint8_t*)bda , 6 );
}

/*
 *  Handle User Confirmation received over UART
 */
void hci_control_handle_user_confirmation( uint8_t *p_bda, uint8_t accept_pairing )
{
    wiced_bt_device_address_t bd_addr;

    STREAM_TO_BDADDR(bd_addr,p_bda);
    wiced_bt_dev_confirm_req_reply( accept_pairing == WICED_TRUE ? WICED_BT_SUCCESS : WICED_BT_ERROR, bd_addr);

    hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
}

/*
 *  Send Device Started event through UART
 */
void hci_control_send_device_started_evt( wiced_result_t fw_status, uint8_t app_status )
{
    wiced_result_t* result;
    uint8_t event_data[sizeof(wiced_result_t) + 1 ];
    result = (wiced_result_t *)&event_data;
    *result = fw_status;
    event_data[sizeof(wiced_result_t)] = app_status;
    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_STARTED, event_data, sizeof(wiced_result_t) + 1 );

    WICED_BT_TRACE( "maxLinks:%d maxChannels:%d maxpsm:%d rfcom max links%d, rfcom max ports:%d\n",
                    wiced_bt_cfg_settings.l2cap_application.max_links,
                    wiced_bt_cfg_settings.l2cap_application.max_channels,
                    wiced_bt_cfg_settings.l2cap_application.max_psm,
                    wiced_bt_cfg_settings.rfcomm_cfg.max_links,
                    wiced_bt_cfg_settings.rfcomm_cfg.max_ports );
}

/*
 *  Send Device Error event through UART
 */
void hci_control_send_device_error_evt( uint8_t fw_error_code, uint8_t app_error_code )
{
    uint8_t event_data[] = { 0, 0 };

    event_data[0] = app_error_code;
    event_data[1] = fw_error_code;

    WICED_BT_TRACE( "[hci_control_send_device_error_evt]  app_error_code=0x%02x  fw_error_code=0x%02x\n", event_data[0], event_data[1] );

    wiced_transport_send_data( HCI_CONTROL_EVENT_DEVICE_ERROR, event_data, 2 );
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
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(uint8_t);
    uint8_t event_data[cmd_size];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = status;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    WICED_BT_TRACE( "pairing complete evt: %B as %B status %d\n", bdaddr, &event_data[1], status );

    wiced_transport_send_data( HCI_CONTROL_EVENT_PAIRING_COMPLETE, event_data, cmd_bytes );
}

/*
 *  Send User Confirmation Request event through UART
 */
void hci_control_send_user_confirmation_request_evt( BD_ADDR bda, uint32_t numeric_value )
{
    uint8_t buf[10];
    int i;
    uint8_t *p = &buf[6];
    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        buf[i] = bda[BD_ADDR_LEN - 1 - i];
    *p++ = numeric_value & 0xff;
    *p++ = (numeric_value >> 8) & 0xff;
    *p++ = (numeric_value >> 16) & 0xff;
    *p++ = (numeric_value >> 24) & 0xff;
    wiced_transport_send_data( HCI_CONTROL_EVENT_USER_CONFIRMATION, buf, 10 );
}

/*
 *  Send Encryption Changed event through UART
 */
void hci_control_send_encryption_changed_evt( uint8_t encrypted ,  wiced_bt_device_address_t bdaddr )
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(uint8_t);
    uint8_t event_data[cmd_size];
    int     cmd_bytes = 0;

    event_data[cmd_bytes++] = encrypted;

    for ( i = 0 ; i < BD_ADDR_LEN; i++ )                     // bd address
        event_data[cmd_bytes++] = bdaddr[BD_ADDR_LEN - 1 - i];

    wiced_transport_send_data( HCI_CONTROL_EVENT_ENCRYPTION_CHANGED, event_data, cmd_bytes );
}

wiced_result_t hci_control_audio_send_codec_configured( wiced_bt_a2dp_codec_info_t codec_info)
{
    int i=0;
    const int cmd_size = sizeof(uint8_t) + sizeof(wiced_bt_a2d_sbc_cie_t);
    uint8_t event_data[cmd_size];

    WICED_BT_TRACE( "[%s]\n", __FUNCTION__);


        event_data[i++] = codec_info.codec_id;
        event_data[i++] = codec_info.cie.sbc.samp_freq;
        event_data[i++] = codec_info.cie.sbc.ch_mode;
        event_data[i++] = codec_info.cie.sbc.block_len;
        event_data[i++] = codec_info.cie.sbc.num_subbands;
        event_data[i++] = codec_info.cie.sbc.alloc_mthd;
        event_data[i++] = codec_info.cie.sbc.max_bitpool;
        event_data[i++] = codec_info.cie.sbc.min_bitpool;

        return wiced_transport_send_data( HCI_CONTROL_AUDIO_SINK_EVENT_CODEC_CONFIGURED, event_data, cmd_size );
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t);
    uint8_t event_data[cmd_size];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( i = 0; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++]   = ( handle >> 8 ) & 0xff;

        //event_data[i] = wiced_bt_rc_target_is_peer_absolute_volume_capable( );
        return wiced_transport_send_data( HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTED, event_data, cmd_size );
    }
    else
    {
        return wiced_transport_send_data( HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTION_FAILED, NULL, 0 );
    }
}

/*
 *  send audio disconnect complete event to UART
 */
wiced_result_t hci_control_audio_send_disconnect_complete( uint16_t handle, uint8_t status, uint8_t reason )
{
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] %04x status %d reason %d\n", __FUNCTION__, handle, status, reason );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;
    event_data[2] = status;                                 // status
    event_data[3] = reason;                                 // reason(1 byte)

    return wiced_transport_send_data( HCI_CONTROL_AUDIO_SINK_EVENT_DISCONNECTED, event_data, 4 );
}

/*  send audio start Indication to UART */
wiced_result_t hci_control_audio_send_start_indication( uint16_t handle, uint8_t label )
{
    uint8_t event_data[4];
    WICED_BT_TRACE( "[%s] handle %04x, Label:%x \n", __FUNCTION__, handle , label);
    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;
    event_data[2] = label;

#ifdef STANDALONE_HEADSET_APP
    /* start response is sent for stand-alone mode only */
    event_data[3] = A2D_SUCCESS;
    return a2dp_app_hci_control_start_rsp(event_data,4);
#else
    return wiced_transport_send_data( HCI_CONTROL_AUDIO_SINK_EVENT_START_IND, event_data, 3);
#endif
}

/*
 *  send audio connect complete event to UART
 */
wiced_result_t hci_control_audio_send_started_stopped( uint16_t handle, wiced_bool_t started )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data(started ? HCI_CONTROL_AUDIO_SINK_EVENT_STARTED : HCI_CONTROL_AUDIO_SINK_EVENT_STOPPED, event_data, 2);
}

/*
 *  send AVRC event to UART
 */
wiced_result_t hci_control_send_avrc_target_event( int type, uint16_t handle )
{
    uint8_t event_data[2];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data( type, event_data, 2 );
}

wiced_result_t hci_control_send_avrc_volume( uint16_t handle, uint8_t volume )
{
    uint8_t event_data[3];

    //Build event payload
    event_data[0] = handle & 0xff;                          // handle
    event_data[1] = ( handle >> 8 ) & 0xff;
    event_data[2] = volume;                  // Volume level in percentage

    WICED_BT_TRACE( "[%s] handle %04x Volume(Pct): %d\n", __FUNCTION__, handle, event_data[2] );

    return wiced_transport_send_data( HCI_CONTROL_AVRC_TARGET_EVENT_VOLUME_LEVEL, event_data, 3 );
}

/*********************************************************************************************
 * AVRCP controller event handlers
 *********************************************************************************************/
/*
 *  send avrcp controller complete event to UART
 */
wiced_result_t hci_control_avrc_send_connect_complete( wiced_bt_device_address_t bd_addr, uint8_t status, uint16_t handle )
{
    int i = 0;
    const int cmd_size = BD_ADDR_LEN + sizeof(handle) + sizeof(uint8_t);
    uint8_t event_data[cmd_size];

    WICED_BT_TRACE( "[%s] %B status %x handle %x\n", __FUNCTION__, bd_addr, status, handle );

    //Build event payload
    if ( status == WICED_SUCCESS )
    {
        for ( ; i < BD_ADDR_LEN; i++ )                     // bd address
            event_data[i] = bd_addr[BD_ADDR_LEN - 1 - i];

        event_data[i++] = status;

        event_data[i++] = handle & 0xff;                        //handle
        event_data[i++] = ( handle >> 8 ) & 0xff;
#ifdef STANDALONE_HEADSET_APP
       /* store the handle for stand alone app*/
        app_state_cb.remote_control_handle = handle;
#endif
    }
    else
    {
        event_data[i++] = status;
    }

    return wiced_transport_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED, event_data, i );
}

/*
 *  send avrcp controller disconnect complete event to UART
 */
wiced_result_t hci_control_avrc_send_disconnect_complete( uint16_t handle )
{
    int i;
    uint8_t event_data[4];

    WICED_BT_TRACE( "[%s] handle: %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    return wiced_transport_send_data( HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED, event_data, 2 );
}

/*
 *  send avrcp controller play status event to UART
 */
wiced_result_t hci_control_avrc_send_play_status_change( uint16_t handle, uint8_t play_status )
{
    uint8_t event_data[3];

    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    event_data[2] = play_status & 0xff;                      // play status

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS, event_data, 3);
}

/*
 *  send avrc controller play status information( play_status, song-position and song-length) to UART
 */
wiced_result_t hci_control_avrc_send_play_status_info(uint16_t handle, uint8_t play_status, uint32_t song_length, uint32_t song_position)
{
    uint8_t event_data[11];
    WICED_BT_TRACE( "[%s] handle %04x\n", __FUNCTION__, handle );

    //Build event payload
    event_data[0] = handle & 0xff;                          //handle
    event_data[1] = ( handle >> 8 ) & 0xff;

    event_data[2] = play_status & 0xff;                     // play status

    event_data[3] = song_length & 0xff;                     // song-length
    event_data[4] = (song_length >> 8 )& 0xff;
    event_data[5] = (song_length >> 16) & 0xff;
    event_data[6] = (song_length >> 24) & 0xff;

    event_data[7]  = song_position & 0xff;                  // song-position
    event_data[8]  = (song_position >> 8 )& 0xff;
    event_data[9]  = (song_position >> 16) & 0xff;
    event_data[10] = (song_position >> 24) & 0xff;

    return wiced_transport_send_data(HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS_INFO, event_data, 11);
}

/*
 *  send AVRC event to UART
 */

wiced_result_t hci_control_send_avrc_event( int type, uint8_t *p_data, uint16_t data_size )
{
    return wiced_transport_send_data( type, p_data, data_size );
}

/* This is used only for stand-alone mode to store the keys in the NVRAM */
#ifdef STANDALONE_HEADSET_APP

/*
 * Read keys from the NVRAM and update address resolution database
 */
void headset_load_keys_to_addr_resolution_db(void)
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;

    bytes_read = wiced_hal_read_nvram(WICED_NVRAM_VSID_START, sizeof(keys), (uint8_t *)&keys, &result);

    // if failed to read NVRAM, there is nothing saved at that location
    if (result == WICED_SUCCESS)
    {
        result = wiced_bt_dev_add_device_to_address_resolution_db(&keys, keys.key_data.ble_addr_type);
    }
}

/*
 * This function is called to save keys generated as a result of pairing or keys update
 */
wiced_bool_t headset_save_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t                     bytes_written = 0, bytes_read;
    wiced_result_t              result;
    uint8_t                     nvram_id = HEADSET_NVRAM_ID;
    wiced_bt_device_link_keys_t keys;

    // there might be a situation where the keys are already saved, for example in dual mode
    // device the br/edr keys can be present when we are doing le pairing
    bytes_read = wiced_hal_read_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)&keys, &result);

    WICED_BT_TRACE("keys.bd_addr:%B p_keys:%B keys static:%B p_keys:%B\n", keys.bd_addr, p_keys->bd_addr, keys.key_data.static_addr, p_keys->key_data.static_addr);

    if ((bytes_read != sizeof(wiced_bt_device_link_keys_t)) ||
        (memcmp(p_keys->bd_addr, keys.bd_addr, BD_ADDR_LEN) != 0))
    {
        bytes_written = wiced_hal_write_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
        WICED_BT_TRACE("Saved %d bytes at id:%d result:%d\n", bytes_written, nvram_id, result);
        return (bytes_written == sizeof (wiced_bt_device_link_keys_t));
    }

    // try to figure out what is being updated
    if (p_keys->key_data.le_keys_available_mask != 0)
    {
        WICED_BT_TRACE("updating LE keys\n");
        keys.key_data.le_keys_available_mask = p_keys->key_data.le_keys_available_mask;
        keys.key_data.ble_addr_type          = p_keys->key_data.ble_addr_type;
        keys.key_data.static_addr_type       = p_keys->key_data.static_addr_type;
        memcpy (keys.key_data.static_addr, p_keys->key_data.static_addr, sizeof(p_keys->key_data.static_addr));
        memcpy (&keys.key_data.le_keys, &p_keys->key_data.le_keys, sizeof(p_keys->key_data.le_keys));
    }
    else
    {
        WICED_BT_TRACE("updating BR/EDR keys\n");
        keys.key_data.br_edr_key_type       = p_keys->key_data.br_edr_key_type;
        memcpy (keys.key_data.br_edr_key, p_keys->key_data.br_edr_key, sizeof(p_keys->key_data.br_edr_key));
    }
    WICED_BT_TRACE("Saved %d bytes at id:%d result:%d\n", bytes_written, nvram_id, result);
    return WICED_TRUE;
}

/*
 * This function is called to read keys for specific bdaddr
 */
wiced_bool_t headset_read_link_keys(wiced_bt_device_link_keys_t *p_keys)
{
    uint8_t                     bytes_read;
    wiced_result_t              result;
    wiced_bt_device_link_keys_t keys;

    bytes_read = wiced_hal_read_nvram(HEADSET_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), (uint8_t *)p_keys, &result);
    WICED_BT_TRACE("read %d bytes at id:%d\n", bytes_read, HEADSET_NVRAM_ID);
    return (bytes_read == sizeof (wiced_bt_device_link_keys_t));
}

/*
 * This function is called to delete paired host info
 */
wiced_bool_t headset_delete_link_keys(void)
{
    uint8_t                     bytes_written, bytes_read;
    wiced_result_t              result;
    uint8_t                     nvram_id = HEADSET_NVRAM_ID;
    wiced_bt_device_link_keys_t keys;

    wiced_hal_delete_nvram(nvram_id, &result);
    return WICED_TRUE;
}

/*
 * Read keys from the NVRAM and update address resolution database
 */
wiced_bool_t headset_get_paired_host_info(uint8_t* bda)
{
    wiced_bt_device_link_keys_t keys;
    if (headset_read_link_keys(&keys))
    {
        memcpy(bda, keys.bd_addr, BD_ADDR_LEN);
        return WICED_TRUE;
    }
    return WICED_FALSE;
}

#endif
/* End of link-keys storing to NVRAM */

/*
 * Write NVRAM function is called to store information in the RAM.  This can be called when
 * stack requires persistent storage, for example to save link keys.  In this case
 * data is also formatted and send to the host for real NVRAM storage.  The same function is
 * called when host pushes NVRAM chunks during the startup.  Parameter from_host in this
 * case is set to FALSE indicating that data does not need to be forwarded.
 */
int hci_control_write_nvram( int nvram_id, int data_len, void *p_data, wiced_bool_t from_host )
{
    uint8_t                    tx_buf[257];
    uint8_t                   *p = tx_buf;
    hci_control_nvram_chunk_t *p1;
    hci_control_nvram_chunk_t *prev = NULL;
    wiced_result_t            result;

    /* first check if this ID is being reused and release the memory chunk */
    hci_control_delete_nvram( nvram_id, WICED_FALSE );

    /* Allocating a buffer from the pool created for storing the peer info */
    if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
    {
        /*There is no space for the new device, delete a node from end of the list i.e first most inserted device */
        p1 = p_nvram_first;
        while(p1->p_next != NULL)
        {
            prev = p1;
            p1 = p1->p_next;
        }
        wiced_bt_free_buffer(prev->p_next);
        prev->p_next = NULL;

        if ( ( p1 = ( hci_control_nvram_chunk_t * )wiced_bt_get_buffer_from_pool( p_key_info_pool ) ) == NULL)
        {
            WICED_BT_TRACE( "Failed to alloc:%d\n", data_len );
            return ( 0 );
        }
    }
    if ( wiced_bt_get_buffer_size( p1 ) < ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) )
    {
        WICED_BT_TRACE( "Insufficient buffer size, Buff Size %d, Len %d  \n",
                        wiced_bt_get_buffer_size( p1 ),
                        ( sizeof( hci_control_nvram_chunk_t ) + data_len - 1 ) );
        wiced_bt_free_buffer( p1 );
        return ( 0 );
    }

    /*Insert the new node at the begining*/
    p1->p_next    = p_nvram_first;
    p1->nvram_id  = nvram_id;
    p1->chunk_len = data_len;
    memcpy( p1->data, p_data, data_len );

    p_nvram_first = p1;

    wiced_bt_device_link_keys_t * p_keys = ( wiced_bt_device_link_keys_t *) p_data;
    result = wiced_bt_dev_add_device_to_address_resolution_db( p_keys,
            p_keys->key_data.ble_addr_type );

    WICED_BT_TRACE("Updated Addr Resolution DB:%d\n", result );

    // If NVRAM chunk arrived from host, no need to send it back, otherwise send over transport
    if (!from_host)
    {
        *p++ = nvram_id & 0xff;
        *p++ = (nvram_id >> 8) & 0xff;
        memcpy(p, p_data, data_len);

        wiced_transport_send_data( HCI_CONTROL_EVENT_NVRAM_DATA, tx_buf, ( int )( data_len + 2 ) );
    }
    else
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }
    return (data_len);
}

/*
 * Find nvram_id of the NVRAM chunk with first bytes matching specified byte array
 */
int hci_control_find_nvram_id(uint8_t *p_data, int len)
{
    hci_control_nvram_chunk_t *p1;

    /* Go through the linked list of chunks */
    for (p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next)
    {
        WICED_BT_TRACE( "find %B %B len:%d\n", p1->data, p_data, len );
        if ( memcmp( p1->data, p_data, len ) == 0 )
        {
            return ( p1->nvram_id );
        }
    }
    return HCI_CONTROL_INVALID_NVRAM_ID;
}

/*
 * Delete NVRAM function is called when host deletes NVRAM chunk from the persistent storage.
 */
void hci_control_delete_nvram( int nvram_id, wiced_bool_t from_host )
{
    hci_control_nvram_chunk_t *p1, *p2;

    /* If Delete NVRAM data command arrived from host, send a Command Status response to ack command */
    if (from_host)
    {
        hci_control_send_command_status_evt( HCI_CONTROL_EVENT_COMMAND_STATUS, HCI_CONTROL_STATUS_SUCCESS );
    }

    if ( p_nvram_first == NULL )
        return;

    /* Special case when need to remove the first chunk */
    if ( ( p_nvram_first != NULL ) && ( p_nvram_first->nvram_id == nvram_id ) )
    {
        p1 = p_nvram_first;

        if ( from_host && ( wiced_bt_dev_delete_bonded_device (p1->data) == WICED_ERROR ) )
        {
            WICED_BT_TRACE("ERROR: while Unbonding device \n");
        }
        else
        {
        p_nvram_first = (hci_control_nvram_chunk_t *)p_nvram_first->p_next;
        wiced_bt_free_buffer( p1 );
        }
        return;
    }

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
    {
        p2 = (hci_control_nvram_chunk_t *)p1->p_next;

        if ( ( p2 != NULL ) && ( p2->nvram_id == nvram_id ) )
        {
            if ( from_host && ( wiced_bt_dev_delete_bonded_device (p2->data) == WICED_ERROR ) )
            {
                WICED_BT_TRACE("ERROR: while Unbonding device \n");
            }
            else
            {
            p1->p_next = p2->p_next;
            wiced_bt_free_buffer( p2 );
            }
            return;
        }
    }
}

/*
 * Read NVRAM actually finds the memory chunk in the RAM
 */
int hci_control_read_nvram( int nvram_id, void *p_data, int data_len )
{
    hci_control_nvram_chunk_t *p1;
    int                        data_read = 0;

    /* Go through the linked list of chunks */
    for ( p1 = p_nvram_first; p1 != NULL; p1 = p1->p_next )
    {
        if ( p1->nvram_id == nvram_id )
        {
            data_read = ( data_len < p1->chunk_len ) ? data_len : p1->chunk_len;
            memcpy( p_data, p1->data, data_read );
            break;
        }
    }
    return ( data_read );
}

/*
 * Allocate nvram_id to save new NVRAM chunk
 */
int hci_control_alloc_nvram_id( )
{
    hci_control_nvram_chunk_t *p1 = p_nvram_first;
    int                        nvram_id;
    uint8_t                    allocated_key_pool_count;

    /* Go through the linked list of chunks */
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id\n" );
    for ( nvram_id = HCI_CONTROL_FIRST_VALID_NVRAM_ID; p1 != NULL; nvram_id++ )
    {
        allocated_key_pool_count = 1;

        for ( p1 = p_nvram_first; p1 != NULL; p1 = (hci_control_nvram_chunk_t *)p1->p_next )
        {
            /* If the key buffer pool is becoming full, we need to notify the mcu and disable Pairing.
             * The mcu will need to delete some nvram entries and enable pairing in order to
             * pair with more devices */
            allocated_key_pool_count++;
            if ( ( allocated_key_pool_count == KEY_INFO_POOL_BUFFER_COUNT ) && ( hci_control_cb.pairing_allowed ) )
            {
                // Send Max Number of Paired Devices Reached event message
                wiced_transport_send_data( HCI_CONTROL_EVENT_MAX_NUM_OF_PAIRED_DEVICES_REACHED, NULL, 0 );

                hci_control_cb.pairing_allowed = WICED_FALSE;
                wiced_bt_set_pairable_mode( hci_control_cb.pairing_allowed, 0 );
            }

            if ( p1->nvram_id == nvram_id )
            {
                /* this nvram_id is already used */
                break;
            }
        }
        if ( p1 == NULL )
        {
            break;
        }
    }
    WICED_BT_TRACE ( "hci_control_alloc_nvram_id:%d\n", nvram_id );
    return ( nvram_id );
}

#if 0
/*
 * Remote Control can work a target or a controller.  This function sets up the appropriate role.
 */
void hci_control_switch_avrcp_role(uint8_t new_role)
{
    WICED_BT_TRACE ( "[%s] New Role: %d \n", __FUNCTION__, new_role);

    if (new_role != avrcp_profile_role)
    {
        switch (avrcp_profile_role)
        {
        case AVRCP_TARGET_ROLE:
            /* Shutdown the avrcp target */
            wiced_bt_rc_target_initiate_close();

            /* Initialize the avrcp controller */
            avrc_app_init();

            avrcp_profile_role = new_role;
            break;

        case AVRCP_CONTROLLER_ROLE:
            /* Shutdown the avrcp controller */
            wiced_bt_avrc_ct_cleanup();

            /* Initialize the avrcp target */
            app_avrc_init();
            wiced_bt_rc_target_register();

            avrcp_profile_role = new_role;
            break;

        default:
            break;
        }
    }
}
#endif

/*
 * hci_control_transport_tx_cplt_cback.
 * This function is called when a Transport Buffer has been sent to the MCU
 */
static void hci_control_transport_tx_cplt_cback(wiced_transport_buffer_pool_t* p_pool)
{

}
