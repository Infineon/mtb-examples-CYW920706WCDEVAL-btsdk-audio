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

#ifndef HCI_CONTROL_RC_H_
#define HCI_CONTROL_RC_H_

#include "bt_types.h"

#include "hci_control.h"
#include "wiced_bt_sdp.h"

#include "wiced_result.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_avrc.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"

#include "wiced_bt_avrc_ct.h"

#define sizeof_array(a) (sizeof(a)/sizeof(a[0]))
#define CASE_RETURN_STR(const) case const: return #const;

#define MAX_AVRCP_VOLUME_LEVEL  0x7f
extern uint8_t hci_control_avrc_handle_command( uint16_t opcode, uint8_t *data, uint16_t payload_len );

/**
 *
 * Function         hci_control_avrc_handle_command
 *
 *                  Handler for the commands sent from the MCU over the HCI to invoke AVRCP
 *                  connection and peer requests
 *
 * @param[in]       cmd_opcode   : HCI Command invoked by the MCU app
 * @param[in]       p_data       : command data bytes
 * @param[in]       payload_len  : command data byte length
 *
 * @return          uint8_t (HCI return status)
 */
uint8_t hci_control_avrc_handle_ctrlr_command( uint16_t cmd_opcode, uint8_t *p_data, uint16_t payload_len );

#endif /* HCI_CONTROL_RC_H_ */
