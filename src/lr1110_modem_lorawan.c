/*!
 * @file      lr1110_modem_lorawan.c
 *
 * @brief     LoRaWAN driver implementation for LR1110 modem
 *
 * Revised BSD License
 * Copyright Semtech Corporation 2020. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include "lr1110_modem_lorawan.h"
#include "lr1110_modem_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

#define LR1110_MODEM_GET_EVENT_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_VERSION_CMD_LENGTH ( 2 )
#define LR1110_MODEM_RESET_CMD_LENGTH ( 2 )
#define LR1110_MODEM_RESET_CHARGE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_CHARGE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_TX_POWER_OFFSET_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_TX_POWER_OFFSET_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_START_CMD_LENGTH ( 2 + 9 )
#define LR1110_MODEM_TEST_MODE_TST_NOP_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_TX_SINGLE_CMD_LENGTH ( 2 + 10 )
#define LR1110_MODEM_TEST_MODE_TST_TX_CONT_CMD_LENGTH ( 2 + 10 )
#define LR1110_MODEM_TEST_MODE_TST_HOP_CMD_LENGTH ( 2 + 5 )
#define LR1110_MODEM_TEST_MODE_TST_CW_CMD_LENGTH ( 2 + 6 )
#define LR1110_MODEM_TEST_MODE_TST_RX_CONT_CMD_LENGTH ( 2 + 8 )
#define LR1110_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_RSSI_CMD_LENGTH ( 2 + 8 )
#define LR1110_MODEM_TEST_MODE_TST_RADIO_RST_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_EXIT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_BUSY_LOOP_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_PANIC_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_WATCHDOG_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_TX_SINGLE_PREAM_CMD_LENGTH ( 2 + 12 )
#define LR1110_MODEM_TEST_MODE_READ_RSSI_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_TEST_MODE_TST_RSSI_2G4_CMD_LENGTH ( 2 + 5 )
#define LR1110_MODEM_TEST_MODE_TST_RSSI_GNSS_CMD_LENGTH ( 2 + 5 )
#define LR1110_MODEM_GET_GPS_TIME_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_STATUS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_ALARM_TIMER_CMD_LENGTH ( 2 + 4 )
#define LR1110_MODEM_GET_PIN_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_CHIP_EUI_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_JOIN_EUI_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_JOIN_EUI_CMD_LENGTH ( 2 + 8 )
#define LR1110_MODEM_GET_DEV_EUI_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_DEV_EUI_CMD_LENGTH ( 2 + 8 )
#define LR1110_MODEM_SET_APP_KEY_CMD_LENGTH ( 2 + LR1110_MODEM_APP_KEY_LENGTH )
#define LR1110_MODEM_GET_CLASS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_CLASS_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_REGION_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_REGION_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_LIST_REGION_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_ADR_PROFILE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_ADR_PROFILE_CMD_LENGTH ( 2 + 1 )  // +16 in case of adr custom
#define LR1110_MODEM_GET_DM_PORT_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_DM_PORT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_DM_INTERVAL_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_DM_INTERVAL_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_DM_FIELDS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_DM_FIELDS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SEND_DM_STATUS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_APP_STATUS_CMD_LENGTH ( 2 + 8 )
#define LR1110_MODEM_JOIN_CMD_LENGTH ( 2 )
#define LR1110_MODEM_LEAVE_NETWORK_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SUSPEND_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_NEXT_TX_MAX_PAYLOAD_CMD_LENGTH ( 2 )
#define LR1110_MODEM_REQUEST_TX_CMD_LENGTH ( 2 + 2 )
#define LR1110_MODEM_EMERGENCY_TX_CMD_LENGTH ( 2 + 2 )
#define LR1110_MODEM_UPLOAD_INIT_CMD_LENGTH ( 2 + 6 )
#define LR1110_MODEM_UPLOAD_DATA_CMD_LENGTH ( 2 )
#define LR1110_MODEM_UPLOAD_START_CMD_LENGTH ( 2 + 4 )
#define LR1110_MODEM_STREAM_INIT_CMD_LENGTH ( 2 + 2 )
#define LR1110_MODEM_SEND_STREAM_DATA_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_STREAM_STATUS_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_CMD_RSP_SIZE_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_SET_GPS_TIME_CMD_LENGTH ( 2 + 4 )
#define LR1110_MODEM_GET_EVENT_SIZE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_DERIVE_KEYS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_MANAGE_RF_OUTPUT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_SET_ALC_SYNC_PORT_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_ALC_SYNC_PORT_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_ALC_SYNC_MODE_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_ALC_SYNC_MODE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SET_CONNECTION_TIMEOUT_CMD_LENGTH ( 2 + 4 )
#define LR1110_MODEM_GET_CONNECTION_TIMEOUT_CMD_LENGTH ( 2 )
#define LR1110_MODEM_GET_LORAWAN_STATE_CMD_LENGTH ( 2 )
#define LR1110_MODEM_WRITE_USER_DEFINED_CHARGE_COUNTER_CMD_LENGTH ( 2 + 2 )
#define LR1110_MODEM_READ_USER_DEFINED_CHARGE_COUNTER_CMD_LENGTH ( 2 )
#define LR1110_MODEM_SELECT_CHARGE_UPLINK_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_DUTY_CYCLE_STATUS_CMD_LENGTH ( 2 )
#define LR1110_MODEM_ACTIVATE_DUTY_CYCLE_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_SET_CERTIFICATION_MODE_CMD_LENGTH ( 2 + 1 )
#define LR1110_MODEM_GET_CERTIFICATION_MODE_CMD_LENGTH ( 2 )

#define LR1110_MODEM_INFO_FIELDS_RBUFFER_MAX_LENGTH ( 20 )
#define LR1110_MODEM_CHIP_EUI_RBUFFER_LENGTH ( 8 )
#define LR1110_MODEM_JOIN_EUI_BUFFER_LENGTH ( 8 )
#define LR1110_MODEM_DEV_EUI_BUFFER_LENGTH ( 8 )
#define LR1110_MODEM_LIST_REGIONS_BUFFER_LENGTH ( 5 )
#define LR1110_MODEM_GET_VERSION_RBUFFER_LENGTH ( 10 )
#define LR1110_MODEM_GET_STREAM_STATUS_RBUFFER_LENGTH ( 4 )
#define LR1110_MODEM_EVENT_HEADER_LENGTH ( 2 )
#define LR1110_MODEM_TEST_MODE_TST_READ_PKT_COUNTER_RX_CONT_RBUFFER_LENGTH ( 4 )

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

enum
{
    LR1110_MODEM_GET_EVENT_CMD                         = 0x00,
    LR1110_MODEM_GET_VERSION_CMD                       = 0x01,
    LR1110_MODEM_RESET_CMD                             = 0x02,
    LR1110_MODEM_RESET_CHARGE_CMD                      = 0x04,
    LR1110_MODEM_GET_CHARGE_CMD                        = 0x05,
    LR1110_MODEM_GET_TX_POWER_OFFSET_CMD               = 0x06,
    LR1110_MODEM_SET_TX_POWER_OFFSET_CMD               = 0x07,
    LR1110_MODEM_TEST_CMD                              = 0x08,
    LR1110_MODEM_GET_GPS_TIME_CMD                      = 0x0A,
    LR1110_MODEM_GET_STATUS_CMD                        = 0x0B,
    LR1110_MODEM_SET_ALARM_TIMER_CMD                   = 0x0C,
    LR1110_MODEM_GET_PIN_CMD                           = 0x0E,
    LR1110_MODEM_GET_CHIP_EUI_CMD                      = 0x0F,
    LR1110_MODEM_GET_JOIN_EUI_CMD                      = 0x10,
    LR1110_MODEM_SET_JOIN_EUI_CMD                      = 0x11,
    LR1110_MODEM_GET_DEV_EUI_CMD                       = 0x12,
    LR1110_MODEM_SET_DEV_EUI_CMD                       = 0x13,
    LR1110_MODEM_SET_APP_KEY_CMD                       = 0x14,
    LR1110_MODEM_GET_CLASS_CMD                         = 0x15,
    LR1110_MODEM_SET_CLASS_CMD                         = 0x16,
    LR1110_MODEM_GET_REGION_CMD                        = 0x18,
    LR1110_MODEM_SET_REGION_CMD                        = 0x19,
    LR1110_MODEM_LIST_REGIONS_CMD                      = 0x1A,
    LR1110_MODEM_GET_ADR_PROFILE_CMD                   = 0x1B,
    LR1110_MODEM_SET_ADR_PROFILE_CMD                   = 0x1C,
    LR1110_MODEM_GET_DM_PORT_CMD                       = 0x1D,
    LR1110_MODEM_SET_DM_PORT_CMD                       = 0x1E,
    LR1110_MODEM_GET_DM_INFO_INTERVAL_CMD              = 0x1F,
    LR1110_MODEM_SET_DM_INFO_INTERVAL_CMD              = 0x20,
    LR1110_MODEM_GET_DM_INFO_FIELDS_CMD                = 0x21,
    LR1110_MODEM_SET_DM_INFO_FIELDS_CMD                = 0x22,
    LR1110_MODEM_DM_STATUS_CMD                         = 0x23,
    LR1110_MODEM_APP_STATUS_CMD                        = 0x24,
    LR1110_MODEM_JOIN_CMD                              = 0x25,
    LR1110_MODEM_LEAVE_NETWORK_CMD                     = 0x26,
    LR1110_MODEM_SUSPEND_MODEM_COM_CMD                 = 0x27,
    LR1110_MODEM_GET_NEXT_TX_MAX_PAYLOAD_CMD           = 0x28,
    LR1110_MODEM_REQUEST_TX_CMD                        = 0x29,
    LR1110_MODEM_EMERGENCY_TX_CMD                      = 0x2A,
    LR1110_MODEM_UPLOAD_INIT_CMD                       = 0x2B,
    LR1110_MODEM_UPLOAD_DATA_CMD                       = 0x2C,
    LR1110_MODEM_UPLOAD_START_CMD                      = 0x2D,
    LR1110_MODEM_STREAM_INIT_CMD                       = 0x2E,
    LR1110_MODEM_SEND_STREAM_DATA_CMD                  = 0x2F,
    LR1110_MODEM_STREAM_STATUS_CMD                     = 0x30,
    LR1110_MODEM_GET_RSP_SIZE_CMD                      = 0x31,
    LR1110_MODEM_SET_GPS_TIME_CMD                      = 0x32,
    LR1110_MODEM_GET_EVENT_SIZE_CMD                    = 0x33,
    LR1110_MODEM_DERIVE_KEYS_CMD                       = 0x34,
    LR1110_MODEM_MANAGE_RF_OUTPUT_CMD                  = 0x36,
    LR1110_MODEM_SET_ALC_SYNC_PORT_CMD                 = 0x37,
    LR1110_MODEM_GET_ALC_SYNC_PORT_CMD                 = 0x38,
    LR1110_MODEM_SET_ALC_SYNC_MODE_CMD                 = 0x39,
    LR1110_MODEM_GET_ALC_SYNC_MODE_CMD                 = 0x3A,
    LR1110_MODEM_SET_CONNECTION_TIMEOUT_CMD            = 0x3C,
    LR1110_MODEM_GET_CONNECTION_TIMEOUT_CMD            = 0x3D,
    LR1110_MODEM_SET_CERTIFICATION_MODE_CMD            = 0x3E,
    LR1110_MODEM_GET_CERTIFICATION_MODE_CMD            = 0x3F,
    LR1110_MODEM_GET_LORAWAN_STATUS_CMD                = 0x40,
    LR1110_MODEM_WRITE_USER_DEFINED_CHARGE_COUNTER_CMD = 0x41,
    LR1110_MODEM_READ_USER_DEFINED_CHARGE_COUNTER_CMD  = 0x42,
    LR1110_MODEM_SELECT_CHARGE_UPLINK_CMD              = 0x43,
    LR1110_MODEM_GET_DUTY_CYCLE_STATUS_CMD             = 0x44,
    LR1110_MODEM_ACTIVATE_DUTY_CYCLE_CMD               = 0x45,
};

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*!
 * @brief This command returns the number of bytes of next Event stored in LR1110 Modem. The returned value is the
 * length encoded in two bytes.
 *
 * @param [in] context Chip implementation context
 *
 * @param [out] event_size Event size encoded in two bytes
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_event_size( const void* context, uint16_t* event_size );

/*!
 * @brief This command returns size of command response to read
 *
 * @param [in] context Chip implementation context
 *
 * @param [in] cmd Command ID of the read command to be analyzed
 *
 * @param [out] cmd_rsp_size Response size
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_get_cmd_rsp_size( const void* context, uint8_t cmd, uint8_t* cmd_rsp_size );

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */


lr1110_modem_response_code_t lr1110_modem_get_version( const void* context, lr1110_modem_version_t* version )
{
    uint8_t                      cbuffer[LR1110_MODEM_GET_VERSION_CMD_LENGTH];
    uint8_t                      rbuffer[LR1110_MODEM_GET_VERSION_RBUFFER_LENGTH] = { 0x00 };
    lr1110_modem_response_code_t rc;

    cbuffer[0] = LR1110_MODEM_GROUP_ID_MODEM;
    cbuffer[1] = LR1110_MODEM_GET_VERSION_CMD;

    rc = ( lr1110_modem_response_code_t ) lr1110_modem_hal_read( context, cbuffer, LR1110_MODEM_GET_VERSION_CMD_LENGTH,
                                                                 rbuffer, LR1110_MODEM_GET_VERSION_RBUFFER_LENGTH );

    version->bootloader = ( ( uint32_t ) rbuffer[0] << 24 ) + ( ( uint32_t ) rbuffer[1] << 16 ) +
                          ( ( uint32_t ) rbuffer[2] << 8 ) + ( ( uint32_t ) rbuffer[3] );
    version->functionality = ( lr1110_modem_functionality_t ) rbuffer[4];
    version->firmware =
        ( ( uint32_t ) rbuffer[5] << 16 ) + ( ( uint32_t ) rbuffer[6] << 8 ) + ( ( uint32_t ) rbuffer[7] );
    version->lorawan = ( ( uint16_t ) rbuffer[8] << 8 ) + rbuffer[9];

    return rc;
}


/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DEFINITION --------------------------------------------
 */

/* --- EOF ------------------------------------------------------------------ */
