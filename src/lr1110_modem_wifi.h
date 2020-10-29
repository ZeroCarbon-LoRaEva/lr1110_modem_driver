/*!
 * @file      lr1110_modem_wifi.h
 *
 * @brief     Wi-Fi passive scan driver definition for LR1110 modem
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

#ifndef LR1110_MODEM_WIFI_H
#define LR1110_MODEM_WIFI_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdbool.h>
#include <stdint.h>
#include "lr1110_modem_common.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC MACROS -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC CONSTANTS --------------------------------------------------------
 */
#define LR1110_MODEM_WIFI_N_RESULTS_MAX_PER_CHUNK LR1110_WIFI_MAX_RESULTS
#define LR1110_MODEM_WIFI_MAX_RESULTS ( 32 )
#define LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH ( 6 )
#define LR1110_MODEM_WIFI_STR_COUNTRY_CODE_SIZE ( 2 )

#define LR1110_MODEM_WIFI_CHANNEL_1_POS ( 0U )  //!< Channel at frequency 2.412 GHz
#define LR1110_MODEM_WIFI_CHANNEL_1_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_1_POS )
#define LR1110_MODEM_WIFI_CHANNEL_2_POS ( 1U )  //!< Channel at frequency 2.417 GHz
#define LR1110_MODEM_WIFI_CHANNEL_2_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_2_POS )
#define LR1110_MODEM_WIFI_CHANNEL_3_POS ( 2U )  //!< Channel at frequency 2.422 GHz
#define LR1110_MODEM_WIFI_CHANNEL_3_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_3_POS )
#define LR1110_MODEM_WIFI_CHANNEL_4_POS ( 3U )  //!< Channel at frequency 2.427 GHz
#define LR1110_MODEM_WIFI_CHANNEL_4_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_4_POS )
#define LR1110_MODEM_WIFI_CHANNEL_5_POS ( 4U )  //!< Channel at frequency 2.432 GHz
#define LR1110_MODEM_WIFI_CHANNEL_5_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_5_POS )
#define LR1110_MODEM_WIFI_CHANNEL_6_POS ( 5U )  //!< Channel at frequency 2.437 GHz
#define LR1110_MODEM_WIFI_CHANNEL_6_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_6_POS )
#define LR1110_MODEM_WIFI_CHANNEL_7_POS ( 6U )  //!< Channel at frequency 2.442 GHz
#define LR1110_MODEM_WIFI_CHANNEL_7_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_7_POS )
#define LR1110_MODEM_WIFI_CHANNEL_8_POS ( 7U )  //!< Channel at frequency 2.447 GHz
#define LR1110_MODEM_WIFI_CHANNEL_8_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_8_POS )
#define LR1110_MODEM_WIFI_CHANNEL_9_POS ( 8U )  //!< Channel at frequency 2.452 GHz
#define LR1110_MODEM_WIFI_CHANNEL_9_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_9_POS )
#define LR1110_MODEM_WIFI_CHANNEL_10_POS ( 9U )  //!< Channel at frequency 2.457 GHz
#define LR1110_MODEM_WIFI_CHANNEL_10_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_10_POS )
#define LR1110_MODEM_WIFI_CHANNEL_11_POS ( 10U )  //!< Channel at frequency 2.462 GHz
#define LR1110_MODEM_WIFI_CHANNEL_11_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_11_POS )
#define LR1110_MODEM_WIFI_CHANNEL_12_POS ( 11U )  //!< Channel at frequency 2.467 GHz
#define LR1110_MODEM_WIFI_CHANNEL_12_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_12_POS )
#define LR1110_MODEM_WIFI_CHANNEL_13_POS ( 12U )  //!< Channel at frequency 2.472 GHz
#define LR1110_MODEM_WIFI_CHANNEL_13_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_13_POS )
#define LR1110_MODEM_WIFI_CHANNEL_14_POS ( 13U )  //!< Channel at frequency 2.484 GHz
#define LR1110_MODEM_WIFI_CHANNEL_14_MASK ( 0x01UL << LR1110_MODEM_WIFI_CHANNEL_14_POS )

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*!
 * @brief Wi-Fi Channels index
 */
typedef enum
{
    LR1110_MODEM_WIFI_NO_CHANNEL   = 0x00,
    LR1110_MODEM_WIFI_CHANNEL_1    = 0x01,  //!< Channel at frequency 2.412 GHz
    LR1110_MODEM_WIFI_CHANNEL_2    = 0x02,  //!< Channel at frequency 2.417 GHz
    LR1110_MODEM_WIFI_CHANNEL_3    = 0x03,  //!< Channel at frequency 2.422 GHz
    LR1110_MODEM_WIFI_CHANNEL_4    = 0x04,  //!< Channel at frequency 2.427 GHz
    LR1110_MODEM_WIFI_CHANNEL_5    = 0x05,  //!< Channel at frequency 2.432 GHz
    LR1110_MODEM_WIFI_CHANNEL_6    = 0x06,  //!< Channel at frequency 2.437 GHz
    LR1110_MODEM_WIFI_CHANNEL_7    = 0x07,  //!< Channel at frequency 2.442 GHz
    LR1110_MODEM_WIFI_CHANNEL_8    = 0x08,  //!< Channel at frequency 2.447 GHz
    LR1110_MODEM_WIFI_CHANNEL_9    = 0x09,  //!< Channel at frequency 2.452 GHz
    LR1110_MODEM_WIFI_CHANNEL_10   = 0x0A,  //!< Channel at frequency 2.457 GHz
    LR1110_MODEM_WIFI_CHANNEL_11   = 0x0B,  //!< Channel at frequency 2.462 GHz
    LR1110_MODEM_WIFI_CHANNEL_12   = 0x0C,  //!< Channel at frequency 2.467 GHz
    LR1110_MODEM_WIFI_CHANNEL_13   = 0x0D,  //!< Channel at frequency 2.472 GHz
    LR1110_MODEM_WIFI_CHANNEL_14   = 0x0E,  //!< Channel at frequency 2.484 GHz
    LR1110_MODEM_WIFI_ALL_CHANNELS = 0x0F,
} lr1110_modem_wifi_channel_t;

/*!
 * @brief Wi-Fi signal type for passive scanning configuration
 *
 * Note it is not possible to configure the Wi-Fi passive scanning to search Wi-Fi type N GreenField. Only Wi-Fi type N
 * Mixed Mode can be scanned by LR1110.
 *
 * @warning LR1110_WIFI_TYPE_SCAN_G and LR1110_WIFI_TYPE_SCAN_N configurations are implemented the same way, and both
 * will scan Wi-Fi type G **AND** Wi-Fi type N.
 */
typedef enum
{
    LR1110_MODEM_WIFI_TYPE_SCAN_B     = 0x01,  //!< WiFi B
    LR1110_MODEM_WIFI_TYPE_SCAN_G     = 0x02,  //!< WiFi G
    LR1110_MODEM_WIFI_TYPE_SCAN_N     = 0x03,  //!< WiFi N
    LR1110_MODEM_WIFI_TYPE_SCAN_B_G_N = 0x04,  //!< Scan WiFi B and WiFi G/N
} lr1110_modem_wifi_signal_type_scan_t;

/*!
 * @brief Wi-Fi signal type for passive scan results
 *
 * Note that the WiFi N detected is Wi-Fi N Mixed mode, and not GreenField.
 */
typedef enum
{
    LR1110_MODEM_WIFI_TYPE_RESULT_B = 0x01,  //!< WiFi B
    LR1110_MODEM_WIFI_TYPE_RESULT_G = 0x02,  //!< WiFi G
    LR1110_MODEM_WIFI_TYPE_RESULT_N = 0x03,  //!< WiFi N
} lr1110_modem_wifi_signal_type_result_t;

/*!
 * @brief WiFi capture mode
 */
typedef enum
{
    LR1110_MODEM_WIFI_SCAN_MODE_BEACON = 1,             //!< Only scan for Beacons and Probe Response Access Points' MAC
                                                        //!< addresses until Period Beacon field (Basic result)
    LR1110_MODEM_WIFI_SCAN_MODE_BEACON_AND_PACKET = 2,  //!< Scan for beacons Access
                                                        //!< Points' MAC addresses until Period Beacon field, and for
                                                        //!< Packets until third Mac Address field (Basic result)
} lr1110_modem_wifi_mode_t;

typedef enum
{
    LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_COMPLETE         = 0x01,
    LR1110_MODEM_WIFI_RESULT_FORMAT_BASIC_MAC_TYPE_CHANNEL = 0x04,
} lr1110_modem_wifi_result_format_t;

/*!
 * @brief Type to store a MAC address
 */
typedef uint8_t lr1110_modem_wifi_mac_address_t[LR1110_MODEM_WIFI_MAC_ADDRESS_LENGTH];

typedef uint8_t lr1110_modem_wifi_datarate_info_byte_t;

typedef uint8_t lr1110_modem_wifi_channel_info_byte_t;

typedef uint8_t lr1110_modem_wifi_frame_type_info_byte_t;

typedef uint16_t lr1110_modem_wifi_channel_mask_t;

/*!
 * @brief Cumulative timings
 *
 * This structure is representing the cumulative time spent in the different modes of Wi-Fi passive scanning procedure.
 * Timing provided in [us].
 * */
typedef struct
{
    uint32_t rx_detection_us;    //!< Cumulative time spent during NFE or TOA
    uint32_t rx_correlation_us;  //!< Cumulative time spent during preamble detection
    uint32_t rx_capture_us;      //!< Cumulative time spent during signal acquisition
    uint32_t demodulation_us;    //!< Cumulative time spent during software
                                 //!< demodulation
} lr1110_modem_wifi_cumulative_timings_t;

/*!
 * @brief Basic complete result structure
 *
 * The beacon period is expressed in TU (Time Unit). 1 TU is 1024 microseconds.
 */
typedef struct
{
    lr1110_modem_wifi_datarate_info_byte_t   data_rate_info_byte;
    lr1110_modem_wifi_channel_info_byte_t    channel_info_byte;
    int8_t                                   rssi;
    lr1110_modem_wifi_frame_type_info_byte_t frame_type_info_byte;
    lr1110_modem_wifi_mac_address_t          mac_address;
    int16_t                                  phi_offset;
    uint64_t                                 timestamp_us;  //!< Indicate the up-time of the Access Point
                                                            //!< transmitting the Beacon [us]
    uint16_t beacon_period_tu;
} lr1110_modem_wifi_basic_complete_result_t;

/*!
 * @brief Basic MAC, type, channel result structure
 */
typedef struct
{
    lr1110_modem_wifi_datarate_info_byte_t data_rate_info_byte;
    lr1110_modem_wifi_channel_info_byte_t  channel_info_byte;
    int8_t                                 rssi;
    lr1110_modem_wifi_mac_address_t        mac_address;
} lr1110_modem_wifi_basic_mac_type_channel_result_t;

/*!
 * @brief Wi-Fi version parameters
 */
typedef struct
{
    uint8_t major;
    uint8_t minor;
} lr1110_modem_wifi_version_t;

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/*!
 * @brief Enable/Disable usage of hardware de-barker
 *
 * Hardware de-barker is used by the chip only in Wi-Fi type B passive scan. Using it dramatically reduces the passive
 * scan time. It must be enabled for country code search operations. It is enabled by default.
 *
 * @warning Disabling the Hardware De-Barker makes the LR1110_WIFI_SCAN_MODE_FULL_PACKET unusable.
 *
 * @param [in] context Chip implementation context
 * @param [in] enable_hardware_debarker Set to true to enable usage of hardware de-barker, false to disable
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_wifi_cfg_hardware_debarker( const void* context,
                                                                      const bool  enable_hardware_debarker );

/*!
 * @brief Reset the internal counters of cumulative timing
 *
 * @param [in] context Chip implementation context
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_wifi_reset_cumulative_timing( const void* context );

/*!
 * @brief Read the internal counters of cumulative timing
 *
 * @param [in] context Chip implementation context
 * @param [out] timing A pointer to the cumulative timing structure to populate
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_wifi_read_cumulative_timing( const void*                             context,
                                                                       lr1110_modem_wifi_cumulative_timings_t* timing );

/*!
 * @brief Configure the timestamp used to discriminate mobile access points from gateways.
 *
 * This filtering is based on the hypothesis that mobile access points have timestamp shorter than gateways.
 *
 * @param [in] context Chip implementation context
 * @param [in] timestamp_in_s Timestamp value in second
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_wifi_cfg_timestamp_ap_phone( const void* context, uint32_t timestamp_in_s );

/*!
 * @brief Get the internal wifi firmware version
 *
 * @param [in] context Chip implementation context
 * @param [out] wifi_version The wifi version structure populated with version numbers
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_wifi_read_version( const void*                  context,
                                                             lr1110_modem_wifi_version_t* wifi_version );

/*!
 * @brief Start a Wi-Fi passive scan operation
 *
 * During the complete passive scan operation, the LR1110 remains busy and cannot receive any commands. Using this
 * command **DOES** reset the results already obtained by previous passive scan operations.
 *
 * The result can be read at the end of the passive scan issuing the command lr1110_wifi_get_nb_results (to get the
 * number of results to read) and lr1110_wifi_read_basic_results or lr1110_wifi_read_extended_results to actually get
 * the result bytes.
 *
 * Reading with lr1110_wifi_read_basic_results or
 * lr1110_wifi_read_extended_results depends on the scan_mode selected when
 * calling this method with the correspondence: <table> <tr><th> Scan mode on
 * Wi-Fi Passive Scan </th><th> Result method to use </th> <tr><td>
 * LR1110_WIFI_SCAN_MODE_BEACON </td><td> lr1110_wifi_read_basic_results </td>
 * <tr><td> LR1110_WIFI_SCAN_MODE_PACKET </td><td>
 * lr1110_wifi_read_basic_results
 * </td> <tr><td> LR1110_WIFI_SCAN_MODE_FULL_BEACON </td><td>
 * lr1110_wifi_read_extended_results
 * </td> <tr><td> LR1110_WIFI_SCAN_MODE_FULL_PACKET </td><td>
 * lr1110_wifi_read_extended_results </td>
 * </table>
 *
 * @param [in] context Chip implementation context
 * @param [in] signal_type The type of Wi-Fi Signal to scan for
 * @param [in] channels Mask of the Wi-Fi channels to scan
 * @param [in] scan_mode Scan mode to execute
 * @param [in] max_results The maximal number of results to gather. When this limit is reached, the passive scan
 * automatically stop. Range of allowed values is [1:32]. Note that value 0 is forbidden.
 * @param [in] nb_scan_per_channel The number of internal scan sequences per channel scanned. Range of accepted values
 * is [1:255]. Note that value 0 is forbidden.
 * @param [in] timeout_in_ms The maximal duration of a single preamble search. Expressed in ms. Range of allowed values
 * is [1:65535]. Note that value 0 is forbidden.
 * @param [in] abort_on_timeout If true, the beacon search jumps to next configured Wi-Fi channel (or stop if there is
 * no more channel to scan) as soon as a search timeout is encountered
 * @param [in] result_format scanner result format \see lr1110_modem_wifi_result_format_t
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_wifi_passive_scan(
    const void* context, const lr1110_modem_wifi_signal_type_scan_t signal_type,
    const lr1110_modem_wifi_channel_mask_t channels, const lr1110_modem_wifi_mode_t scan_mode,
    const uint8_t max_results, const uint8_t nb_scan_per_channel, const uint16_t timeout_in_ms,
    const bool abort_on_timeout, const lr1110_modem_wifi_result_format_t result_format );

/*!
 * @brief Start a Wi-Fi passive scan operation
 *
 * During the complete passive scan operation, the LR1110 remains busy and cannot receive any commands. Using this
 * command **DOES** reset the results already obtained by previous passive scan operations.
 *
 * The result can be read at the end of the passive scan issuing the command lr1110_wifi_get_nb_results (to get the
 * number of results to read) and lr1110_wifi_read_basic_results or lr1110_wifi_read_extended_results to actually get
 * the result bytes.
 *
 * Reading with lr1110_wifi_read_basic_results or
 * lr1110_wifi_read_extended_results depends on the scan_mode selected when
 * calling this method with the correspondence: <table> <tr><th> Scan mode on
 * Wi-Fi Passive Scan </th><th> Result method to use </th> <tr><td>
 * LR1110_WIFI_SCAN_MODE_BEACON </td><td> lr1110_wifi_read_basic_results </td>
 * <tr><td> LR1110_WIFI_SCAN_MODE_PACKET </td><td>
 * lr1110_wifi_read_basic_results
 * </td> <tr><td> LR1110_WIFI_SCAN_MODE_FULL_BEACON </td><td>
 * lr1110_wifi_read_extended_results
 * </td> <tr><td> LR1110_WIFI_SCAN_MODE_FULL_PACKET </td><td>
 * lr1110_wifi_read_extended_results </td>
 * </table>
 *
 * @param [in] context Chip implementation context
 * @param [in] signal_type The type of Wi-Fi Signal to scan for
 * @param [in] channels Mask of the Wi-Fi channels to scan
 * @param [in] scan_mode Scan mode to execute
 * @param [in] max_results The maximal number of results to gather. When this limit is reached, the passive scan
 * automatically stop. Range of allowed values is [1:32]. Note that value 0 is forbidden.
 * @param [in] timeout_per_channel_ms The maximal duration of a single preamble search. Expressed in ms. Range of
 * allowed values is [1:65535]. Note that value 0 is forbidden.
 * @param [in] timeout_per_scan_ms The maximal duration of a single preamble search. Expressed in ms. Range of allowed
 * values is [0:65535]. If set to 0, the command will keep listening until exhaustion of timeout_per_channel_ms or until
 * nb_max_results is reached
 * @param [in] result_format scanner result format \see lr1110_modem_wifi_result_format_t
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_wifi_passive_scan_time_limit(
    const void* context, const lr1110_modem_wifi_signal_type_scan_t signal_type,
    const lr1110_modem_wifi_channel_mask_t channels, const lr1110_modem_wifi_mode_t scan_mode,
    const uint8_t max_results, const uint16_t timeout_per_channel_ms, const uint16_t timeout_per_scan_ms,
    const lr1110_modem_wifi_result_format_t result_format );

/*!
 * @brief Start a Wi-Fi passive scan for country codes extraction
 *
 * This command starts a Wi-Fi passive scan operation for Beacons and Probe Responses on Wi-Fi type B only. It is to
 * be used to extract the Country Code fields.
 *
 * During the passive scan, the results are filtered to keep only single MAC addresses.
 *
 * @param [in] context Chip implementation context
 * @param [in] channels_mask Mask of the Wi-Fi channels to scan
 * @param [in] nb_max_results The maximum number of country code to gather. When this limit is reached, the passive
 * scan automatically stops. Maximal value is 32
 * @param [in] nb_scan_per_channel Maximal number of scan attempts per channel. Maximal value is 255
 * @param [in] timeout_in_ms The maximal duration of a single beacon search. Expressed in ms. Maximal value is 65535
 * ms
 * @param [in] abort_on_timeout If true, the beacon search jumps to next configured Wi-Fi channel (or stop if there
 * is no more channel to scan) as soon as a search timeout is encountered
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_wifi_search_country_code(
    const void* context, const lr1110_modem_wifi_channel_mask_t channels_mask, const uint8_t nb_max_results,
    const uint8_t nb_scan_per_channel, const uint16_t timeout_in_ms, const bool abort_on_timeout );

/*!
 * @brief Start a Wi-Fi passive scan for country code extraction with duration stop conditions
 *
 * This command acts similarly to lr1110_wifi_search_country_code, except that its stop conditions are provided by
 * timeouts, and not by maximal number of scan attempts or results to find.
 *
 * @param [in] context Chip implementation context
 * @param [in] channels_mask Mask of the Wi-Fi channels to scan
 * @param [in] nb_max_results The maximum number of country code to gather. When this limit is reached, the passive scan
 * automatically stops. Maximal value is 32
 * @param [in] timeout_per_channel_ms The maximal duration of a scan on one channel. Expressed in ms
 * @param [in] timeout_per_scan_ms The maximal duration of a single preamble search. Expressed in ms. Range of allowed
 * values is [0:65535]. If set to 0, the command will keep listening until exhaustion of timeout_per_channel_ms or until
 * nb_max_results is reached
 *
 * @returns Operation status
 */
lr1110_modem_response_code_t lr1110_modem_wifi_search_country_code_time_limit(
    const void* context, const lr1110_modem_wifi_channel_mask_t channels_mask, const uint8_t nb_max_results,
    const uint16_t timeout_per_channel_ms, const uint16_t timeout_per_scan_ms );

/*!
 * @brief Read complete results
 *
 * @param [in] buffer Buffer containing the raw data
 * @param [in] buffer_len Size of the raw data buffer
 * @param [out] results Pointer to an array of result structures to populate. It is up to the caller to ensure this
 * array can hold at least nb_results elements.
 * @param [out] nb_results Number of results read
 */
void lr1110_modem_wifi_read_basic_results( const uint8_t* buffer, const uint16_t buffer_len,
                                           lr1110_modem_wifi_basic_mac_type_channel_result_t* results,
                                           uint8_t*                                           nb_results );

/*!
 * @brief Read basic results
 *
 * @param [in] buffer Buffer containing the raw data
 * @param [in] buffer_len Size of the raw data buffer
 * @param [out] results Pointer to an array of result structures to populate. It is up to the caller to ensure this
 * array can hold at least nb_results elements.
 * @param [out] nb_results Number of results read
 */
void lr1110_modem_wifi_read_complete_results( const uint8_t* buffer, const uint16_t buffer_len,
                                              lr1110_modem_wifi_basic_complete_result_t* results, uint8_t* nb_results );

/*!
 * @brief Helper method to retrieve channel from channel info byte
 *
 * @param [in] info_byte The chanel info byte from passive scan result
 *
 * @returns The channel of scanned MAC address
 */
lr1110_modem_wifi_channel_t lr1110_modem_extract_channel_from_info_byte(
    const lr1110_modem_wifi_channel_info_byte_t info_byte );

/*!
 * @brief Helper method to retrieve the signal type from data rate info byte
 *
 * @param [in] data_rate_info The data rate info byte from a passive scan result
 *
 * @returns The Signal Type of the scanned frame
 */
lr1110_modem_wifi_signal_type_result_t lr1110_modem_extract_signal_type_from_data_rate_info(
    const lr1110_modem_wifi_datarate_info_byte_t data_rate_info );

#ifdef __cplusplus
}
#endif

#endif  // LR1110_MODEM_WIFI_H

/* --- EOF ------------------------------------------------------------------ */
