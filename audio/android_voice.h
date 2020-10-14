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
 *Android TV voice service
 *
 */
#ifdef ANDROID_AUDIO

#ifndef __ATV_VOICE__H
#define __ATV_VOICE__H

#ifndef BLE_SUPPORT
# error "ANDROID_AUDIO requires BLE_SUPPORT to be enabled!"
#endif

// ATV Voice Service - AB5E0001-5A21-4F05-BC7D-AF01F617B664
#define UUID_ATV_VOICE_SERVICE 0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, 0x7d, 0xbc, 0x05, 0x4f, 0x21, 0x5a, 0x01, 0x0, 0x5e, 0xab

// Write Characteristic - Characteristic AB5E0002-5A21-4F05-BC7D-AF01F617B664
// Used by ATV to send information to the Remote.
#define UUID_ATV_VOICE_TX_CHARACTERISTIC  0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, 0x7d, 0xbc, 0x05, 0x4f, 0x21, 0x5a, 0x02, 0x0, 0x5e, 0xab

// Read Characteristic - Characteristic AB5E0003-5A21-4F05-BC7D-AF01F617B664
// Used to return audio data in packets of 20 bytes each.
#define UUID_ATV_VOICE_RX_CHARACTERISTIC  0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, 0x7d, 0xbc, 0x05, 0x4f, 0x21, 0x5a, 0x03, 0x0, 0x5e, 0xab

// Control Characteristic - Characteristic AB5E0004-5A21-4F05-BC7D-AF01F617B664
// Used to return control data in packets of 20 bytes each.
#define UUID_ATV_VOICE_CTL_CHARACTERISTIC 0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, 0x7d, 0xbc, 0x05, 0x4f, 0x21, 0x5a, 0x04, 0x0, 0x5e, 0xab

#define HANDLE_ATV_VOICE_SERVICE                                        0xfe00
#define HANDLE_ATV_VOICE_TX_CHARACTERISTIC                              0xfe01
#define HANDLE_ATV_VOICE_TX_CHARACTERISTIC_VALUE                        0xfe02
#define HANDLE_ATV_VOICE_RX_CHARACTERISTIC                              0xfe03
#define HANDLE_ATV_VOICE_RX_CHARACTERISTIC_VALUE                        0xfe04
#define HANDLE_ATV_VOICE_RX_CLIENT_CONFIGURATION_DESCRIPTOR             0xfe05
#define HANDLE_ATV_VOICE_CTL_CHARACTERISTIC                             0xfe06
#define HANDLE_ATV_VOICE_CTL_CHARACTERISTIC_VALUE                       0xfe07
#define HANDLE_ATV_VOICE_CTL_CLIENT_CONFIGURATION_DESCRIPTOR            0xfe08
#define HANDLE_ATV_VOICE_SERVICE_END                                    HANDLE_ATV_VOICE_CTL_CLIENT_CONFIGURATION_DESCRIPTOR

#define is_Android_handle(handle) ((handle >= HANDLE_ATV_VOICE_SERVICE) && (handle <= HANDLE_ATV_VOICE_SERVICE_END))

#define ATV_VOICE_SERVICE_GET_CAPS_REQUEST          0x0A
#define ATV_VOICE_SERVICE_MIC_OPEN                  0x0C
#define ATV_VOICE_SERVICE_MIC_CLOSE                 0x0D

#define ATV_VOICE_SERVICE_AUDIO_STOP                0x0
#define ATV_VOICE_SERVICE_AUDIO_START               0x04
#define ATV_VOICE_SERVICE_START_SEARCH              0x08
#define ATV_VOICE_SERVICE_AUDIO_SYNC                0x0A
#define ATV_VOICE_SERVICE_GET_CAPS_RESP             0x0B
#define ATV_VOICE_SERVICE_MIC_OPEN_ERROR            0x0C

wiced_bt_gatt_status_t android_gatts_req_read_callback( uint16_t conn_id, wiced_bt_gatt_read_t * p_data );
wiced_bt_gatt_status_t android_gatts_req_write_callback( uint16_t conn_id, wiced_bt_gatt_write_t * p_data );

void android_send_ctl(uint8_t value);
void android_send_data(uint8_t * ptr, uint16_t len);

#endif // __ATV_VOICE__H

#else
 #define  android_gatts_req_read_callback   NULL
 #define  android_gatts_req_write_callback  NULL
#endif // ANDROID_AUDIO
