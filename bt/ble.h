/*
 * Copyright 2016-2023, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
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
 * BR/EDR function and data
 *
 */
#ifndef __APP_BLE_H__
#define __APP_BLE_H__

#ifdef BLE_SUPPORT
#include "wiced.h"

/*****************************************************************************
 * Define Client Config Notification Flags
 ****************************************************************************/
// bit postion
typedef enum {
    BLE_RPT_INDX_STD_KEY,                // 0
    BLE_RPT_INDX_BIT_MAPPED,             // 1
    BLE_RPT_INDX_USER_DEFINED_KEY,       // 2
    BLE_RPT_INDX_BATTERY,                // 3
    BLE_RPT_INDX_VOICE,                  // 4
    BLE_RPT_INDX_VOICE_CTRL,             // 5
    BLE_RPT_INDX_TOUCHPAD,               // 6
    BLE_RPT_INDX_MAX
} CLIENT_CONFIG_NOTIF_e;

typedef uint8_t CLIENT_CONFIG_NOTIF_T;

/*****************************************************************************
 * Define Client Config Notification Flags
 ****************************************************************************/
#define APP_CLIENT_CONFIG_NOTIF_NONE                  0
#define APP_CLIENT_CONFIG_NOTIF_STD_KEY_RPT          (1<<BLE_RPT_INDX_STD_KEY)                // 0x001
#define APP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT       (1<<BLE_RPT_INDX_BIT_MAPPED)             // 0x002
#define APP_CLIENT_CONFIG_NOTIF_USER_DEFINED_KEY_RPT (1<<BLE_RPT_INDX_USER_DEFINED_KEY)       // 0x004
#define APP_CLIENT_CONFIG_NOTIF_BATTERY_RPT          (1<<BLE_RPT_INDX_BATTERY)                // 0x008
#define APP_CLIENT_CONFIG_NOTIF_VOICE_RPT            (1<<BLE_RPT_INDX_VOICE)                  // 0x010
#define APP_CLIENT_CONFIG_NOTIF_VOICE_CTRL_RPT       (1<<BLE_RPT_INDX_VOICE_CTRL)             // 0x020
#define APP_CLIENT_CONFIG_NOTIF_TOUCHPAD_RPT         (1<<BLE_RPT_INDX_TOUCHPAD)               // 0x040

/******************************************************************************
 *                         handle Definitions
 ******************************************************************************/
typedef enum
{
    HANDLE_APP_GATT_SERVICE = 0x1, // service handle

    HANDLE_APP_GAP_SERVICE = 0x14, // service handle
        HANDLE_APP_GAP_SERVICE_CHAR_DEV_NAME, // 0x15 characteristic handl
        HANDLE_APP_GAP_SERVICE_CHAR_DEV_NAME_VAL, // 0x16 char value handle

        HANDLE_APP_GAP_SERVICE_CHAR_DEV_APPEARANCE, // 0x17 characteristic handl
        HANDLE_APP_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,// 0x18 char value handle

        HANDLE_APP_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM, // 0x19 characteristic handl
        HANDLE_APP_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM_VAL,// 0x1a char value handle

    HANDLE_APP_DEV_INFO_SERVICE = 0x28,
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_PNP_ID, // 0x29 characteristic handle
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_PNP_ID_VAL,// 0x2a char value handle

        HANDLE_APP_DEV_INFO_SERVICE_CHAR_MFR_NAME, // characteristic handle
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,// char value handle

        HANDLE_APP_DEV_INFO_SERVICE_CHAR_FW_VER, // characteristic handle
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_FW_VER_VAL,// char value handle

    HANDLE_APP_BATTERY_SERVICE = 0x30, // service handle
        HANDLE_APP_BATTERY_SERVICE_CHAR_LEVEL, // characteristic handl
        HANDLE_APP_BATTERY_SERVICE_CHAR_LEVEL_VAL, // char value handle
        HANDLE_APP_BATTERY_SERVICE_CHAR_CFG_DESCR, // charconfig desc handl
        HANDLE_APP_BATTERY_SERVICE_RPT_REF_DESCR, // char desc handl

    HANDLE_APP_SCAN_PARAM_SERVICE = 0x40, // service handle
        HANDLE_APP_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW, // characteristic handl
        HANDLE_APP_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW_VAL, // char value handle

    HANDLE_APP_LE_HID_SERVICE = 0x4F, // service handle
        HANDLE_APP_LE_HID_SERVICE_INC_BAS_SERVICE,    // 0x50 include service

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT,         // 0x51 characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL,     // 0x52 char value handle

        HANDLE_APP_LE_HID_SERVICE_HID_INFO,         // 0x53 characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_INFO_VAL,     // 0x54 char value handle

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_MAP,      // characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_MAP_VAL,  // char value handle

        HANDLE_APP_LE_HID_SERVICE_EXT_RPT_REF_DESCR,// char desc handl

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT=0x5D,                 // 0x5d characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,                  // 0x5e char value handle
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,       // 0x5f charconfig desc handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_RPT_REF_DESCR,        // 0x60 char desc handl

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_OUTPUT,                     // 0x61 characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_VAL,                 // 0x62 char value handle
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_RPT_REF_DESCR,       // 0x63 char desc handl

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP,                         // 0x64 characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_VAL,                     // 0x65 char value handle
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_CHAR_CFG_DESCR,          // 0x66 charconfig desc handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_RPT_REF_DESCR,           // 0x67 char desc handl

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0=0x6c,            // 0x6c characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_VAL,             // 0x6d char value handle
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_CHAR_CFG_DESCR,  // 0x6e charconfig desc handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_RPT_REF_DESCR,   // 0x6f char desc handl

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE,                          // 0x70 characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_VAL,                      // 0x71 char value handle
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CHAR_CFG_DESCR,           // 0x72 charconfig desc handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_RPT_REF_DESCR,            // 0x73 char desc handl

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT,               // 0x74 characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_VAL,           // 0x75 char value handle
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_CHAR_CFG_DESCR,// 0x76 charconfig desc handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_RPT_REF_DESCR, // 0x77 char desc handl

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA,                 // 0x78 characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_VAL,             // 0x79 char value handle
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_RPT_REF_DESCR,   // 0x7a char desc handl

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD,                       // 0x7b characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_VAL,                   // 0x7c char value handle
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_CHAR_CFG_DESCR,        // 0x7d charconfig desc handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_RPT_REF_DESCR,         // 0x7e char desc handl

        HANDLE_APP_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL,                // 0x7f characteristic handl
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_VAL,            // 0x80 char value handle
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_RPT_REF_DESCR,  // 0x81 char desc handl

    HANDLE_APP_IMMEDIATE_ALERT_SERVICE = 0x90,            // 0x90 service handle
        HANDLE_APP_IMMEDIATE_ALERT_SERVICE_CHAR_LEVEL,    // 0x91 characteristic handle
        HANDLE_APP_IMMEDIATE_ALERT_SERVICE_CHAR_LEVEL_VAL,// 0x92 char value handle

    HANDLE_APP_FASTPAIR_SERVICE = 0x100,
        HANDLE_APP_FASTPAIR_SERVICE_CHAR_KEY_PAIRING,
        HANDLE_APP_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_VAL,
        HANDLE_APP_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_CFG_DESC,
        HANDLE_APP_FASTPAIR_SERVICE_CHAR_PASSKEY,
        HANDLE_APP_FASTPAIR_SERVICE_CHAR_PASSKEY_VAL,
        HANDLE_APP_FASTPAIR_SERVICE_CHAR_PASSKEY_CFG_DESC,
        HANDLE_APP_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY,
        HANDLE_APP_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_VAL,
        HANDLE_APP_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_CFG_DESC,

}HANDLE_APP_t;

/*******************************************************************************
 * Function Name: wiced_bool_t ble_get_cccd_flag(CLIENT_CONFIG_NOTIF_T idx)
 ********************************************************************************
 * Summary: Get report flags
 *
 * Parameters:
 *  idx -- report index
 *
 * Return:
 *  report cccd flags
 *
 *******************************************************************************/
uint16_t ble_get_cccd_flag(CLIENT_CONFIG_NOTIF_T idx);

/*******************************************************************************
 * Function Name: wiced_bool_t ble_is_notification_enabled(CLIENT_CONFIG_NOTIF_T idx)
 ********************************************************************************
 * Summary: Check if the notification flag is enabled
 *
 * Parameters:
 *  idx -- report index
 *
 * Return:
 *  TRUE if notifcation is enabled in cccd flag
 *
 *******************************************************************************/
#define ble_is_notification_enabled(idx) (ble_get_cccd_flag(idx) & GATT_CLIENT_CONFIG_NOTIFICATION)

/*******************************************************************************
 * Function Name: wiced_bool_t ble_is_indication_enabled(CLIENT_CONFIG_NOTIF_T idx)
 ********************************************************************************
 * Summary: Check if the indication flag is enabled
 *
 * Parameters:
 *  idx -- report index
 *
 * Return:
 *  TRUE if indication is enabled in cccd flag
 *
 *******************************************************************************/
#define ble_is_indication_enabled(idx) (ble_get_cccd_flag(idx) & GATT_CLIENT_CONFIG_INDICATION)

/*******************************************************************************
 * Function Name: ble_updateClientConfFlags
 ********************************************************************************
 * Summary:
 *   This function updates the client configuration characteristic values for the client in NVRAM
 *
 * Parameters:
 *  enable -- TRUE to set the flag. FALSE to clear the flag
 *  featureBit -- bit to set or clear
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void ble_updateClientConfFlags(uint16_t enable, uint16_t featureBit);

/*******************************************************************************
 * Function Name: void ble_init()
 ********************************************************************************
 * Summary: Bluetooth LE transport init.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void ble_init();

#else  // !BLE_SUPPORT
# define ble_init()
# define BLE_updateGattMapWithNotifications(f)
#endif // BLE_SUPPORT

#endif // __APP_BLE_H__
