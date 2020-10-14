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
