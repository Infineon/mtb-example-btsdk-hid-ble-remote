/*
 * Copyright 2016-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
 * Entry point to application.
 *
 */
#include "app.h"

/*****************************************************************************
 * wiced_bt_stack buffer pool configuration
 *
 * Configure buffer pools used by the stack
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
static const wiced_bt_cfg_buf_pool_t wiced_bt_hid_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count } */
    { 64,       4   },      /* Small Buffer Pool */
    { 100,      30  },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
#ifdef CYW20735B1           // we need to allocate more buffer to ensure audio quality
    { 300,      50  },      /* Large Buffer Pool  (used for HCI ACL messages) */
#else                       // 208xx doesn't have enough RAM. audio quality is problematic
    { 300,      12  },      /* Large Buffer Pool  (used for HCI ACL messages) */
#endif
    { 1024,      1  },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};

/******************************************************************************
 *                          Function Definitions
******************************************************************************/

#ifdef ANDROID_AUDIO
/*
 * bleremote ble link management callbacks
 */
wiced_result_t app_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_RESUME_HIDD_LIB_HANDLER;

    switch( event )
    {
        case BTM_ENCRYPTION_STATUS_EVT:
            if (p_event_data->encryption_status.result == WICED_SUCCESS)
            {
                //configure ATT MTU size with peer device
                wiced_bt_gatt_configure_mtu(blelink.gatts_conn_id, bt_cfg.gatt_cfg.max_mtu_size);
            }
            // let default library to handle the reset
            break;
    }
    return result;
}
#else
 #define app_management_cback NULL
#endif

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{
    extern const wiced_platform_led_config_t platform_led[];
    extern const size_t led_count;

    // Initialize LED/UART for debug
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    hidd_led_init(led_count, platform_led);

    hidd_start(app_start, app_management_cback, &bt_cfg, wiced_bt_hid_cfg_buf_pools);

#if (SLEEP_ALLOWED == 3)
    hidd_allowed_hidoff(TRUE);
#endif

    WICED_BT_TRACE("\nDEV=%d Version:%d.%d Rev=%d Build=%d",hidd_chip_id(), WICED_SDK_MAJOR_VER, WICED_SDK_MINOR_VER, WICED_SDK_REV_NUMBER, WICED_SDK_BUILD_NUMBER);
    WICED_BT_TRACE("\nSLEEP_ALLOWED=%d",SLEEP_ALLOWED);
    WICED_BT_TRACE("\nLED=%d",LED_SUPPORT);

#ifdef OTA_FIRMWARE_UPGRADE
    WICED_BT_TRACE("\nOTA_FW_UPGRADE");
 #ifdef OTA_SECURE_FIRMWARE_UPGRADE
    WICED_BT_TRACE("\nOTA_SEC_FW_UPGRADE");
 #endif
#endif

#ifdef SUPPORT_AUDIO
 #ifdef CELT_ENCODER
    WICED_BT_TRACE("\nENABLE_AUDIO(CELT)");
 #elif defined(ADPCM_ENCODER)
  #ifdef ANDROID_AUDIO
    WICED_BT_TRACE("\nANDROID_AUDIO(ADPCM)");
  #else
    WICED_BT_TRACE("\nENABLE_AUDIO(ADPCM)");
  #endif
 #else
    WICED_BT_TRACE("\nENABLE_AUDIO(mSBC)");
 #endif
 #ifdef SUPPORT_DIGITAL_MIC
    WICED_BT_TRACE("\nENABLE_DIGITAL_MIC");
 #endif
#endif

#ifdef AUTO_RECONNECT
    WICED_BT_TRACE("\nAUTO_RECONNECT");
#endif

#ifdef SKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
    WICED_BT_TRACE("\nSKIP_PARAM_UPDATE");
#endif

#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
    WICED_BT_TRACE("\nSTART_ADV_ON_POWERUP");
#endif

#ifdef CONNECTED_ADVERTISING_SUPPORTED
    WICED_BT_TRACE("\nENABLE_CONNECTED_ADV");
#endif

#ifdef ENDLESS_LE_ADVERTISING_WHILE_DISCONNECTED
    WICED_BT_TRACE("\nDISCONNECTED_ENDLESS_ADV");
#endif

#ifdef ASSYM_SLAVE_LATENCY
    WICED_BT_TRACE("\nASSYMETRIC_SLAVE_LATENCY");
#endif

#ifdef LE_LOCAL_PRIVACY_SUPPORT
    WICED_BT_TRACE("\nLE_LOCAL_PRIVACY_SUPPORT");
#endif

#ifdef SUPPORT_IR
    WICED_BT_TRACE("\nENABLE_IR");
#endif

#ifdef EASY_PAIR
    WICED_BT_TRACE("\nENABLE_EASY_PAIR");
#endif

#ifdef SUPPORTING_FINDME
    WICED_BT_TRACE("\nENABLE_FINDME");
#endif

#ifdef FASTPAIR_ENABLE
    WICED_BT_TRACE("\nFASTPAIR_ENABLE");
#endif

}
