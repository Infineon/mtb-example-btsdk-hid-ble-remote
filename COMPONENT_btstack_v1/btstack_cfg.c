/*
 * Copyright 2016-2022, Cypress Semiconductor Corporation (an Infineon company) or
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
 */
#include "btstack.h"
#include "app.h"

uint8_t dev_local_name[]                 = BT_LOCAL_NAME;

/*****************************************************************************
 * wiced_bt core stack configuration
 ****************************************************************************/
wiced_bt_cfg_settings_t bt_cfg =
{
    .device_name                         = dev_local_name,                                   /**< Local device name (NULL terminated) */
    .device_class                        = {0x20, 0x25, 0x0c},                               /**< Local device class */
    .security_requirement_mask           = BTM_SEC_ENCRYPT,                                  /**< Security requirements mask (BTM_SEC_NONE, or combinination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT (see #wiced_bt_sec_level_e)) */

    .max_simultaneous_links              = 2,                                                /**< Maximum number simultaneous links to different devices */

    .br_edr_scan_cfg = /* BR/EDR scan config */
    {
        .inquiry_scan_type               = BTM_SCAN_TYPE_STANDARD,                           /**< Inquiry scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .inquiry_scan_interval           = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,       /**< Inquiry scan interval  (0 to use default) */
        .inquiry_scan_window             = WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,         /**< Inquiry scan window (0 to use default) */

        .page_scan_type                  = BTM_SCAN_TYPE_STANDARD,                           /**< Page scan type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        .page_scan_interval              = WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,          /**< Page scan interval  (0 to use default) */
        .page_scan_window                = WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW             /**< Page scan window (0 to use default) */
    },

    .ble_scan_cfg = /* BLE scan settings  */
    {
        .scan_mode                       = BTM_BLE_SCAN_MODE_PASSIVE,                        /**< BLE scan mode (BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE) */

        /* Advertisement scan configuration */
        .high_duty_scan_interval         = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_INTERVAL,     /**< High duty scan interval */
        .high_duty_scan_window           = WICED_BT_CFG_DEFAULT_HIGH_DUTY_SCAN_WINDOW,       /**< High duty scan window */
        .high_duty_scan_duration         = 5,                                                /**< High duty scan duration in seconds (0 for infinite) */

        .low_duty_scan_interval          = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_INTERVAL,      /**< Low duty scan interval  */
        .low_duty_scan_window            = WICED_BT_CFG_DEFAULT_LOW_DUTY_SCAN_WINDOW,        /**< Low duty scan window */
        .low_duty_scan_duration          = 5,                                                /**< Low duty scan duration in seconds (0 for infinite) */

        /* Connection scan configuration */
        .high_duty_conn_scan_interval    = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_INTERVAL,/**< High duty cycle connection scan interval */
        .high_duty_conn_scan_window      = WICED_BT_CFG_DEFAULT_HIGH_DUTY_CONN_SCAN_WINDOW,  /**< High duty cycle connection scan window */
        .high_duty_conn_duration         = 30,                                               /**< High duty cycle connection duration in seconds (0 for infinite) */

        .low_duty_conn_scan_interval     = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_INTERVAL, /**< Low duty cycle connection scan interval */
        .low_duty_conn_scan_window       = WICED_BT_CFG_DEFAULT_LOW_DUTY_CONN_SCAN_WINDOW,   /**< Low duty cycle connection scan window */
        .low_duty_conn_duration          = 30,                                               /**< Low duty cycle connection duration in seconds (0 for infinite) */

#if defined(CELT_ENCODER) || defined(ADPCM_ENCODER)
        /* Connection configuration */
        .conn_min_interval               = 16,                                               /**< Minimum connection interval */
        .conn_max_interval               = 16,                                               /**< Maximum connection interval */
        .conn_latency                    = 49,                                               /**< Connection latency */
        .conn_supervision_timeout        = 500,                                              /**< Connection link supervision timeout */
#else
        /* Connection configuration */
        .conn_min_interval               = 8,                                                /**< Minimum connection interval */
        .conn_max_interval               = 8,                                                /**< Maximum connection interval */
        .conn_latency                    = 99,                                               /**< Connection latency */
        .conn_supervision_timeout        = 500,                                              /**< Connection link supervision timeout */
#endif
    },

    .ble_advert_cfg = /* BLE advertisement settings */
    {
        .channel_map                     = BTM_BLE_ADVERT_CHNL_37 |                          /**< Advertising channel map */
                                           BTM_BLE_ADVERT_CHNL_38 |
                                           BTM_BLE_ADVERT_CHNL_39,

        .high_duty_min_interval          = 32,                                               /**< High duty undirected connectable minimum advertising interval */
        .high_duty_max_interval          = 32,                                               /**< High duty undirected connectable maximum advertising interval */
#ifdef ALLOW_SDS_IN_DISCOVERABLE
        .high_duty_duration              = 0,                                                /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */
#else
        .high_duty_duration              = 30,                                               /**< High duty undirected connectable advertising duration in seconds (0 for infinite) */
#endif
        .low_duty_min_interval           = 48,                                               /**< Low duty undirected connectable minimum advertising interval */
        .low_duty_max_interval           = 48,                                               /**< Low duty undirected connectable maximum advertising interval */
#if defined(ALLOW_SDS_IN_DISCOVERABLE) || defined(ENDLESS_LE_ADVERTISING)
        .low_duty_duration               = 0,                                                /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */
#else
        .low_duty_duration               = 180,                                              /**< Low duty undirected connectable advertising duration in seconds (0 for infinite) */
#endif
        .high_duty_directed_min_interval = 32,                                               /**< High duty directed connectable minimum advertising interval */
        .high_duty_directed_max_interval = 32,                                               /**< High duty directed connectable maximum advertising interval */

        .low_duty_directed_min_interval  = 2048,                                             /**< Low duty directed connectable minimum advertising interval */
        .low_duty_directed_max_interval  = 2048,                                             /**< Low duty directed connectable maximum advertising interval */
        .low_duty_directed_duration      = 0,                                                /**< Low duty directed connectable advertising duration in seconds (0 for infinite) */

        .high_duty_nonconn_min_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL, /**< High duty non-connectable minimum advertising interval */
        .high_duty_nonconn_max_interval  = WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL, /**< High duty non-connectable maximum advertising interval */
        .high_duty_nonconn_duration      = 30,                                                      /**< High duty non-connectable advertising duration in seconds (0 for infinite) */

        .low_duty_nonconn_min_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,  /**< Low duty non-connectable minimum advertising interval */
        .low_duty_nonconn_max_interval   = WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,  /**< Low duty non-connectable maximum advertising interval */
        .low_duty_nonconn_duration       = 0                                                        /**< Low duty non-connectable advertising duration in seconds (0 for infinite) */
    },

    .gatt_cfg = /* GATT configuration */
    {
        .appearance                     = APPEARANCE_GENERIC_TAG,                            /**< GATT appearance (see gatt_appearance_e) */
        .client_max_links               = 1,                                                 /**< Client config: maximum number of servers that local client can connect to  */
        .server_max_links               = 2,                                                 /**< Server config: maximum number of remote clients connections allowed by the local */
        .max_attr_len                   = 246,                                               /**< Maximum attribute length; gki_cfg must have a corresponding buffer pool that can hold this length */
#if !defined(CYW20706A2)
        .max_mtu_size                   = 251                                                /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
#endif
    },

    .rfcomm_cfg =                                                                            /* RFCOMM configuration */
    {
        .max_links                      = 0,                                                 /**< Maximum number of simultaneous connected remote devices*/
        .max_ports                      = 0                                                  /**< Maximum number of simultaneous RFCOMM ports */
    },

    .l2cap_application =                                                                     /* Application managed l2cap protocol configuration */
    {
        .max_links                      = 0,                                                 /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */

        /* BR EDR l2cap configuration */
        .max_psm                        = 0,                                                 /**< Maximum number of application-managed BR/EDR PSMs */
        .max_channels                   = 0,                                                 /**< Maximum number of application-managed BR/EDR channels  */

        /* LE L2cap connection-oriented channels configuration */
        .max_le_psm                     = 0,                                                 /**< Maximum number of application-managed LE PSMs */
        .max_le_channels                = 0,                                                 /**< Maximum number of application-managed LE channels */
#if !defined(CYW20706A2)
        /* LE L2cap fixed channel configuration */
        .max_le_l2cap_fixed_channels    = 0                                                  /**< Maximum number of application managed fixed channels supported (in addition to mandatory channels 4, 5 and 6). > */
#endif
    },

    .avdt_cfg =
    /* Audio/Video Distribution configuration */
    {
        .max_links                      = 0,                                                 /**< Maximum simultaneous audio/video links */
#if !defined(CYW20706A2)
        .max_seps                       = 0                                                  /**< Maximum number of stream end points */
#endif
    },

    .avrc_cfg =                                                                              /* Audio/Video Remote Control configuration */
    {
        .roles                          = 0,                                                 /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        .max_links                      = 0                                                  /**< Maximum simultaneous remote control links */
    },

    /* LE Address Resolution DB size  */
    .addr_resolution_db_size            = 5,                                                 /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/

#ifdef CYW20706A2
    .max_mtu_size                       = 251,                                               /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5) */
    .max_pwr_db_val                     = 12                                                 /**< Max. power level of the device */
#else
    /* Maximum number of buffer pools */
    .max_number_of_buffer_pools         = 4,                                                 /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */

    /* Interval of  random address refreshing */
#if defined(LE_LOCAL_PRIVACY_SUPPORT) || defined(FASTPAIR_ENABLE)
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_CHANGE_TIMEOUT,/**< Interval of  random address refreshing - secs */
#else
    .rpa_refresh_timeout                = WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE,  /**< Interval of  random address refreshing - secs */
#endif
    /* BLE Filter Accept List size */
    .ble_filter_accept_list_size                = 2,                                                 /**< Maximum number of Filter Accept List devices allowed. Cannot be more than 128 */
#endif

#if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1) || defined (CYW20820A2)
    .default_ble_power_level            = 0                                                  /**< Default LE power level, Refer lm_TxPwrTable table for the power range */
#endif
};

/*****************************************************************************
 * wiced_bt_stack buffer pool configuration
 *
 * Configure buffer pools used by the stack
 *
 * Pools must be ordered in increasing buf_size.
 * If a pool runs out of buffers, the next  pool will be used
 *****************************************************************************/
const wiced_bt_cfg_buf_pool_t wiced_bt_hid_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count } */
    { 64,       12  },      /* Small Buffer Pool */
    { 100,      30  },      /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 300,     100  },      /* Large Buffer Pool  (used for HCI ACL messages) */
    { 1024,      2  },      /* Extra Large Buffer Pool - Used for avdt media packets and miscellaneous (if not needed, set buf_count to 0) */
};
