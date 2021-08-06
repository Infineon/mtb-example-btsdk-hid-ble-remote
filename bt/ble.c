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
 * ble.c
 *
 * This file provides BLE transport functions
 *
 */
#ifdef BLE_SUPPORT
#include "app.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_sdp_defs.h"

#ifdef FASTPAIR_ENABLE
#include "wiced_bt_gfps.h"

/* MODEL-specific definitions */
 #if defined(CYW20721B2) || defined(CYW43012C0)
  #define FASTPAIR_MODEL_ID                   0x82DA6E
 #else
  #define FASTPAIR_MODEL_ID                   0xCE948F //0xB49236 //0x000107 //0x140A02 // 0xCE948F
 #endif

 #if (FASTPAIR_MODEL_ID == 0x82DA6E)
const uint8_t anti_spoofing_public_key[] =  { 0x95, 0xcf, 0xdb, 0xae, 0xc0, 0xef, 0xc5, 0x1f, 0x39, 0x0f, 0x2a, 0xe0, 0x16, 0x5a, 0x2b, 0x59,\
		                                      0x62, 0xb2, 0xfe, 0x82, 0xfa, 0xf0, 0xd4, 0x1e, 0xa3, 0x4f, 0x07, 0x7e, 0xf7, 0x3d, 0xc0, 0x44,\
		                                      0x3d, 0xd0, 0x38, 0xb2, 0x31, 0x5d, 0xc6, 0x45, 0x72, 0x8a, 0x08, 0x0e, 0xc7, 0x4f, 0xc7, 0x76,\
		                                      0xd1, 0x19, 0xed, 0x8b, 0x17, 0x50, 0xb3, 0xa6, 0x94, 0x2e, 0xc8, 0x6b, 0xbb, 0x02, 0xc7, 0x4d };

const uint8_t anti_spoofing_private_key[] = { 0x84, 0xee, 0x67, 0xc3, 0x67, 0xea, 0x57, 0x38, 0xa7, 0x7e, 0xe2, 0x4d, 0x68, 0xaa, 0x9c, 0xf0,\
                                              0xc7, 0x9f, 0xc8, 0x07, 0x7e, 0x4e, 0x20, 0x35, 0x4c, 0x15, 0x43, 0x4d, 0xb5, 0xd2, 0xd1, 0xc3 };

 #elif (FASTPAIR_MODEL_ID == 0xCE948F)
const uint8_t anti_spoofing_public_key[] =  { 0x0e, 0xe2, 0xbf, 0xe7, 0x96, 0xc6, 0xe1, 0x13, 0xf6, 0x57, 0x4a, 0xa8, 0x8c, 0x3a, 0x1b, 0x9c,\
                                              0x67, 0x1e, 0x36, 0xdf, 0x62, 0x69, 0xd8, 0xe5, 0x07, 0xe6, 0x8a, 0x72, 0x66, 0x4c, 0x9c, 0x90,\
                                              0xfc, 0xff, 0x00, 0x4f, 0x0f, 0x95, 0xde, 0x63, 0xe1, 0xc0, 0xbb, 0xa0, 0x75, 0xb1, 0xd2, 0x76,\
                                              0xfd, 0xe9, 0x66, 0x25, 0x0d, 0x45, 0x43, 0x7d, 0x5b, 0xf9, 0xce, 0xc0, 0xeb, 0x11, 0x03, 0xbe };

const uint8_t anti_spoofing_private_key[] = { 0x71, 0x11, 0x42, 0xb5, 0xe4, 0xa0, 0x6c, 0xa2, 0x8b, 0x74, 0xd4, 0x87, 0x7d, 0xac, 0x15, 0xc5,\
                                              0x42, 0x38, 0x1d, 0xb7, 0xba, 0x21, 0x19, 0x60, 0x17, 0x67, 0xfc, 0xba, 0x67, 0x47, 0x44, 0xc6 };

 #else
const uint8_t anti_spoofing_public_key[] =  "";
const uint8_t anti_spoofing_private_key[] = "";
 #warning "No Anti-Spooging key"

 #endif
#endif //FASTPAIR_ENABLE

#define blehid_app_gatts_req_read_callback  atv_gatts_req_read_callback

/******************************************************************************
 * data for handle attrib value
 ******************************************************************************/
static uint8_t dev_pnp_id[]                 = {0x01, 0x31, 0x01, 0xB4, 0x04, 0x01, 0x00}; //BT SIG, cypress semiconductor, 0x04B4, 0x0001
static uint8_t dev_char_mfr_name_value[]    = "Cypress Semiconductor";
static uint8_t dev_char_fw_version_value[]  = {'1','2','3'};
static uint8_t rpt_ref_battery[]            = {RPT_ID_IN_BATTERY,        WICED_HID_REPORT_TYPE_INPUT};
static uint8_t rpt_ref_std_key_input[]      = {RPT_ID_IN_STD_KEY,        WICED_HID_REPORT_TYPE_INPUT};
static uint8_t rpt_ref_std_key_output[]     = {RPT_ID_OUT_KB_LED,        WICED_HID_REPORT_TYPE_OUTPUT};
static uint8_t rpt_ref_bitmap[]             = {RPT_ID_IN_BIT_MAPPED,     WICED_HID_REPORT_TYPE_INPUT};
static uint8_t rpt_ref_user_defined_0[]     = {RPT_ID_IN_MEDIA_KEY,      WICED_HID_REPORT_TYPE_INPUT};
static uint8_t dev_hid_information[]        = {0x00, 0x01, 0x00, 0x00};  // Verison 1.00, Not localized, Cannot remote wake, not normally connectable
static uint16_t dev_battery_service_uuid    = UUID_CHARACTERISTIC_BATTERY_LEVEL;
#ifdef HID_AUDIO
static uint8_t rpt_ref_voice[]              = {RPT_ID_IN_AUDIO_DATA,     WICED_HID_REPORT_TYPE_INPUT};
static uint8_t rpt_ref_voice_ctrl_input[]   = {RPT_ID_IN_AUDIO_CTL,      WICED_HID_REPORT_TYPE_INPUT};
static uint8_t rpt_ref_voice_ctrl_feature[] = {RPT_ID_FEATURE_AUDIO_CTL, WICED_HID_REPORT_TYPE_FEATURE}; //feature rpt, OK to change to output report. But need chang in host BSA.
#endif
#ifdef SUPPORT_TOUCHPAD
static uint8_t rpt_ref_touchpad[]           = {RPT_ID_IN_ABS_XY,         WICED_HID_REPORT_TYPE_INPUT};
#endif
static uint8_t rpt_ref_connection_ctrl[]    = {RPT_ID_FEATURE_CNT_CTL,   WICED_HID_REPORT_TYPE_FEATURE}; //feature rpt
static uint8_t app_connection_ctrl_rpt      = 0;

//static void BLE_updateGattMapWithNotifications(uint16_t flags);

/////////////////////////////////////////////////////////////////////////////////
/// atv_gatts_req_write_callback
/////////////////////////////////////////////////////////////////////////////////
wiced_bt_gatt_status_t blehid_app_gatts_req_write_callback( uint16_t conn_id, wiced_bt_gatt_write_t * p_data )
{
    wiced_bt_gatt_status_t status = WICED_BT_GATT_NOT_FOUND;

#ifdef ANDROID_AUDIO
    status = atv_gatts_req_write_callback(conn_id, p_data);
#endif
#ifdef SUPPORTING_FINDME
    if (status == WICED_BT_GATT_NOT_FOUND)
    {
        status =  findme_gatts_req_write_handler(conn_id, p_data);
    }
#endif
    return status;
}

/*****************************************************************************
 * data for ble module
 ****************************************************************************/
typedef struct {
    wiced_timer_t conn_param_update_timer;
} ble_data_t;

static ble_data_t ble = {};

/*****************************************************************************
 * This is the report map for HID Service
 ****************************************************************************/
uint8_t blehid_rpt_map[] = USB_LE_RPT_DESCRIPTOR;

static uint16_t cccd[BLE_RPT_INDX_MAX] = {0,};

/*****************************************************************************
 * This is the attribute table containing GATTDB_PERM_READABLE attributes
 ****************************************************************************/
attribute_t blehid_gattAttributes[] =
{
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL,
        sizeof(app_connection_ctrl_rpt),
        &app_connection_ctrl_rpt
    },
    {
        HANDLE_APP_GAP_SERVICE_CHAR_DEV_NAME_VAL,
        sizeof(BT_LOCAL_NAME)-1,
        dev_local_name  //fixed
    },
    {
        HANDLE_APP_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
        sizeof(bt_cfg.gatt_cfg.appearance),
        &bt_cfg.gatt_cfg.appearance
    },
    {
        HANDLE_APP_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM_VAL,
        8,
        &bt_cfg.ble_scan_cfg.conn_min_interval //fixed
    },
    {
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_PNP_ID_VAL,
        sizeof(dev_pnp_id),
        dev_pnp_id //fixed
    },
    {
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
        sizeof(dev_char_mfr_name_value)-1,
        dev_char_mfr_name_value //fixed
    },
    {
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_FW_VER_VAL,
        3,
        dev_char_fw_version_value //fixed
    },
#ifdef BATTERY_REPORT_SUPPORT
    {
        HANDLE_APP_BATTERY_SERVICE_CHAR_LEVEL_VAL,
        1,
        &batRpt.level
    },
    {
        HANDLE_APP_BATTERY_SERVICE_CHAR_CFG_DESCR,
        2,
        &cccd[BLE_RPT_INDX_BATTERY]
    },
    {
        HANDLE_APP_BATTERY_SERVICE_RPT_REF_DESCR,
        2,
        rpt_ref_battery  //fixed
    },
#endif
    {
        HANDLE_APP_LE_HID_SERVICE_HID_INFO_VAL,
        sizeof(dev_hid_information),
        dev_hid_information   //fixed
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_MAP_VAL,
        sizeof(blehid_rpt_map),
        blehid_rpt_map, //fixed
    },
    {
        HANDLE_APP_LE_HID_SERVICE_EXT_RPT_REF_DESCR,
        2,
        &dev_battery_service_uuid //fixed
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,
        sizeof(KeyboardStandardReport)-1,
        &key_rpts.stdRpt.modifierKeys //updated everytime a std key input report sent
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,
        2,
        &cccd[BLE_RPT_INDX_STD_KEY]
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_RPT_REF_DESCR,
        2,
        rpt_ref_std_key_input //fixed
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_VAL,
        sizeof(kbrpt_ledStates),
        &kbrpt_ledStates //updated everytime a std key output report received
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_RPT_REF_DESCR,
        2,
        rpt_ref_std_key_output //fixed
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_VAL,
        sizeof(KeyboardBitMappedReport)-1,
        key_rpts.bitMappedReport.bitMappedKeys //updated everytime a bitmap report sent
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_CHAR_CFG_DESCR,
        2,
        &cccd[BLE_RPT_INDX_BIT_MAPPED]
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_RPT_REF_DESCR,
        2,
        rpt_ref_bitmap //fixed
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_VAL,
        KEY_NUM_BYTES_IN_USER_DEFINED_REPORT,
        &key_rpts.userRpt.userKeys    //updated everytime a user defined 0/key report sent
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_CHAR_CFG_DESCR,
        2,
        &cccd[BLE_RPT_INDX_USER_DEFINED_KEY]
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_RPT_REF_DESCR,
        2,
        rpt_ref_user_defined_0  //fixed
    },
#ifdef HID_AUDIO
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_VAL,
        AUDIO_MTU_SIZE,
        voice_rpt //updated everytime a voice report sent
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CHAR_CFG_DESCR,
        2,
        &cccd[BLE_RPT_INDX_VOICE]
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_RPT_REF_DESCR,
        2,
        rpt_ref_voice   //fixed
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_VAL,
        11,
        app_voice_ctrl_input_rpt  //updated everytime a voice ctrl input report sent
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_CHAR_CFG_DESCR,
        2,
        &cccd[BLE_RPT_INDX_VOICE_CTRL]
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_RPT_REF_DESCR,
        2,
        rpt_ref_voice_ctrl_input    //fixed
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_VAL,
        11,
        app_voice_ctrl_feature_rpt   //updated everytime a voice ctrl feature report received
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_RPT_REF_DESCR,
        2,
        rpt_ref_voice_ctrl_feature  //fixed
    },
#endif
#ifdef SUPPORT_TOUCHPAD
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_VAL,
        9,
        app_touchpad_rpt  //updated everytime a touchpad report sent
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_CHAR_CFG_DESCR,
        2,
        &cccd[BLE_RPT_INDX_TOUCHPAD]
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_RPT_REF_DESCR,
        2,
        rpt_ref_touchpad    //fixed
    },
#endif
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_VAL,
        1,
        &app_connection_ctrl_rpt  //even though it is defined. But no usage. ignore now.
    },
    {
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_RPT_REF_DESCR,
        2,
        rpt_ref_connection_ctrl     //fixed
    },
};
const uint16_t blehid_gattAttributes_size = sizeof(blehid_gattAttributes)/sizeof(attribute_t);


/*****************************************************************************
 * This is the GATT database for the BLE HID Remote application.  It defines
 * services, characteristics and descriptors supported by the sensor.  Each
 * attribute in the database has a handle, (characteristic has two, one for
 * characteristic itself, another for the value).  The handles are used by
 * the peer to access attributes, and can be used locally by application for
 * example to retrieve data written by the peer.  Definition of characteristics
 * and descriptors has GATT Properties (read, write, notify...) but also has
 * permissions which identify if application is allowed to read or write
 * into it.  Handles do not need to be sequential, but need to be in order.
 ****************************************************************************/
uint8_t blehid_db_data[]=
{
    // Declare gatt service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_APP_GATT_SERVICE, UUID_SERVCLASS_GATT_SERVER ),

   // Declare GAP service. Device Name and Appearance are mandatory
    // characteristics of GAP service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_APP_GAP_SERVICE, UUID_SERVCLASS_GAP_SERVER ),

    // Declare mandatory GAP service characteristic: Dev Name
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_GAP_SERVICE_CHAR_DEV_NAME,
        HANDLE_APP_GAP_SERVICE_CHAR_DEV_NAME_VAL,
        GATT_UUID_GAP_DEVICE_NAME,
        GATTDB_CHAR_PROP_READ,
        GATTDB_PERM_READABLE
    ),

    // Declare mandatory GAP service characteristic: Appearance
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_GAP_SERVICE_CHAR_DEV_APPEARANCE,
        HANDLE_APP_GAP_SERVICE_CHAR_DEV_APPEARANCE_VAL,
        GATT_UUID_GAP_ICON,
        GATTDB_CHAR_PROP_READ,
        GATTDB_PERM_READABLE
    ),

    // Declare GAP service characteristic: Peripheral Prefered Connection Parameter
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM,
        HANDLE_APP_GAP_SERVICE_CHAR_PERI_PREFER_CONNPARAM_VAL,
        GATT_UUID_GAP_PREF_CONN_PARAM,
        GATTDB_CHAR_PROP_READ,
        GATTDB_PERM_READABLE
    ),

    // Declare Device info service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_APP_DEV_INFO_SERVICE, UUID_SERVCLASS_DEVICE_INFO ),

    // Handle 0x29: characteristic PnP ID, handle 0x2A characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_PNP_ID,
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_PNP_ID_VAL,
        GATT_UUID_PNP_ID,
        GATTDB_CHAR_PROP_READ,
        GATTDB_PERM_READABLE
    ),

    // Handle 0x2B: characteristic Manufacturer Name, handle 0x2C characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_MFR_NAME,
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_MFR_NAME_VAL,
        GATT_UUID_MANU_NAME,
        GATTDB_CHAR_PROP_READ,
        GATTDB_PERM_READABLE
    ),

    // Handle 0x2D: characteristic Firmware Revision String, handle 0x2E characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_FW_VER,
        HANDLE_APP_DEV_INFO_SERVICE_CHAR_FW_VER_VAL,
        GATT_UUID_FW_VERSION_STR,
        GATTDB_CHAR_PROP_READ,
        GATTDB_PERM_READABLE
    ),

#ifdef BATTERY_REPORT_SUPPORT
   // Declare Battery service
   PRIMARY_SERVICE_UUID16
        ( HANDLE_APP_BATTERY_SERVICE, UUID_SERVCLASS_BATTERY),

   // Handle 0x31: characteristic Battery Level, handle 0x32 characteristic value
   CHARACTERISTIC_UUID16
    (
        HANDLE_APP_BATTERY_SERVICE_CHAR_LEVEL,       // attribute handle
        HANDLE_APP_BATTERY_SERVICE_CHAR_LEVEL_VAL, // attribute value handle
        GATT_UUID_BATTERY_LEVEL,
        GATTDB_CHAR_PROP_READ|GATTDB_CHAR_PROP_NOTIFY,
        GATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_APP_BATTERY_SERVICE_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        GATTDB_PERM_READABLE|GATTDB_PERM_WRITE_CMD|GATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x34: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_BATTERY_SERVICE_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        GATTDB_PERM_READABLE
    ),
#endif

    // Declare Scan Parameters service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_APP_SCAN_PARAM_SERVICE, UUID_SERVCLASS_SCAN_PARAM),

    // Handle 0x41: characteristic Battery Level, handle 0x42 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_APP_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW,
        HANDLE_APP_SCAN_PARAM_SERVICE_CHAR_SCAN_INT_WINDOW_VAL,
        GATT_UUID_SCAN_INT_WINDOW,
        GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ
    ),

    // Declare HID over LE
    PRIMARY_SERVICE_UUID16
        ( HANDLE_APP_LE_HID_SERVICE, UUID_SERVCLASS_LE_HID),

#ifdef BATTERY_REPORT_SUPPORT
    // Include BSA SERVICE
    INCLUDE_SERVICE_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_INC_BAS_SERVICE,
        HANDLE_APP_BATTERY_SERVICE,
        HANDLE_APP_BATTERY_SERVICE_RPT_REF_DESCR,
        UUID_SERVCLASS_BATTERY
    ),
#endif

    // HID control point
    // Handle 0x51: characteristic HID Report, handle 0x52 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT,
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL,
        GATT_UUID_HID_CONTROL_POINT,
        GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        GATTDB_PERM_WRITE_CMD
    ),

    // Handle 0x53: characteristic HID information, handle 0x54 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_INFO,
        HANDLE_APP_LE_HID_SERVICE_HID_INFO_VAL,
        GATT_UUID_HID_INFORMATION,
        GATTDB_CHAR_PROP_READ,
        GATTDB_PERM_READABLE
    ),

    // Handle 0x55: characteristic HID Report MAP, handle 0x56 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_MAP,
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_MAP_VAL,
        GATT_UUID_HID_REPORT_MAP,
        GATTDB_CHAR_PROP_READ,
        GATTDB_PERM_READABLE
    ),

    // include Battery Service
    // Handle 0x57: external report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_EXT_RPT_REF_DESCR,
        GATT_UUID_EXT_RPT_REF_DESCR,
        GATTDB_PERM_READABLE
    ),

    // STD Input report
    // Handle 0x5D: characteristic HID Report, handle 0x5E characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT,
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,
        GATT_UUID_HID_REPORT,
        GATTDB_CHAR_PROP_READ|GATTDB_CHAR_PROP_NOTIFY,
        GATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        GATTDB_PERM_READABLE|GATTDB_PERM_WRITE_CMD|GATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x60: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        GATTDB_PERM_READABLE
    ),

    // STD Output report
    // Handle 0x61: characteristic HID Report, handle 0x62 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_OUTPUT,
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_VAL,
        GATT_UUID_HID_REPORT,
        GATTDB_CHAR_PROP_READ|GATTDB_CHAR_PROP_WRITE_NO_RESPONSE|GATTDB_CHAR_PROP_WRITE,
        GATTDB_PERM_READABLE|GATTDB_PERM_WRITE_CMD|GATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x63: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        GATTDB_PERM_READABLE
    ),

    // Bit mapped report, Report ID=2
    // Handle 0x64: characteristic HID Report, handle 0x65 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP,
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_VAL,
        GATT_UUID_HID_REPORT,
        GATTDB_CHAR_PROP_READ|GATTDB_CHAR_PROP_NOTIFY,
        GATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        GATTDB_PERM_READABLE|GATTDB_PERM_WRITE_CMD|GATTDB_PERM_WRITE_REQ
    ),

   // Handle 0x67: report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        GATTDB_PERM_READABLE
    ),

    // user defined 0 report
    // Handle 0x6C: characteristic HID Report, handle 0x6D characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0,
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_VAL,
        GATT_UUID_HID_REPORT,
        GATTDB_CHAR_PROP_READ|GATTDB_CHAR_PROP_NOTIFY,
        GATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        GATTDB_PERM_READABLE|GATTDB_PERM_WRITE_CMD|GATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x6F: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        GATTDB_PERM_READABLE
    ),

#ifdef HID_AUDIO
    //Voice report
    // Handle 0x70: characteristic HID Report, handle 0x71 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE,
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_VAL,
        GATT_UUID_HID_REPORT,
        GATTDB_CHAR_PROP_READ|GATTDB_CHAR_PROP_NOTIFY,
        GATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CHAR_CFG_DESCR,
        GATT_UUID_CHAR_CLIENT_CONFIG,
        GATTDB_PERM_READABLE|GATTDB_PERM_WRITE_CMD|GATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x73: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        GATTDB_PERM_READABLE
    ),

    //Voice Ctl INPUT report (remote->host)
    // Handle 0x74: characteristic HID Report, handle 0x75 characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT,
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_VAL,
        GATT_UUID_HID_REPORT,
        GATTDB_CHAR_PROP_READ|GATTDB_CHAR_PROP_NOTIFY,
        GATTDB_PERM_READABLE
    ),

    // Declare client specific characteristic cfg desc. // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection // for bonded devices
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
         HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_CHAR_CFG_DESCR,
         GATT_UUID_CHAR_CLIENT_CONFIG,
         GATTDB_PERM_READABLE|GATTDB_PERM_WRITE_CMD|GATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x77: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        GATTDB_PERM_READABLE
    ),

    //Voice Ctl FEATURE report (host->remote)
    // Handle 0x78: characteristic HID Report, handle 0x79 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA,
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_VAL,
        GATT_UUID_HID_REPORT,
        GATTDB_CHAR_PROP_READ|GATTDB_CHAR_PROP_WRITE,
        GATTDB_PERM_READABLE|GATTDB_PERM_WRITE_CMD|GATTDB_PERM_WRITE_REQ
    ),

    // Handle 0x7A: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        GATTDB_PERM_READABLE
    ),
#endif

#ifdef SUPPORT_TOUCHPAD
    //touchpad report
    // Handle 0x7B: characteristic HID Report, handle 0x7C characteristic value
    CHARACTERISTIC_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD,     //0x7B(CHARACTERISTIC handle)
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_VAL, //0x7C(CHARACTERISTIC value handle)
        GATT_UUID_HID_REPORT,                                 //0x2A4D (Report)
        GATTDB_CHAR_PROP_READ|GATTDB_CHAR_PROP_NOTIFY,    //0x12 (Read|Notify)
        GATTDB_PERM_READABLE                                //permission
    ),

    // Declare client specific characteristic cfg desc.
    // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection
    // for bonded devices
    // Handle 0x7D: client specific characteristic cfg desc
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_CHAR_CFG_DESCR,       //0x7D (client CHARACTERISTIC cfg desc handle)
        GATT_UUID_CHAR_CLIENT_CONFIG,                                          //0x2902 (Client CHARACTERISTIC configuration)
        GATTDB_PERM_READABLE|GATTDB_PERM_WRITE_CMD|GATTDB_PERM_WRITE_REQ //permission
    ),

    // Handle 0x7E: report reference
    CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_RPT_REF_DESCR, //0x7E (Report reference handle)
        GATT_UUID_RPT_REF_DESCR,                                        //0x2908 (Report reference)
        GATTDB_PERM_READABLE                                          //permission
    ),
#endif

    // Connection control feature
    // Handle 0x7F: characteristic HID Report, handle 0x80 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL,
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_VAL,
        GATT_UUID_HID_REPORT,
        GATTDB_CHAR_PROP_READ|GATTDB_CHAR_PROP_WRITE,
        GATTDB_PERM_READABLE|GATTDB_PERM_WRITE_REQ
    ),

   // Handle 0x81: report reference
   CHAR_DESCRIPTOR_UUID16
    (
        HANDLE_APP_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_RPT_REF_DESCR,
        GATT_UUID_RPT_REF_DESCR,
        GATTDB_PERM_READABLE
    ),

#ifdef SUPPORTING_FINDME
    // Declare Imediate Alert Service
    PRIMARY_SERVICE_UUID16
        ( HANDLE_APP_IMMEDIATE_ALERT_SERVICE, UUID_SERVCLASS_IMMEDIATE_ALERT),

    // Handle 0x91: characteristic alert level, handle 0x92 characteristic value
    CHARACTERISTIC_UUID16_WRITABLE
    (
        HANDLE_APP_IMMEDIATE_ALERT_SERVICE_CHAR_LEVEL,
        HANDLE_APP_IMMEDIATE_ALERT_SERVICE_CHAR_LEVEL_VAL,
        GATT_UUID_ALERT_LEVEL,
        GATTDB_CHAR_PROP_WRITE_NO_RESPONSE,
        GATTDB_PERM_WRITE_CMD
    ),
#endif // SUPPORTING_FINDME

#ifdef FASTPAIR_ENABLE
    // Declare Fast Pair service
    PRIMARY_SERVICE_UUID16 (HANDLE_APP_FASTPAIR_SERVICE, WICED_BT_GFPS_UUID16),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_APP_FASTPAIR_SERVICE_CHAR_KEY_PAIRING,
                                    HANDLE_APP_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_KEY_PAIRING,
                                    GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_APP_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_REQ),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_APP_FASTPAIR_SERVICE_CHAR_PASSKEY,
                                    HANDLE_APP_FASTPAIR_SERVICE_CHAR_PASSKEY_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_PASSKEY,
                                    GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_APP_FASTPAIR_SERVICE_CHAR_PASSKEY_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_REQ),

    CHARACTERISTIC_UUID16_WRITABLE (HANDLE_APP_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY,
                                    HANDLE_APP_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_VAL,
                                    WICED_BT_GFPS_UUID_CHARACTERISTIC_ACCOUNT_KEY,
                                    GATTDB_CHAR_PROP_WRITE,
                                    GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ),

    CHAR_DESCRIPTOR_UUID16_WRITABLE(HANDLE_APP_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_CFG_DESC,
                                    UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                    GATTDB_PERM_AUTH_READABLE | GATTDB_PERM_WRITE_REQ),
#endif

#ifdef ANDROID_AUDIO
    // Handle 0xfe00: Android TV Voice Service
    PRIMARY_SERVICE_UUID128
        ( HANDLE_ATV_VOICE_SERVICE, UUID_ATV_VOICE_SERVICE),

    // Handle 0xfe01: Write Characteristic (ATVV_CHAR_TX), handle 0xfe02 characteristic value
    CHARACTERISTIC_UUID128_WRITABLE
    (
        HANDLE_ATV_VOICE_TX_CHARACTERISTIC,
        HANDLE_ATV_VOICE_TX_CHARACTERISTIC_VALUE,
        UUID_ATV_VOICE_TX_CHARACTERISTIC,
        GATTDB_CHAR_PROP_WRITE,
        GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_WRITE_REQ  | GATTDB_PERM_AUTH_WRITABLE
    ),

    // Handles 0xfe03: Read characteristic (ATVV_CHAR_RX), handle 0xfe04 characteristic value.
    CHARACTERISTIC_UUID128
    (
        HANDLE_ATV_VOICE_RX_CHARACTERISTIC,
        HANDLE_ATV_VOICE_RX_CHARACTERISTIC_VALUE,
        UUID_ATV_VOICE_RX_CHARACTERISTIC,
        GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_READ,
        GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_READABLE
    ),

    // Handle 0xfe05
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_ATV_VOICE_RX_CLIENT_CONFIGURATION_DESCRIPTOR,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE
    ),

    // Handles 0xfe06: Read characteristic (ATVV_CHAR_RX), handle 0xfe07 characteristic value.
    CHARACTERISTIC_UUID128
    (
        HANDLE_ATV_VOICE_CTL_CHARACTERISTIC,
        HANDLE_ATV_VOICE_CTL_CHARACTERISTIC_VALUE,
        UUID_ATV_VOICE_CTL_CHARACTERISTIC,
        GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_READ,
        GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_READABLE
    ),

    // Handle 0xfe08
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_ATV_VOICE_CTL_CLIENT_CONFIGURATION_DESCRIPTOR,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_CMD | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_AUTH_WRITABLE
    ),
#endif

#ifdef OTA_FIRMWARE_UPGRADE
 #ifdef OTA_SECURE_FIRMWARE_UPGRADE
    // Handle 0xff00: Cypress vendor specific WICED Secure OTA Upgrade Service.
    PRIMARY_SERVICE_UUID128
        (HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_SEC_FW_UPGRADE_SERVICE),
 #else
    // Handle 0xff00: Cypress vendor specific WICED OTA Upgrade Service.
    PRIMARY_SERVICE_UUID128
        ( HANDLE_OTA_FW_UPGRADE_SERVICE, UUID_OTA_FW_UPGRADE_SERVICE ),
 #endif

    // Handles 0xff03: characteristic WS Control Point, handle 0xff04 characteristic value.
    CHARACTERISTIC_UUID128_WRITABLE
    (
        HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
        HANDLE_OTA_FW_UPGRADE_CONTROL_POINT,
        UUID_OTA_FW_UPGRADE_CHARACTERISTIC_CONTROL_POINT,
        GATTDB_CHAR_PROP_WRITE | GATTDB_CHAR_PROP_NOTIFY | GATTDB_CHAR_PROP_INDICATE,
        GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_WRITE_REQ /*| GATTDB_PERM_AUTH_WRITABLE*/
    ),

    // Declare client characteristic configuration descriptor
    // Value of the descriptor can be modified by the client
    // Value modified shall be retained during connection and across connection
    // for bonded devices.  Setting value to 1 tells this application to send notification
    // when value of the characteristic changes.  Value 2 is to allow indications.
    CHAR_DESCRIPTOR_UUID16_WRITABLE
    (
        HANDLE_OTA_FW_UPGRADE_CLIENT_CONFIGURATION_DESCRIPTOR,
        UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
        GATTDB_PERM_READABLE | GATTDB_PERM_WRITE_REQ /*| GATTDB_PERM_AUTH_WRITABLE */
    ),

    // Handle 0xff07: characteristic WS Data, handle 0xff08 characteristic value. This
    // characteristic is used to send next portion of the FW Similar to the control point
    CHARACTERISTIC_UUID128_WRITABLE
    (
        HANDLE_OTA_FW_UPGRADE_CHARACTERISTIC_DATA,
        HANDLE_OTA_FW_UPGRADE_DATA,
        UUID_OTA_FW_UPGRADE_CHARACTERISTIC_DATA,
        GATTDB_CHAR_PROP_WRITE,
        GATTDB_PERM_VARIABLE_LENGTH | GATTDB_PERM_WRITE_REQ | GATTDB_PERM_RELIABLE_WRITE
    ),
#endif
};

/********************************************************************************
 * Function Name: BLE_connparamupdate_timeout
 ********************************************************************************
 * Summary: Connection parameter update timer timeout handling function
 *
 * Parameters:
 *  arg -- not used
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void BLE_connparamupdate_timeout( uint32_t arg )
{
    //request connection param update if it not requested before
    if ( !hidd_blelink_conn_param_updated()
         // if we are not in the middle of OTAFWU
         && !ota_is_active()
       )
    {
        hidd_blelink_conn_param_update();
    }
}

/********************************************************************************
 * Function Name: BLE_clientConfWriteRptStd
 ********************************************************************************
 * Summary: Client characteritics conf write handler for standard key report
 *
 * Parameters:
 *   reportType -- Report type
 *   reportId -- Report ID
 *   payload -- pointer to payload
 *   payloadSize -- payload size
 *
 * Return:
 *   none
 *
 *******************************************************************************/
static void BLE_clientConfWriteRptStd(wiced_hidd_report_type_t reportType,
                                      uint8_t reportId,
                                      void *payload,
                                      uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

//    WICED_BT_TRACE("\nclientConfWriteRptStd");

    ble_updateClientConfFlags(notification, APP_CLIENT_CONFIG_NOTIF_STD_KEY_RPT);
}

/********************************************************************************
 * Function Name: BLE_clientConfWriteRptBitMapped
 ********************************************************************************
 * Summary: Client characteritics conf write handler for bitmap report
 *
 * Parameters:
 *   reportType -- Report type
 *   reportId -- Report ID
 *   payload -- pointer to payload
 *   payloadSize -- payload size
 *
 * Return:
 *   none
 *
 *******************************************************************************/
static void BLE_clientConfWriteRptBitMapped(wiced_hidd_report_type_t reportType,
                                            uint8_t reportId,
                                            void *payload,
                                            uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

//    WICED_BT_TRACE("\nclientConfWriteRptBitMapped");

    ble_updateClientConfFlags(notification, APP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT);
}

/********************************************************************************
 * Function Name: BLE_clientConfWriteBatteryRpt
 ********************************************************************************
 * Summary: Client characteritics conf write handler for bitmap report
 *
 * Parameters:
 *   reportType -- Report type
 *   reportId -- Report ID
 *   payload -- pointer to payload
 *   payloadSize -- payload size
 *
 * Return:
 *   none
 *
 *******************************************************************************/
static void BLE_clientConfWriteBatteryRpt(wiced_hidd_report_type_t reportType,
                                     uint8_t reportId,
                                     void *payload,
                                     uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t  indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

//    WICED_BT_TRACE("\nclientConfWriteBatteryRpt");

    ble_updateClientConfFlags(notification, APP_CLIENT_CONFIG_NOTIF_BATTERY_RPT);
}

/********************************************************************************
 * Function Name: BLE_clientConfWriteBatteryRpt
 ********************************************************************************
 * Summary: Client characteritics conf write handler for user defined key report
 *
 * Parameters:
 *   reportType -- Report type
 *   reportId -- Report ID
 *   payload -- pointer to payload
 *   payloadSize -- payload size
 *
 * Return:
 *   none
 *
 *******************************************************************************/
static void BLE_clientConfWriteRptUserDefinedKey(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    ble_updateClientConfFlags(notification, APP_CLIENT_CONFIG_NOTIF_USER_DEFINED_KEY_RPT);
}

#ifdef HID_AUDIO
/********************************************************************************
 * Function Name: BLE_clientConfWriteBatteryRpt
 ********************************************************************************
 * Summary: Client characteritics conf write handler for audio report
 *
 * Parameters:
 *   reportType -- Report type
 *   reportId -- Report ID
 *   payload -- pointer to payload
 *   payloadSize -- payload size
 *
 * Return:
 *   none
 *
 *******************************************************************************/
static void BLE_clientConfWriteRptVoice(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    ble_updateClientConfFlags(notification, APP_CLIENT_CONFIG_NOTIF_VOICE_RPT);
}

/********************************************************************************
 * Function Name: BLE_clientConfWriteBatteryRpt
 ********************************************************************************
 * Summary: Client characteristics conf write handler for audio control report
 *
 * Parameters:
 *   reportType -- Report type
 *   reportId -- Report ID
 *   payload -- pointer to payload
 *   payloadSize -- payload size
 *
 * Return:
 *   none
 *
 *******************************************************************************/
static void BLE_clientConfWriteRptVoiceCtrl(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //uint8_t indication = *(uint16_t *)payload & GATT_CLIENT_CONFIG_INDICATION;

    ble_updateClientConfFlags(notification, APP_CLIENT_CONFIG_NOTIF_VOICE_CTRL_RPT);
}
#endif

#ifdef SUPPORT_TOUCHPAD
/********************************************************************************
 * Function Name: BLE_clientConfWriteBatteryRpt
 ********************************************************************************
 * Summary: Client characteristics conf write handler for touchpad report
 *
 * Parameters:
 *   reportType -- Report type
 *   reportId -- Report ID
 *   payload -- pointer to payload
 *   payloadSize -- payload size
 *
 * Return:
 *   none
 *
 *******************************************************************************/
static void BLE_clientConfWriteRptTouchpad(wiced_hidd_report_type_t reportType,
                                 uint8_t reportId,
                                 void *payload,
                                 uint16_t payloadSize)
{
    uint8_t  notification = *(uint16_t *)payload & GATT_CLIENT_CONFIG_NOTIFICATION;
    //WICED_BT_TRACE("\napp_clientConfWriteRptTouchpad: notification=%x, payload=%x", notification, *(uint16_t *)payload);

    ble_updateClientConfFlags(notification, APP_CLIENT_CONFIG_NOTIF_TOUCHPAD_RPT);
}
#endif

/********************************************************************************
 * Function Name: BLE_clientConfWriteBatteryRpt
 ********************************************************************************
 * Summary: Client characteristics conf write handler for HID control point
 *
 * Parameters:
 *   reportType -- Report type
 *   reportId -- Report ID
 *   payload -- pointer to payload
 *   payloadSize -- payload size
 *
 * Return:
 *   none
 *
 *******************************************************************************/
static void BLE_ctrlPointWrite(wiced_hidd_report_type_t reportType,
                          uint8_t reportId,
                          void *payload,
                          uint16_t payloadSize)
{
//    WICED_BT_TRACE("\ndisconnecting");

    hidd_link_disconnect();
}

/*********************************************************************************
 * Gatt Map for Report Mode
 ********************************************************************************/
wiced_blehidd_report_gatt_characteristic_t reportModeGattMap[] =
{
    // STD keyboard Input report
    {
        .reportId           =RPT_ID_IN_STD_KEY,
        .reportType         =WICED_HID_REPORT_TYPE_INPUT,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =NULL,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_STD_KEY_RPT
    },
    // Client char config for STD keyboard Input report
    {
        .reportId           =RPT_ID_CLIENT_CHAR_CONF,
        .reportType         =WICED_HID_CLIENT_CHAR_CONF,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_INPUT_CHAR_CFG_DESCR,
        .sendNotification   =FALSE,
        .writeCallback      =BLE_clientConfWriteRptStd,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },
    // Std output report
    {
        .reportId           =RPT_ID_OUT_KB_LED,
        .reportType         =WICED_HID_REPORT_TYPE_OUTPUT,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_STD_OUTPUT_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =app_setReport,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },
    //Bitmapped input report
    {
        .reportId           =RPT_ID_IN_BIT_MAPPED,
        .reportType         =WICED_HID_REPORT_TYPE_INPUT,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =NULL,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_BIT_MAPPED_RPT
    },
    // Client char config for Bitmapped input report
    {
        .reportId           =RPT_ID_CLIENT_CHAR_CONF,
        .reportType         =WICED_HID_CLIENT_CHAR_CONF,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_BITMAP_CHAR_CFG_DESCR,
        .sendNotification   =FALSE,
        .writeCallback      =BLE_clientConfWriteRptBitMapped,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },
    // HID Control point
    {
        .reportId           =RPT_ID_IN_NOT_USED,
        .reportType         =WICED_HID_REPORT_TYPE_OTHER,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_HID_CTRL_POINT_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =BLE_ctrlPointWrite,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },
    // user defined 0 (media) key input report
    {
        .reportId           =RPT_ID_IN_MEDIA_KEY,
        .reportType         =WICED_HID_REPORT_TYPE_INPUT,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =NULL,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_USER_DEFINED_KEY_RPT
    },
    // Client char config for user defined 0 (media) key input report
    {
        .reportId           =RPT_ID_CLIENT_CHAR_CONF,
        .reportType         =WICED_HID_CLIENT_CHAR_CONF,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_USER_DEFINED_0_CHAR_CFG_DESCR,
        .sendNotification   =FALSE,
        .writeCallback      =BLE_clientConfWriteRptUserDefinedKey,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },
    // Battery Input report
    {
        .reportId           =RPT_ID_IN_BATTERY,
        .reportType         =WICED_HID_REPORT_TYPE_INPUT,
        .handle             =HANDLE_APP_BATTERY_SERVICE_CHAR_LEVEL_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =NULL,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_BATTERY_RPT
    },
    {
        .reportId           =RPT_ID_CLIENT_CHAR_CONF,
        .reportType         =WICED_HID_CLIENT_CHAR_CONF,
        .handle             =HANDLE_APP_BATTERY_SERVICE_CHAR_CFG_DESCR,
        .sendNotification   =FALSE,
        .writeCallback      =BLE_clientConfWriteBatteryRpt,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },
#ifdef SUPPORT_TOUCHPAD
    // touchpad input report
    {
        .reportId           =RPT_ID_IN_ABS_XY,
        .reportType         =WICED_HID_REPORT_TYPE_INPUT,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =NULL,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_TOUCHPAD_RPT
    },
    // Client char config for touchpad input report
    {
        .reportId           =RPT_ID_CLIENT_CHAR_CONF,
        .reportType         =WICED_HID_CLIENT_CHAR_CONF,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_TOUCHPAD_CHAR_CFG_DESCR,
        .sendNotification   =FALSE,
        .writeCallback      =BLE_clientConfWriteRptTouchpad,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },
#endif
#ifdef HID_AUDIO
    //voice data input report
    {
        .reportId           =HIDD_VOICE_REPORT_ID,
        .reportType         =WICED_HID_REPORT_TYPE_INPUT,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =NULL,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_VOICE_RPT
    },
    //voice ctrl input report
    {
        .reportId           =RPT_ID_IN_AUDIO_CTL,
        .reportType         =WICED_HID_REPORT_TYPE_INPUT,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =NULL,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_VOICE_CTRL_RPT
    },
    // voice ctrl feature report from host
    {
        .reportId           =RPT_ID_FEATURE_AUDIO_CTL,
        .reportType         =WICED_HID_REPORT_TYPE_FEATURE,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_FEA_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =app_setReport,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },
    // Client char config for voice data input report
    {
        .reportId           =RPT_ID_CLIENT_CHAR_CONF,
        .reportType         =WICED_HID_CLIENT_CHAR_CONF,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CHAR_CFG_DESCR,
        .sendNotification   =FALSE,
        .writeCallback      =BLE_clientConfWriteRptVoice,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },
    // Client char config for voice ctrl input report
    {
        .reportId           =RPT_ID_CLIENT_CHAR_CONF,
        .reportType         =WICED_HID_CLIENT_CHAR_CONF,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_VOICE_CTRL_INPUT_CHAR_CFG_DESCR,
        .sendNotification   =FALSE,
        .writeCallback      =BLE_clientConfWriteRptVoiceCtrl,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },
  #endif

    //connection control feature report from host
    {
        .reportId           =RPT_ID_FEATURE_CNT_CTL,
        .reportType         =WICED_HID_REPORT_TYPE_FEATURE,
        .handle             =HANDLE_APP_LE_HID_SERVICE_HID_RPT_CONNECTION_CTRL_VAL,
        .sendNotification   =FALSE,
        .writeCallback      =app_setReport,
        .clientConfigBitmap =APP_CLIENT_CONFIG_NOTIF_NONE
    },

};

/********************************************************************************
 * Function Name: BLE_updateGattMapWithNotifications
 ********************************************************************************
 * Summary: GATT map flag update
 *
 * Parameters:
 *   flags - flag to be updated
 *
 * Return:
 *   none
 *
 *******************************************************************************/
static void BLE_updateGattMapWithNotifications(uint16_t flags)
{
    uint8_t i;
    wiced_blehidd_report_gatt_characteristic_t* map = reportModeGattMap;

    //update cccd for gatt read req
    for (i=0; i<BLE_RPT_INDX_MAX; i++)
    {
        cccd[i] = (flags >> i) & GATT_CLIENT_CONFIG_NOTIFICATION;
    }

    for(i = 0; i < sizeof(reportModeGattMap)/sizeof(reportModeGattMap[0]); i++)
    {
        if(map->reportType == WICED_HID_REPORT_TYPE_INPUT)
        {
            map->sendNotification =
                ((flags & map->clientConfigBitmap) == map->clientConfigBitmap) ? TRUE : FALSE;
        }

        map++;
    }
}

/********************************************************************************
 * Function Name: BLE_transportStateChangeNotification
 ********************************************************************************
 * Summary: This function informs the application that the state of a link changed.
 *
 * Parameters:
 *   newState -- new state of the link
 *
 * Return:
 *   none
 *
 *******************************************************************************/
static void BLE_transportStateChangeNotification(uint32_t newState)
{
    int16_t flags;

    switch (newState) {
    case HIDLINK_LE_CONNECTED:
        //get host client configuration characteristic descriptor values
        flags = hidd_host_get_flags(blelink.gatts_peer_addr, blelink.gatts_peer_addr_type);
        if(flags != -1)
        {
            WICED_BT_TRACE("\nhost config flag:%08x",flags);
            BLE_updateGattMapWithNotifications(flags);
        }
        else
        {
            WICED_BT_TRACE("\nhost NOT found!");
        }

        //start 15 second timer to make sure connection param update is requested before SDS
        wiced_start_timer(&ble.conn_param_update_timer,15000); //15 seconds. timeout in ms
        break;
    }

    // tell applicaton state is changed
    app_transportStateChangeNotification(newState);
}

#ifdef FASTPAIR_ENABLE
 #define BLE_setupPairingData() BLE_init_fast_pair()
/*
 * Initiate Google Fast Pair Service Provider
 */
static void BLE_init_fast_pair(void)
{
    wiced_bt_gfps_provider_conf_t fastpair_conf = {0};
    static wiced_bt_ble_advert_elem_t app_adv_elem;

    /* set Tx power level data type in ble advertisement */
 #if defined(CYW20719B2) || defined(CYW20721B2) || defined(CYW20819A1) || defined (CYW20820A1)
    fastpair_conf.ble_tx_pwr_level = wiced_bt_cfg_settings.default_ble_power_level;
 #else
    fastpair_conf.ble_tx_pwr_level = 0;
 #endif

    /* set assigned handles for GATT attributes */
    fastpair_conf.gatt_db_handle.key_pairing_val        = HANDLE_APP_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_VAL;
    fastpair_conf.gatt_db_handle.key_pairing_cfg_desc   = HANDLE_APP_FASTPAIR_SERVICE_CHAR_KEY_PAIRING_CFG_DESC;
    fastpair_conf.gatt_db_handle.passkey_val            = HANDLE_APP_FASTPAIR_SERVICE_CHAR_PASSKEY_VAL;
    fastpair_conf.gatt_db_handle.passkey_cfg_desc       = HANDLE_APP_FASTPAIR_SERVICE_CHAR_PASSKEY_CFG_DESC;
    fastpair_conf.gatt_db_handle.account_key_val        = HANDLE_APP_FASTPAIR_SERVICE_CHAR_ACCOUNT_KEY_VAL;

    /* model id */
    fastpair_conf.model_id = FASTPAIR_MODEL_ID;

    /* anti-spoofing public key */
    memcpy((void *) &fastpair_conf.anti_spoofing_key.public[0],
           (void *) &anti_spoofing_public_key[0],
           WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PUBLIC);

    /* anti-spoofing private key */
    memcpy((void *) &fastpair_conf.anti_spoofing_key.private[0],
           (void *) &anti_spoofing_private_key[0],
           WICED_BT_GFPS_ANTI_SPOOFING_KEY_LEN_PRIVATE);

    /* Account Key Filter generate format */
    fastpair_conf.account_key_filter_generate_random = WICED_TRUE;;

    /* Account Key list size */
    fastpair_conf.account_key_list_size = FASTPAIR_ACCOUNT_KEY_NUM;

    /* NVRAM id for Account Key list */
    fastpair_conf.account_key_list_nvram_id = VS_ID_GFPS_ACCOUNT_KEY;

    /* BLE advertisement data appended to fast pair advertisement data */
    app_adv_elem.advert_type    = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    app_adv_elem.len            = sizeof(BT_LOCAL_NAME)-1;
    app_adv_elem.p_data         = (uint8_t *)dev_local_name;

    fastpair_conf.appended_adv_data.p_elem      = &app_adv_elem;
    fastpair_conf.appended_adv_data.elem_num    = 1;

    /* Initialize Google Fast Pair Service. */
    if (hidd_gatts_gfps_init(&fastpair_conf) == WICED_FALSE)
    {
        WICED_BT_TRACE("wiced_bt_gfps_provider_init fail\n");
    }
}

#else
 #define BLE_setupPairingData() BLE_setUpAdvData()
/********************************************************************************
 * Function Name: void BLE_setUpAdvData(void)
 ********************************************************************************
 * Summary: set up LE Advertising data
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void BLE_setUpAdvData(void)
{
    wiced_bt_ble_advert_elem_t app_adv_elem[4];
    uint8_t app_adv_flag         = BTM_BLE_LIMITED_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint16_t app_adv_appearance  = APPEARANCE_GENERIC_REMOTE_CONTROL;
    uint16_t app_adv_service     = UUID_SERVCLASS_LE_HID;

    // flag
    app_adv_elem[0].advert_type  = BTM_BLE_ADVERT_TYPE_FLAG;
    app_adv_elem[0].len          = sizeof(uint8_t);
    app_adv_elem[0].p_data       = &app_adv_flag;

    // Appearance
    app_adv_elem[1].advert_type  = BTM_BLE_ADVERT_TYPE_APPEARANCE;
    app_adv_elem[1].len          = sizeof(uint16_t);
    app_adv_elem[1].p_data       = (uint8_t *)&app_adv_appearance;

    //16 bits Service: UUID_SERVCLASS_LE_HID
    app_adv_elem[2].advert_type  = BTM_BLE_ADVERT_TYPE_16SRV_COMPLETE;
    app_adv_elem[2].len          = sizeof(uint16_t);
    app_adv_elem[2].p_data       = (uint8_t *)&app_adv_service;

    //dev name
    app_adv_elem[3].advert_type  = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    app_adv_elem[3].len          = sizeof(BT_LOCAL_NAME)-1;
    app_adv_elem[3].p_data       = (uint8_t *)dev_local_name;

    wiced_bt_ble_set_raw_advertisement_data(4, app_adv_elem);
}
#endif

/********************************************************************************
 * Function Name: uint16_t ble_get_cccd_flag(CLIENT_CONFIG_NOTIF_T idx)
 ********************************************************************************
 * Summary: Get report flags
 *
 * Parameters:
 *  idx -- report index
 *
 * Return:
 *  report flags
 *
 *******************************************************************************/
uint16_t ble_get_cccd_flag(CLIENT_CONFIG_NOTIF_T idx)
{
    return (idx < BLE_RPT_INDX_MAX) ? cccd[idx] : 0;
}

/********************************************************************************
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
void ble_updateClientConfFlags(uint16_t enable, uint16_t featureBit)
{
    BLE_updateGattMapWithNotifications(hidd_host_set_flags(blelink.gatts_peer_addr, enable, featureBit));
}

/********************************************************************************
 * Function Name: void ble_init()
 ********************************************************************************
 * Summary: Bluetooth transport init.
 *
 * Parameters:
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void ble_init()
{
    WICED_BT_TRACE("\nble_init");

    /*  LE GATT DB Initialization  */
    hidd_gatts_init( reportModeGattMap, sizeof(reportModeGattMap)/sizeof(wiced_blehidd_report_gatt_characteristic_t),
                     blehid_db_data, sizeof(blehid_db_data),
                     blehid_gattAttributes, blehid_gattAttributes_size,
                     blehid_app_gatts_req_read_callback, blehid_app_gatts_req_write_callback);

    BLE_setupPairingData();

    //timer to request connection param update
    wiced_init_timer( &ble.conn_param_update_timer, BLE_connparamupdate_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    hidd_blelink_add_state_observer(BLE_transportStateChangeNotification);
}



#endif // BLE_SUPPORT
