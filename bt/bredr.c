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
 * BR/EDR function and data
 *
 */
#ifdef BR_EDR_SUPPORT
#include "app.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"

///////////////////////////////////////////////////////////////////////////////////
// BR/EDR Link related defines
///////////////////////////////////////////////////////////////////////////////////
#define LINK_SUPERVISION_TIMEOUT_IN_SLOTS   3200

// Use this to find out the value of SPD_RPT_DESCRIPTOR_SIZE, if the value is over 255, need to use two-byte field instead
//char data[] = {USB_BREDR_RPT_DESCRIPTOR};
// WICED_BT_TRACE("\nSize of SPD_RPT_DESCRIPTOR_SIZE is %d", sizeof(data));  -- located in bredr_init()
#define SPD_RPT_DESCRIPTOR_SIZE 195

/*****************************************************************************
 * This is the SDP database for the BT HID KB application.
 * It defines 2 service records:
 *     UUID_SERVCLASS_HUMAN_INTERFACE,
 *     UUID_SERVCLASS_PNP_INFORMATION.
 ****************************************************************************/
static const UINT8 wiced_bt_sdp_db[] =
{
    // length is the sum of all records
    SDP_ATTR_SEQUENCE_2(259 + SPD_RPT_DESCRIPTOR_SIZE +                 //HID Device
                        162),                                           //Device ID

    // First SDP record HIDD: Keyboard
    SDP_ATTR_SEQUENCE_2(256 + SPD_RPT_DESCRIPTOR_SIZE),                 // 3 bytes for length of the record + length of the record
    SDP_ATTR_RECORD_HANDLE(0x10001),                                    // 8 byte ==>0x9, 0x0, 0x0, 0xA, 0x00, 0x01, 0x00, 0x02
    SDP_ATTR_CLASS_ID(UUID_SERVCLASS_HUMAN_INTERFACE),                  // 8 bytes ==>0x9, 0x0, 0x1, 0x35, 0x3, 0x19, 0x11, 0x24
    SDP_ATTR_HIDP_PROTOCOL_DESC_LIST(HID_PSM_CONTROL),                  // 18 bytes ==>0x9, 0x0, 0x4, 0x35, 0xD, 0x35, 0x06, 0x19, 0x01, 0x00, 0x9, 0x00, 0x11, 0x35, 0x3, 0x19, 0x0, 0x011
    SDP_ATTR_BROWSE_LIST,                                               // 8 bytes ==>0x9, 0x00, 0x05, 0x35, 0x3, 0x19, 0x10, 0x02
    SDP_ATTR_LANGUAGE_BASE_ATTR_ID_LIST,                                // 14 bytes ==>0x9, 0x00, 0x06, 0x35, 0x9, 0x9, 0x65, 0x6E, 0x9, 0x00, 0x6A, 0x9, 0x01, 0x00
    SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_HUMAN_INTERFACE, 0x0111),      //13 bytes ==>0x9, 0x00, 0x09, 0x35, 0x8, 0x35, 0x6, 0x19, 0x11, 0x24, 0x9, 0x01, 0x11
    SDP_ATTR_ID(ATTR_ID_ADDITION_PROTO_DESC_LISTS), SDP_ATTR_SEQUENCE_1(15), // 5 bytes ==>0x9, 0x00, 0x0D, 0x35, 0x0F,
        SDP_ATTR_SEQUENCE_1(13),                                        // 2 bytes ==> 0x35, 0x0D,
        SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes ==> 0x35, 0x6,
        SDP_ATTR_UUID16(UUID_PROTOCOL_L2CAP),                           // 3 bytes ==> 0x19, 0x01, 0x00,
            SDP_ATTR_ID(HID_PSM_INTERRUPT),                             // 3 bytes ==> 0x9, 0x00, 0x13,
        SDP_ATTR_SEQUENCE_1(3),                                         // 2 bytes ==> 0x35, 0x03,
            SDP_ATTR_UUID16(UUID_PROTOCOL_HIDP),                        // 3 bytes ==> 0x19, 0x00, 0x11,
    SDP_ATTR_SERVICE_NAME(35),                                          // 40 bytes ==>0x9, 0x01, 0x00, 0x25, 0x23,
        'C', 'y', 'p', 'r', 'e', 's', 's', ' ',
        'B', 'l', 'u', 'e', 't', 'o', 'o', 't', 'h', ' ',
        'W', 'i', 'r', 'e', 'l', 'e', 's', 's', ' ',
        'K', 'e', 'y', 'b', 'o', 'a', 'r', 'd',
    SDP_ATTR_SERVICE_DESCRIPTION(8),                                    //13 bytes==>0x9, 0x01, 0x01, 0x25, 0x8,
        'K', 'e', 'y', 'b', 'o', 'a', 'r', 'd',
    SDP_ATTR_PROVIDER_NAME(21),                                         //26 byptes ==> 0x9, 0x01, 0x02, 0x25, 0x15,
        'C', 'y', 'p', 'r', 'e', 's', 's', ' ',
        'S', 'e', 'm', 'i', 'c', 'o', 'n', 'd', 'u', 'c', 't', 'o', 'r',
    SDP_ATTR_UINT2(ATTR_ID_HID_PARSER_VERSION, 0x0111),                 // 6 bytes==>0x9, 0x02, 0x01, 0x9, 0x01, 0x11(v1.1.1)
    SDP_ATTR_UINT1(ATTR_ID_HID_DEVICE_SUBCLASS, 0x40),                  // 5 bytes==>0x9, 0x02, 0x01, 0x8, 0x40(keyboard)
    SDP_ATTR_UINT1(ATTR_ID_HID_COUNTRY_CODE, 0x21),                     // 5 bytes==>0x9, 0x02, 0x01, 0x8, 0x21(USA)
    SDP_ATTR_BOOLEAN(ATTR_ID_HID_VIRTUAL_CABLE, HID_DEV_VIRTUAL_CABLE),          // 5 bytes==>0x9, 0x02, 0x01, 0x28, 0x1(TRUE)
    SDP_ATTR_BOOLEAN(ATTR_ID_HID_RECONNECT_INITIATE, HID_DEV_RECONN_INITIATE),   // 5 bytes==>0x9, 0x02, 0x01, 0x28, 0x1(TRUE)

    SDP_ATTR_ID(ATTR_ID_HID_DESCRIPTOR_LIST), SDP_ATTR_SEQUENCE_1(SPD_RPT_DESCRIPTOR_SIZE+6), // 5 bytes ==>0x9, 0x02, 0x06, 0x35, 0xEF,
        SDP_ATTR_SEQUENCE_1(SPD_RPT_DESCRIPTOR_SIZE+4),                 // 2 bytes ==>0x35, 0xED,
        SDP_ATTR_VALUE_UINT1(0x22),                                     // 2 bytes ==> 0x08, 0x22(Report)
        SDP_ATTR_VALUE_TEXT_1(SPD_RPT_DESCRIPTOR_SIZE),                 // 2+SPD_RPT_DESCRIPTOR_SIZE bytes ==> 0x25, SPD_RPT_DESCRIPTOR_SIZE,
        USB_BREDR_RPT_DESCRIPTOR,

    SDP_ATTR_ID(ATTR_ID_HID_LANGUAGE_ID_BASE), SDP_ATTR_SEQUENCE_1(8),  // 5 bytes ==>0x9, 0x02, 0x07, 0x35, 0x8,
        SDP_ATTR_SEQUENCE_1(6),                                         // 2 bytes ==>0x35, 0x6,
        SDP_ATTR_VALUE_UINT2(0x0409),                                   // 3 bytes ==>0x9, 0x04, 0x09 (English)
        SDP_ATTR_VALUE_UINT2(LANGUAGE_BASE_ID),                         // 3 bytes ==>0x9, 0x01, 0x00
    SDP_ATTR_BOOLEAN(ATTR_ID_HID_BATTERY_POWER, HID_DEV_BATTERY_POW),                    // 5 bytes==>0x9, 0x02, 0x09, 0x28, 0x1(TRUE)
    SDP_ATTR_BOOLEAN(ATTR_ID_HID_REMOTE_WAKE, HID_DEV_REMOTE_WAKE),                      // 5 bytes==>0x9, 0x02, 0x0A, 0x28, 0x1(TRUE)
    SDP_ATTR_UINT2(ATTR_ID_HID_LINK_SUPERVISION_TO, LINK_SUPERVISION_TIMEOUT_IN_SLOTS),  // 6 bytes==>0x9, 0x02, 0x0C, 0x9, 0x0C, 0x80 (0xC80=3200 slots = 2 seconds)
    SDP_ATTR_BOOLEAN(ATTR_ID_HID_NORMALLY_CONNECTABLE, HID_DEV_NORMALLY_CONN),           // 5 bytes==>0x9, 0x02, 0x0D, 0x28, 0x0(FALSE)
    SDP_ATTR_BOOLEAN(ATTR_ID_HID_BOOT_DEVICE, 0x01),                    // 5 bytes==>0x9, 0x02, 0x0E, 0x28, 0x1(TRUE)
    SDP_ATTR_UINT2(ATTR_ID_HID_SSR_HOST_MAX_LAT, 792),                  // 6 bytes==>0x9, 0x02, 0x0F, 0x9, 0x03, 0x18 (0xC80=792 slots = 495 mS)
    SDP_ATTR_UINT2(ATTR_ID_HID_SSR_HOST_MIN_TOUT, 0),                   // 6 bytes==>0x9, 0x02, 0x10, 0x9, 0x0, 0x0 (recommend 0x00 for this value)

    // Second SDP record Device ID
    SDP_ATTR_SEQUENCE_1(160),                                           // 2 bytes for length of the record  + length of the record
    SDP_ATTR_RECORD_HANDLE(0x10002),                                    // 8 bytes ==>0x9, 0x0, 0x0, 0xA, 0x00, 0x01, 0x00, 0x03
    SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),                  // 8 bytes ==>0x9, 0x0, 0x1, 0x35, 0x3, 0x19, 0x12, 0x00
    SDP_ATTR_SDP_PROTOCOL_DESC_LIST(SDP_PSM),                           // 18 bytes ==>0x9, 0x0, 0x4, 0x35, 0xD, 0x35, 0x06, 0x19, 0x01, 0x00, 0x9, 0x00, 0x01, 0x35, 0x3,0x19, 0x00, 0x01
    SDP_ATTR_LANGUAGE_BASE_ATTR_ID_LIST,                                // 14 bytes ==>0x9, 0x00, 0x06, 0x35, 0x9, 0x9, 0x65, 0x6E, 0x9, 0x00, 0x6A, 0x9, 0x01, 0x00
    SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_PNP_INFORMATION, 0x0100), // 13 bytes ==>0x9, 0x00, 0x09, 0x35, 0x8, 0x35, 0x6, 0x19, 0x12, 0x00, 0x9, 0x01, 0x00
    SDP_ATTR_SERVICE_NAME(46),                                          // 51 bytes ==>0x9, 0x01, 0x00, 0x25, 0x2E,
        'C', 'y', 'p', 'r', 'e', 's', 's', ' ',
        'B', 'l', 'u', 'e', 't', 'o', 'o', 't', 'h', ' ',
        'W', 'i', 'r', 'e', 'l', 'e', 's', 's', ' ',
        'R', 'e', 'm', 'o', 't', 'e', ' ', ' ', ' ',
        'P', 'n', 'P', ' ', 'S', 'e', 'r', 'v', 'e', 'r',
    SDP_ATTR_SERVICE_DESCRIPTION(8),                                    //13 bytes==>0x9, 0x01, 0x01, 0x25, 0x8,
        'R', 'e', 'm', 'o', 't', 'e', ' ', ' ',
    SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, BLUETOOTH_DI_SPECIFICATION), // 6 bytes==>0x9, 0x02, 0x00, 0x9, 0x01, 0x03
    SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x131),                           // 6 bytes==>0x9, 0x02, 0x01, 0x9, 0x01, 0x31 (Cypress)
    SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x8502),                         // 6 bytes==>0x9, 0x02, 0x02, 0x9, 0x85, 0x02 (Keyboard)
    SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    // 6 bytes==>0x9, 0x02, 0x03, 0x9, 0x00, 0x01
    SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     // 5 bytes==>0x9, 0x02, 0x04, 0x28, 0x1(TRUE)
    SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG),// 6 bytes==>0x9, 0x02, 0x05, 0x9, 0x00, 0x01

};
//const uint16_t wiced_bt_sdp_db_size = (sizeof(wiced_bt_sdp_db));

/*****************************************************************************
 * BT HID power management states configuration
 ****************************************************************************/
static wiced_bt_hidd_pm_pwr_state_t bthid_powerStateList[] =
{
    {
        POWER_MODE_HIGH,            // mode
        0,                          // maxNegotiationAttempts 0= infinite
        60000,                      // timeoutToNextInMs. 60s in this state
        SNIFF_NORMAL,               // type
        1,                         // asymmetricMultiplier
        {
            36,                     // 36 slots
            36,                     // 36 slots
            1,                      // 1 attempt
            1                       // 0 timeout
        }
    },
    {
        POWER_MODE_DISCONNECT,      // BtLowPowerMode
        0,                          // dont care
        0,                          // dont care
        SNIFF_NORMAL,               // dont care
        1,                           // dont care
        {
            0,                       // dont care
            0,                       // dont care
            0,                       // dont care
            0                        // dont care
        }
    }
};
//const uint8_t bthid_powerStateList_num = sizeof(bthid_powerStateList)/sizeof(wiced_bt_hidd_pm_pwr_state_t);

/*****************************************************************************
 * BT HID power management states configuration used when sniff subrate (SSR) is enabled by host
 ****************************************************************************/
static wiced_bt_hidd_pm_pwr_state_t bthid_SSRPowerStatesList[] =
{
    {
        POWER_MODE_IDLE,
        0,
        60000,                           // Next state after 60s
        SNIFF_NORMAL,
        1,
        {
            36,                      // dont care
            36,                      // dont care
            1,                       // dont care
            0                        // dont care
        }
    },
    {
        POWER_MODE_DISCONNECT,       // PowerMode
        0,                           // dont care
        0,                           // dont care
        SNIFF_NORMAL,                // dont care
        1,                           // dont care
        {
            0,                       // dont care
            0,                       // dont care
            0,                       // dont care
            0                        // dont care
        }
    }
};
//const uint8_t bthid_SSRPowerStatesList_num = sizeof(bthid_SSRPowerStatesList)/sizeof(wiced_bt_hidd_pm_pwr_state_t);

////////////////////////////////////////////////////////////////////////////////
//Prepare extended inquiry response data.  Current version HID service.
////////////////////////////////////////////////////////////////////////////////
#define WICED_HID_EIR_BUF_MAX_SIZE      264
static void BREDR_write_eir(const char * dev_name)
{
    uint8_t pBuf[WICED_HID_EIR_BUF_MAX_SIZE] = {0, };
    uint8_t *p = NULL;
    uint8_t length = strlen(dev_name);

    p = pBuf;

    /* Update the length of the name (Account for the type field(1 byte) as well) */
    *p++ = (1 + length);
    *p++ = 0x09;            // EIR type full name

    /* Copy the device name */
    memcpy(p, dev_name, length);
    p += length;


    *p++ = ( 1 * 2 ) + 1;     // length of services + 1
    *p++ = 0x02;            // EIR type full list of 16 bit service UUIDs
    *p++ = UUID_SERVCLASS_HUMAN_INTERFACE & 0xff;
    *p++ = ( UUID_SERVCLASS_HUMAN_INTERFACE >> 8 ) & 0xff;
    *p++ = 0;

    // print EIR data
    STRACE_ARRAY("\nEIR:\n", (uint8_t*) (pBuf + 1), p - (uint8_t*) pBuf);
    wiced_bt_dev_write_eir(pBuf, (uint16_t) (p - pBuf));
}

/*******************************************************************************
 * Function Name: void bredr_init()
 ********************************************************************************
 * Summary: Bluetooth BR/EDR transport init.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void bredr_init()
{
    /* BR/EDR SDP database init */
    wiced_bt_sdp_db_init(( uint8_t* )wiced_bt_sdp_db, sizeof(wiced_bt_sdp_db));

    // Use this to find out the value of SPD_RPT_DESCRIPTOR_SIZE for the define
    // WICED_BT_TRACE("\nSize of SPD_RPT_DESCRIPTOR_SIZE is %d", sizeof(data));

    /* BT HID power management configuration  */
//    wiced_bt_hidd_configure_power_management_params(bthid_powerStateList, sizeof(bthid_powerStateList)/sizeof(wiced_bt_hidd_pm_pwr_state_t),
//                                               bthid_SSRPowerStatesList, sizeof(bthid_SSRPowerStatesList)/sizeof(wiced_bt_hidd_pm_pwr_state_t));

    /* initialize eir */
    BREDR_write_eir((char *) dev_local_name);
}
#endif
