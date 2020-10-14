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
 * Remote control
 *
 * This file provides definitions and function prototypes for remote control
 * device
 *
 */

#ifndef __APP_H__
#define __APP_H__
#include "wiced.h"
#include "wiced_hidd_lib.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "hidd_lib.h"

/*******************************************************************************
* Types and Defines
*******************************************************************************/
#if defined(BLE_SUPPORT) && defined(BR_EDR_SUPPORT)
 #define BT_LOCAL_NAME "CY REMOTE"
#elif defined(BLE_SUPPORT)
 #define BT_LOCAL_NAME "CY LE REMOTE"
#else
 #define BT_LOCAL_NAME "CY BT REMOTE"
#endif

#if is_20735Family
 #define NUM_KEYSCAN_ROWS    5  // Num of Rows in keyscan matrix
 #define NUM_KEYSCAN_COLS    4  // Num of Cols in keyscan matrix
 #define CONNECT_KEY_INDEX   18 // need to find out from hardware
 #define AUDIO_KEY_INDEX     16
 #define IR_KEY_INDEX        17
#else
 #define NUM_KEYSCAN_ROWS    7  // Num of Rows in keyscan matrix
 #define NUM_KEYSCAN_COLS    7  // Num of Cols in keyscan matrix
 #define IR_KEY_INDEX        0
 #define CONNECT_KEY_INDEX   29 // need to find out from hardware
 #define AUDIO_KEY_INDEX     23
 #if !defined(REMOTE_PLATFORM)
  #define FAKE_CONNECT_KEY_INDEX   8 // need to find out from hardware
  #define FAKE_AUDIO_KEY_INDEX     16
 #endif
#endif
#define USE_TOUCHPAD_VIRTUAL_KEY 0

#define LED_RED             WICED_PLATFORM_LED_1
#define LED_GREEN           WICED_PLATFORM_LED_2

#define LED_ERROR           LED_RED
#define LED_LE_LINK         LED_GREEN

typedef void (app_poll_callback_t)(void);
/*******************************************************************************
 * Report ID defines
 ********************************************************************************/
// Input report id
typedef enum {
    RPT_ID_IN_STD_KEY      =0x01,
    RPT_ID_IN_BIT_MAPPED   =0x02,
    RPT_ID_IN_BATTERY      =0x03,
    RPT_ID_IN_TOUCHPAD     =0x07,
    RPT_ID_IN_MOUSE        =0x08,
    RPT_ID_IN_MEDIA_KEY    =0x0a,
    RPT_ID_IN_ABS_XY       =0x20,
    RPT_ID_IN_CNT_CTL      =0xcc,
    RPT_ID_IN_AUDIO_DATA   =WICED_HIDD_VOICE_REPORT_ID,
    RPT_ID_IN_AUDIO_CTL    =WICED_HIDD_VOICE_CTL_REPORT_ID,
    RPT_ID_IN_NOT_USED     =0xff,
    RPT_ID_CLIENT_CHAR_CONF=0xff,
} rpt_id_in_e;

// Output report id
typedef enum {
    RPT_ID_OUT_KB_LED      =0x01,
    RPT_ID_OUT_AUDIO_DATA=WICED_HIDD_VOICE_REPORT_ID,
    RPT_ID_OUT_AUDIO_CTL =WICED_HIDD_VOICE_CTL_REPORT_ID,
} rpt_id_out_e;

// Feature report id
typedef enum {
    RPT_ID_FEATURE_CNT_CTL =0xcc,
    RPT_ID_FEATURE_AUDIO_DATA=WICED_HIDD_VOICE_REPORT_ID,
    RPT_ID_FEATURE_AUDIO_CTL =WICED_HIDD_VOICE_CTL_REPORT_ID,
} rpt_id_feature_e;

/*******************************************************************************
 * App queue defines
 ********************************************************************************/
typedef union {
    uint8_t                     type;
    HidEvent                    info;
    HidEventMotionXY            mouse;
    HidEventButtonStateChange   button;
    HidEventKey                 key;
    HidEventAny                 any;
    HidEventUserDefine          user;
} app_queue_t;

#define APP_QUEUE_SIZE sizeof(app_queue_t)
#define APP_QUEUE_MAX  44                         // max number of event in queue

/*******************************************************************************
 * Include all components
 *******************************************************************************/
#include "battery.h"
#include "ota.h"
#include "bt.h"
#include "key.h"
#include "audio.h"
#include "touchPad.h"
#include "ir.h"
#include "findme.h"

/*******************************************************************************
 * Function Name: app_setReport
 ********************************************************************************
 * Summary:
 *  This function implements the rxSetReport function defined by
 *  the HID application to handle "Set Report" messages.
 *  This function looks at the report ID and passes the message to the
 *  appropriate handler.
 *
 * Parameters:
 *  reportType -- type of incoming report,  WICED_HID_REPORT_TYPE_INPUT, WICED_HID_REPORT_TYPE_OUTPUT, or WICED_HID_REPORT_TYPE_FEATURE
 *  reportId -- report id
 *  payload -- pointer to data that came along with the set report request after the report ID
 *  payloadSize -- size of the payload excluding the report ID
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_setReport(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize);

/*******************************************************************************
 * Function Name: app_queueEvent
 ********************************************************************************
 * Summary:
 *  Queue an event to event queue
 *
 * Parameters:
 *  event -- event to queue
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_queueEvent(app_queue_t * event);

/*******************************************************************************
 * Function Name: app_transportStateChangeNotification
 ********************************************************************************
 * Summary:
 *  This function is called when the state of a link is changed.
 *
 * Parameters:
 *  newState new state of the link
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_transportStateChangeNotification(uint32_t newState);

/*******************************************************************************
 * Function Name: app_start
 ********************************************************************************
 * Summary: This is application start function. After system initialization is done, when the
 *          bt management calls with BTM_ENABLED_EVT, this function is called to
 *          start application
 *
 * Parameters:
 *  none
 *
 * Return:
 *  WICED_BT_SUCCESS -- if application initialization is okay and ready to start;
 *                      otherwise, should return the error code defined in wiced_result_t
 *
 *******************************************************************************/
wiced_result_t app_start();

#endif // __BLEREMOTE_H__
