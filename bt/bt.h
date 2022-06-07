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
 * BR/EDR function and data
 *
 */
#ifndef __APP_BT_H__
#define __APP_BT_H__

#include "ble.h"
#include "bredr.h"

extern uint8_t blehid_rpt_map[];

#if defined(MOUSE_XY_SIZE_16)
# define MOUSE_XY_SIZE 16
#elif defined(MOUSE_XY_SIZE_12)
# define MOUSE_XY_SIZE 12
#else
# define MOUSE_XY_SIZE 8
#endif

#define HAS_MOUSE_REPORT  defined(SUPPORT_TOUCHPAD)

#define STD_KB_REPORT_DESCRIPTOR \
    /* RPT_ID_IN_STD_KEY */ \
    /* Input Report, 8 bytes */ \
    /* 1st byte:Keyboard LeftControl/Keyboard Right GUI */ \
    /* 2nd byte:Constant, 3rd ~ 6th: keycode */ \
    /* Output Report, 1 byte: LED control */ \
    0x05, 0x01,                    /* USAGE_PAGE (Generic Desktop) */ \
    0x09, 0x06,                    /* USAGE (Keyboard) */ \
    0xA1, 0x01,                    /* COLLECTION (Application) */ \
    0x85, RPT_ID_IN_STD_KEY,       /*    REPORT_ID */ \
    0x75, 0x01,                    /*    REPORT_SIZE (1) */ \
    0x95, 0x08,                    /*    REPORT_COUNT (8) */ \
    0x05, 0x07,                    /*    USAGE_PAGE (Keyboard) */ \
    0x19, 0xE0,                    /*    USAGE_MINIMUM (Keyboard LeftControl) */ \
    0x29, 0xE7,                    /*    USAGE_MAXIMUM (Keyboard Right GUI) */ \
    0x15, 0x00,                    /*    LOGICAL_MINIMUM (0) */ \
    0x25, 0x01,                    /*    LOGICAL_MAXIMUM (1) */ \
    0x81, 0x02,                    /*    INPUT (Data,Var,Abs) */ \
    0x95, 0x01,                    /*    REPORT_COUNT (1) */ \
    0x75, 0x08,                    /*    REPORT_SIZE (8) */ \
    0x81, 0x03,                    /*    INPUT (Cnst,Var,Abs) */ \
    0x95, 0x05,                    /*    REPORT_COUNT (5) */ \
    0x75, 0x01,                    /*    REPORT_SIZE (1) */ \
    0x05, 0x08,                    /*    USAGE_PAGE (LEDs) */ \
    0x19, 0x01,                    /*    USAGE_MINIMUM (Num Lock) */ \
    0x29, 0x05,                    /*    USAGE_MAXIMUM (Kana) */ \
    0x91, 0x02,                    /*    OUTPUT (Data,Var,Abs) */ \
    0x95, 0x01,                    /*    REPORT_COUNT (1) */ \
    0x75, 0x03,                    /*    REPORT_SIZE (3) */ \
    0x91, 0x03,                    /*    OUTPUT (Cnst,Var,Abs) */ \
    0x95, 0x06,                    /*    REPORT_COUNT (6) */ \
    0x75, 0x08,                    /*    REPORT_SIZE (8) */ \
    0x15, 0x00,                    /*    LOGICAL_MINIMUM (0) */ \
    0x26, 0xFF, 0x00,              /*    LOGICAL_MAXIMUM (255) */ \
    0x05, 0x07,                    /*    USAGE_PAGE (Keyboard) */ \
    0x19, 0x00,                    /*    USAGE_MINIMUM (Reserved (no event indicated)) */ \
    0x29, 0xFF,                    /*    USAGE_MAXIMUM (Reserved (no event indicated)) */ \
    0x81, 0x00,                    /*    INPUT (Data,Ary,Abs) */ \
    0xC0,                          /* END_COLLECTION */

enum bitmapped_key
{
    BITMAP_AC_BACK,   // bit 0
    BITMAP_AC_HOME,   // bit 1
    BITMAP_AC_SEARCH, // bit 2
    BITMAP_MENU_LEFT, // bit 3
    BITMAP_MENU_UP,   // bit 4
    BITMAP_MENU_DOWN, // bit 5
    BITMAP_VOL_DOWN,  // bit 6
    BITMAP_VOL_UP,    // bit 7

    BITMAP_MENU_RIGHT,// bit 8
    BITMAP_MENU_PICK, // bit 9
    BITMAP_PLAY_PAUSE,// bit 10
    BITMAP_MUTE,      // bit 11
    BITMAP_FAST_FORWD,// bit 12
    BITMAP_REWIND,    // bit 13
    BITMAP_CH_UP,     // bit 14
    BITMAP_CH_DOWN,   // bit 15

    BITMAP_CH,        // bit 16
    BITMAP_ODR_MOVIE, // bit 17
    BITMAP_PREVIOUS,  // bit 18 (Recall Last)
    BITMAP_POWER,     // bit 19
    BITMAP_SHOPPING,  // bit 20
    BITMAP_VIEW_TGL,  // bit 21
    BITMAP_NUMBER,    // bit 22
    BITMAP_NEXT_TRACK,// bit 23

    BITMAP_PREV_TRACK,// bit 24
    BITMAP_REVIEW,    // bit 25
    BITMAP_MENU,      // bit 26
    BITMAP_MAX,

};

#define USAGE_AC_BACK       0x0A, 0x24, 0x02
#define USAGA_AC_HOME       0x0A, 0x23, 0x02
#define USAGE_AC_SEARCH     0x0A, 0x21, 0x02
#define USAGE_MENU_LEFT     0x09, 0x44
#define USAGE_MENU_UP       0x09, 0x42
#define USAGE_MENU_DOWN     0x09, 0x43
#define USAGE_VOL_DOWN      0x09, 0xEA
#define USAGE_VOL_UP        0x09, 0xE9
#define USAGE_MENU_RIGHT    0x09, 0x45
#define USAGE_MENU_PICK     0x09, 0x41
#define USAGE_MENU_PAUSE    0x09, 0xCD
#define USAGE_MUTE          0x09, 0xE2
#define USAGE_FAST_FORWRD   0x09, 0xB3
#define USAGE_REWIND        0x09, 0xB4
#define USAGE_CH_UP         0x09, 0x9C
#define USAGE_CH_DOWN       0x09, 0x9D
#define USAGE_CH            0x09, 0x86
#define USAGE_ODR_MOVIE     0x09, 0x85
#define USAGE_RECALL_LAST   0x09, 0x83
#define USAGE_POWER         0x09, 0x30
#define USAGE_SHOPPING      0x0A, 0xC1, 0x01
#define USAGE_VIEW_TOGGLE   0x0A, 0x32, 0x02
#define USAGE_NUMBER_LIST   0x0A, 0x58, 0x02
#define USAGE_NEXT_TRACK    0x09, 0xB5
#define USAGE_PREV_TRACK    0x09, 0xB6
#define USAGE_PR_REVIEW     0x0A, 0x67, 0x02
#define USAGE_MENU          0x09, 0x40

#define BITMAPPED_REPORT_DESCRIPTOR \
    /* Bit mapped report, BITMAPPED_REPORT_ID */ \
    0x05, 0x0C,                    /* USAGE_PAGE (Consumer Devices) */ \
    0x09, 0x01,                    /* USAGE (Consumer Control) */ \
    0xA1, 0x01,                    /* COLLECTION (Application) */ \
    0x85, RPT_ID_IN_BIT_MAPPED,    /*    REPORT_ID (2) */ \
    0x15, 0x00,                    /*    LOGICAL_MINIMUM (0) */ \
    0x25, 0x01,                    /*    LOGICAL_MAXIMUM (1) */ \
    0x75, 0x01,                    /*    REPORT_SIZE (1) */ \
    0x95, BITMAP_MAX,              /*    REPORT_COUNT (27) */ \
    /* byte 0 */ \
    USAGE_AC_BACK,                 /*0   USAGE (AC Back) */ \
    USAGA_AC_HOME,                 /*1   USAGE (AC Home) */ \
    USAGE_AC_SEARCH,               /*2   USAGE (AC Search) */ \
    USAGE_MENU_LEFT,               /*3   USAGE (Menu Left) */ \
    USAGE_MENU_UP,                 /*4   USAGE (Menu Up) */ \
    USAGE_MENU_DOWN,               /*5   USAGE (Menu Down) */ \
    USAGE_VOL_DOWN,                /*6   USAGE (Volume Down) */ \
    USAGE_VOL_UP,                  /*7   USAGE (Volume Up) */ \
    /* byte 1 */ \
    USAGE_MENU_RIGHT,              /*8   USAGE (Menu Right) */ \
    USAGE_MENU_PICK,               /*9   USAGE (Menu Pick) */ \
    USAGE_MENU_PAUSE,              /*10  USAGE (Play/Pause) */ \
    USAGE_MUTE,                    /*11  USAGE (Mute) */ \
    USAGE_FAST_FORWRD,             /*12  USAGE (Fast Forward) */ \
    USAGE_REWIND,                  /*13  USAGE (Rewind) */ \
    USAGE_CH_UP,                   /*14  USAGE (Fast Forward) */ \
    USAGE_CH_DOWN,                 /*15  USAGE (Rewind) */ \
    /* byte 2 */ \
    USAGE_CH,                      /*16  USAGE (Channel) */ \
    USAGE_ODR_MOVIE,               /*17  USAGE (Order Movie) */ \
    USAGE_RECALL_LAST,             /*18  USAGE (Recall Last) */ \
    USAGE_POWER,                   /*19  USAGE (Power) */ \
    USAGE_SHOPPING,                /*20  USAGE (Online Shopping Brower) */ \
    USAGE_VIEW_TOGGLE,             /*21  USAGE (View Toggle) */ \
    USAGE_NUMBER_LIST,             /*22  USAGE (Number List) */ \
    USAGE_NEXT_TRACK,              /*23  USAGE (Next Track) */ \
    /* byte 3 */ \
    USAGE_PREV_TRACK,              /*24  USAGE (Previous Track) */ \
    USAGE_PR_REVIEW,               /*25  USAGE (Print Review) */ \
    USAGE_MENU,                    /*26  USAGE (Menu) */ \
    0x81, 0x02,                    /*    INPUT (Data,Var,Abs) */ \
    0x95, 0x01,                    /*    REPORT_COUNT (1) */ \
    0x75, 8-(BITMAP_MAX % 8),      /*    REPORT_SIZE (?) */ \
    0x81, 0x03,                    /*    INPUT (Cnst,Var,Abs) */ \
    0xC0,                          /* END_COLLECTION */

#define MEDIA_KEY_REPORT_DESCRIPTOR \
    /* Media key report */ \
    0x05, 0x0C,                    /* Usage Page (Consumer Devices) */ \
    0x09, 0x01,                    /* Usage (Consumer Control) */ \
    0xA1, 0x01,                    /* Collection (Application) */ \
    0x85, RPT_ID_IN_MEDIA_KEY,     /*    Report ID */ \
    0x95, 0x08,                    /*    REPORT_COUNT (8) */ \
    0x75, KEY_NUM_BYTES_IN_USER_DEFINED_REPORT, /*    REPORT_SIZE (8) */ \
    0x15, 0x00,                    /*    LOGICAL_MINIMUM (0) */ \
    0x26, 0xFF , 0x00,             /*    LOGICAL_MAXIMUM (255) */ \
    0x05, 0x07,                    /*    USAGE_PAGE (Keyboard) */ \
    0x19, 0x00,                    /*    USAGE_MINIMUM (Reserved (no event indicated)) */ \
    0x29, 0xFF,                    /*    USAGE_MAXIMUM (Reserved (no event indicated)) */ \
    0x81, 0x00,                    /*    INPUT (Data,Ary,Abs) */ \
    0xC0,                          /* END_COLLECTION */

#if HAS_MOUSE_REPORT
#define MOUSE_REPORT_DESCRIPTOR \
    /* Mouse report, RPT_ID_IN_MOUSE */ \
    0x05, 0x01,             /* Usage Page (Generic Desktop), */ \
    0x09, 0x02,             /* Usage (Mouse), */ \
    0xA1, 0x01,             /* Collection: (Application), */ \
    0x85, RPT_ID_IN_MOUSE,  /*     REPORT_ID  */ \
    0x09, 0x01,             /*     Usage (Pointer), */ \
    0xA1, 0x00,             /*     Collection: (Linked), */ \
    0x05, 0x09,             /*         Usage Page (Buttons), */ \
    0x19, 0x01,             /*         Usage Minimum (01), */ \
    0x29, 0x03,             /*         Usage Maximum (03), */ \
    0x15, 0x00,             /*         Log Min (0), */ \
    0x25, 0x01,             /*         Log Max (1), */ \
    0x75, 0x01,             /*         Report Size (1), */ \
    0x95, 0x03,             /*         Report Count (03), */ \
    0x81, 0x02,             /*         Input (Data, Variable, Absolute), */ \
    0x75, 0x05,             /*         Report Size (5), */ \
    0x95, 0x01,             /*         Report Count (1), */ \
    0x81, 0x01,             /*         Input (Constant), */ \
    0x05, 0x01,             /*         Usage Page (Generic Desktop), */ \
    0x09, 0x30,             /*         Usage (X), */ \
    0x09, 0x31,             /*         Usage (Y), */ \
    0x15, 0x81,             /*         Logical min (-127), */ \
    0x25, 0x7F,             /*         Logical Max (127), */ \
    0x75, MOUSE_XY_SIZE,    /*         Report Size (16,12, or 8), */ \
    0x95, 0x02,             /*         Report Count (2) (X,Y) */ \
    0x81, 0x06,             /*         Input (Data, Variable, Relative), */ \
    0x09, 0x38,             /*         Usage (Wheel), */ \
    0x15, 0x81,             /*         Logical min (-127), */ \
    0x25, 0x7F,             /*         Logical Max (127), */ \
    0x75, 0x08,             /*         Report Size (8), */ \
    0x95, 0x01,             /*         Report Count (1) (Wheel) */ \
    0x81, 0x06,             /*         Input (Data, Variable, Relative), */ \
    0xC0,                   /*     END_COLLECTION (Logical) */ \
    0xC0,                   /* END_COLLECTION */
#else
#define MOUSE_REPORT_DESCRIPTOR
#endif

#ifdef HID_AUDIO
#define AUDIO_LE_REPORT_DESCRIPTOR \
    /*audio report, HIDD_VOICE_REPORT_ID */ \
    0x05, 0x0C,                                         /* Usage Page (Consumer Devices) */ \
    0x09, 0x01,                                         /* Usage (Consumer Control) */ \
    0xA1, 0x01,                                         /* Collection (Application) */ \
    0x85, RPT_ID_IN_AUDIO_DATA,                         /*    Report ID */ \
    0x96, AUDIO_DATA_SIZE & 0xff, AUDIO_DATA_SIZE >> 8, /*    REPORT_COUNT (audio data size) */ \
    0x75, 0x08,                                         /*    REPORT_SIZE (8) */ \
    0x15, 0x00,                                         /*    LOGICAL_MINIMUM (0) */ \
    0x26, 0xFF, 0x00,                                   /*    LOGICAL_MAXIMUM (255) */ \
    0x81, 0x00,                                         /*    INPUT (Data,Ary,Abs) */ \
    0xC0,                                               /* END_COLLECTION */

#define AUDIO_BREDR_REPORT_DESCRIPTOR \
    /*audio report, HIDD_VOICE_REPORT_ID */ \
    0x05, 0x0C,                                         /* Usage Page (Consumer Devices) */ \
    0x09, 0x01,                                         /* Usage (Consumer Control) */ \
    0xA1, 0x01,                                         /* Collection (Application) */ \
    0x85, RPT_ID_IN_AUDIO_DATA,                         /*    Report ID */ \
    0x95, 0x0B,                                         /*    REPORT_COUNT (11) */ \
    0x75, 0x08,                                         /*    REPORT_SIZE (8) */ \
    0x81, 0x00,                                         /*    INPUT (Data,Ary,Abs) */ \
    0x15, 0x00,                                         /*    LOGICAL_MINIMUM (0) */ \
    0x26, 0xFF, 0x00,                                   /*    LOGICAL_MAXIMUM (255) */ \
    0x96, AUDIO_DATA_SIZE & 0xff, AUDIO_DATA_SIZE >> 8, /*    REPORT_COUNT (audio data size) */ \
    0x75, 0x08,                                         /*    REPORT_SIZE (8) */ \
    0x15, 0x00,                                         /*    LOGICAL_MINIMUM (0) */ \
    0x26, 0xFF, 0x00,                                   /*    LOGICAL_MAXIMUM (255) */ \
    0x81, 0x00,                                         /*    INPUT (Data,Ary,Abs) */ \
    0xC0,                                               /* END_COLLECTION */

#define AUDIO_CTRL_REPORT_DESCRIPTOR \
    /*voice ctrl report, HIDD_VOICE_CTL_REPORT_ID */ \
    0x05, 0x0C,                    /* Usage Page (Consumer Devices) */ \
    0x09, 0x01,                    /* Usage (Consumer Control) */ \
    0xA1, 0x01,                    /* Collection (Application) */ \
    0x85, RPT_ID_IN_AUDIO_CTL,     /*    Report ID=0xF8 */ \
    0x95, 0x0B,                    /*    REPORT_COUNT (11) */ \
    0x75, 0x08,                    /*    REPORT_SIZE (8) */ \
    0x15, 0x00,                    /*    LOGICAL_MINIMUM (0) */ \
    0x26, 0xFF , 0x00,             /*    LOGICAL_MAXIMUM (255) */ \
    0x81, 0x00,                    /*    INPUT (Data,Ary,Abs) */ \
    0x95, 0x0B,                    /*    REPORT_COUNT (11) */ \
    0x75, 0x01,                    /*    REPORT_SIZE (8) */ \
    0x15, 0x00,                    /*    LOGICAL_MINIMUM (0) */ \
    0x26, 0xFF , 0x00,             /*    LOGICAL_MAXIMUM (255) */ \
    0xB1, 0x00,                    /*    FEATURE (Data,Ary,Abs) */ \
    0xC0,                          /* END_COLLECTION */
#else
#define AUDIO_LE_REPORT_DESCRIPTOR
#define AUDIO_BREDR_REPORT_DESCRIPTOR
#define AUDIO_CTRL_REPORT_DESCRIPTOR
#endif

#ifdef SUPPORT_TOUCHPAD
#define TOUCHPAD_REPORT_DESCRIPTOR \
    /*touchpad report,  RPT_ID_IN_IN_ABS_XY */ \
    0x05, 0x0C,                    /* Usage Page (Consumer Devices) */ \
    0x09, 0x01,                    /* Usage (Consumer Control) */ \
    0xA1, 0x01,                    /* COLLECTION (Application) */ \
    0x85, RPT_ID_IN_ABS_XY,        /*    REPORT_ID (0x20) */ \
    0x95, 0x09,                    /*    REPORT_COUNT (9) */ \
    0x75, 0x08,                    /*    REPORT_SIZE (8) */ \
    0x15, 0x00,                    /*    LOGICAL_MINIMUM (0) */ \
    0x26, 0xFF , 0x00,             /*    LOGICAL_MAXIMUM (255) */ \
    0x81, 0x00,                    /*    INPUT (Data,Ary,Abs) */ \
    0xC0,                          /* END_COLLECTION */
#else
#define TOUCHPAD_REPORT_DESCRIPTOR
#endif

// Use BATTERY_REPORT_DESCRIPTOR for the last entry because it has no ',' in the end
#define BATTERY_REPORT_DESCRIPTOR \
    /*Battery report */ \
    0x05, 0x0C,                    /* Usage Page (Consumer Devices), */ \
    0x09, 0x01,                    /* Usage (Consumer Control), */ \
    0xA1, 0x01,                    /* COLLECTION (Application) */ \
    0x85, RPT_ID_IN_BATTERY,       /*    REPORT_ID (3) */ \
    0x05, 0x01,                    /*    Usage Page (Generic Desktop), */ \
    0x09, 0x06,                    /*    Usage (Keyboard) */ \
    0xA1, 0x02,                    /*    Collection: (Logical), */ \
    0x05, 0x06,                    /*        USAGE PAGE (Generic Device Control), */ \
    0x09, 0x20,                    /*        USAGE (Battery Strength), */ \
    0x15, 0x00,                    /*        Log Min (0), */ \
    0x26, 0x64 , 0x00,             /*        Log Max (255), */ \
    0x75, 0x08,                    /*        Report Size (8), */ \
    0x95, 0x01,                    /*        Report Count (1), */ \
    0x81, 0x02,                    /*        Input (Data, Variable, Absolute), */ \
    0xC0,                          /*    END_COLLECTION (Logical) */ \
    0xC0                           /* END_COLLECTION */

#define USB_LE_RPT_DESCRIPTOR \
{\
  STD_KB_REPORT_DESCRIPTOR \
  BITMAPPED_REPORT_DESCRIPTOR \
  MEDIA_KEY_REPORT_DESCRIPTOR \
  MOUSE_REPORT_DESCRIPTOR \
  AUDIO_LE_REPORT_DESCRIPTOR \
  AUDIO_CTRL_REPORT_DESCRIPTOR \
  TOUCHPAD_REPORT_DESCRIPTOR \
  BATTERY_REPORT_DESCRIPTOR \
}

#define USB_BREDR_RPT_DESCRIPTOR \
  STD_KB_REPORT_DESCRIPTOR \
  BITMAPPED_REPORT_DESCRIPTOR \
  MEDIA_KEY_REPORT_DESCRIPTOR \
  MOUSE_REPORT_DESCRIPTOR \
  AUDIO_BREDR_REPORT_DESCRIPTOR \
  AUDIO_CTRL_REPORT_DESCRIPTOR \
  TOUCHPAD_REPORT_DESCRIPTOR \
  BATTERY_REPORT_DESCRIPTOR

/*******************************************************************************
 * Function Name: void bt_init()
 ********************************************************************************
 * Summary: Bluetooth transport init.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void bt_init();

#else
#define bt_init()
#endif // __APP_BT_H__
