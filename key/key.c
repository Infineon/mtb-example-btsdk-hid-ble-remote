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
 * Keyscan Interface
 *
 */

#ifdef SUPPORT_KEY_REPORT

#include "app.h"
#include "usb_usage.h"

#define CODE_ROLLOVER 1

/// Keyboard Key Config
typedef PACKED struct
{
    /// Type of key, e.g. std key, modifier key, etc.
    uint8_t    type;

    /// Translation code. The actual value depend on the key type.
    ///     - For modifier keys, it  is a bit mask for the reported key
    ///     - For std key, it is the usage provided in the std key report
    ///     - For bit mapped keys, it is the row/col of the associated bit in the bit mapped report
    uint8_t    translationValue;
}KbKeyConfig;


/// Key types. Used to direct key codes to the relevant key processing function
enum KeyType
{
    /// Represents no key. This should not occur normally
    KEY_TYPE_NONE,

    /// Represents a standard key. The associated translation code represents the reported value
    /// of this key
    KEY_TYPE_STD,

    /// Represents a modifier key. The associated translation value indicates which bit
    /// in the modifier key mask is controlled by this key
    KEY_TYPE_MODIFIER,

    /// Represents a bit mapped key in the bit mapped report. The associated translation value
    /// provides the row col of the bit which represents this key
    KEY_TYPE_BIT_MAPPED,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_USER_0,

    /// A user defined key. Interpretation is provided by user code
    KEY_TYPE_MAX
};

/*****************************************************************************
/// Key translation table. It maps keyscan matrix position to key types and
/// specific usage within the type. For example, row 5, column 6 may be
/// mapped as a standard key with usage "ESCAPE". This means that the key
/// will be reported in the standard report with a USB usage of "ESCAPE"
/// See config documentation for details and the keyboard config for an example.
/// By default this table is initialized for the BCM keyboard
*****************************************************************************/
KbKeyConfig kbKeyConfig[] =
{
#if is_20835Family // 5x4
 #ifdef ANDROID_AUDIO
    // Column 0:  order is row0 ->row4
    {KEY_TYPE_STD,              USB_USAGE_1},       //#0, 1
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MENU_UP},    //#1, 2 (menu up)
    {KEY_TYPE_STD,              USB_USAGE_3},       //#2, 3
    {KEY_TYPE_BIT_MAPPED,       BITMAP_VOL_DOWN},   //#3, VOL_DOWN
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MENU_PICK},  //#4, ENTER touchpad

    // Column 1: order is row0 ->row4
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MENU_LEFT},  //#5, 4 (menu left)
    {KEY_TYPE_STD,              USB_USAGE_5},       //#6, 5
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MENU_RIGHT}, //#7, 6 (menu right)
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MUTE},       //#8, MUTE
    {KEY_TYPE_BIT_MAPPED,       BITMAP_AC_BACK},    //#9, BACK

   // Column 2: order is row0 ->row4
    {KEY_TYPE_STD,              USB_USAGE_7},       //#10, 7
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MENU_DOWN},  //#11, 8 (menu down)
    {KEY_TYPE_STD,              USB_USAGE_9},       //#12, 9
    {KEY_TYPE_BIT_MAPPED,       BITMAP_VOL_UP},     //#13, VOL_UP
    {KEY_TYPE_BIT_MAPPED,       BITMAP_AC_HOME},    //#14, HOME

    // Column 3: order is row0 ->row4
    {KEY_TYPE_BIT_MAPPED,       BITMAP_AC_SEARCH},  //#15, AUDIO
    {KEY_TYPE_STD,              USB_USAGE_0},       //#16, 0
    {KEY_TYPE_STD,              USB_USAGE_POWER},   //#17, PWR
    {KEY_TYPE_NONE,             0},                 //#18, Heart (discoverable)
    {KEY_TYPE_BIT_MAPPED,       BITMAP_AC_BACK},    //#19, EXIT
 #else
    // Column 0:  order is row0 ->row4
    {KEY_TYPE_STD,              0x1e},  //#0, 1
    {KEY_TYPE_STD,              0x1f},  //#1, 2
    {KEY_TYPE_STD,              0x20},  //#2, 3
    {KEY_TYPE_STD,              0x81},  //#3, VOL_DOWN
    {KEY_TYPE_STD,              0x28},  //#4, ENTER touchpad

    // Column 1: order is row0 ->row4
    {KEY_TYPE_STD,              0x21},  //#5,  4
    {KEY_TYPE_STD,              0x22},  //#6,  5
    {KEY_TYPE_STD,              0x23},  //#7, 6
    {KEY_TYPE_STD,              0x7f},  //#8, MUTE
    {KEY_TYPE_STD,              0xf1},  //#9, BACK

   // Column 2: order is row0 ->row4
    {KEY_TYPE_STD,              0x24},  //#10, 7
    {KEY_TYPE_STD,              0x25},  //#11, 8
    {KEY_TYPE_STD,              0x26},  //#12, 9
    {KEY_TYPE_STD,              0x80},  //#13, VOL_UP
    {KEY_TYPE_STD,              0x4a},  //#14, HOME

    // Column 3: order is row0 ->row4
    {KEY_TYPE_STD,              0x27},  //#15, 0
    {KEY_TYPE_NONE,                0},  //#16, AUDIO
    {KEY_TYPE_STD,              0x66},  //#17, PWR
    {KEY_TYPE_NONE,                0},  //#18, Heart (discoverable)
    {KEY_TYPE_BIT_MAPPED,          0},  //#19, EXIT
  #endif
#else // 7x7
    // Column 0:  order is row0 ->row6
    {KEY_TYPE_NONE,             USB_USAGE_NO_EVENT},    //0  #KB1,     TV_POWER --> IR
    {KEY_TYPE_NONE,             USB_USAGE_NO_EVENT},    //1  #KB8,     Touch_on/off
    {KEY_TYPE_BIT_MAPPED,       BITMAP_PLAY_PAUSE},     //2  #KB15,    Play
    {KEY_TYPE_BIT_MAPPED,       BITMAP_AC_HOME},        //3  #KB22,    Home
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MENU_RIGHT},     //4  #KB29,    Right
    {KEY_TYPE_BIT_MAPPED,       BITMAP_CH},             //5  #KB36,    Chnnel Chart
    {KEY_TYPE_STD,              USB_USAGE_7},           //6  ##KB43,    7

    // Column 1: order is row0 ->row6
    {KEY_TYPE_STD,              USB_USAGE_VOL_UP},      //7  #KB2,     TV Volume UP
    {KEY_TYPE_STD,              USB_USAGE_MENU},        //8  #KB9,     Menu
    {KEY_TYPE_BIT_MAPPED,       BITMAP_FAST_FORWD},     //9  #KB16,    >>
    {KEY_TYPE_BIT_MAPPED,       BITMAP_VIEW_TGL},       //10 #KB23,    Multview
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MENU_DOWN},      //11 #KB30,    Down
    {KEY_TYPE_STD,              USB_USAGE_1},           //12 #KB37,    1
    {KEY_TYPE_STD,              USB_USAGE_8},           //13 #KB44,    8

    // Column 2: order is row0 ->row6
    {KEY_TYPE_STD,              USB_USAGE_VOL_DOWN},    //14 #KB3,     TV Volume Down
    {KEY_TYPE_BIT_MAPPED,       BITMAP_ODR_MOVIE},      //15 #KB10,    Movie
    {KEY_TYPE_BIT_MAPPED,       BITMAP_PREV_TRACK},     //16 #KB17,    |<<
    {KEY_TYPE_BIT_MAPPED,       BITMAP_CH_UP},          //17 #KB24,    Channel UP
    {KEY_TYPE_BIT_MAPPED,       BITMAP_AC_BACK},        //18 #KB31,    Exit
    {KEY_TYPE_STD,              USB_USAGE_2},           //19 #KB38,    2
    {KEY_TYPE_STD,              USB_USAGE_9},           //20 #KB45,    9

    // Column 3: order is row0 ->row6
    {KEY_TYPE_BIT_MAPPED,       BITMAP_POWER},          //21 #KB4,     Power
    {KEY_TYPE_BIT_MAPPED,       BITMAP_REVIEW},         //22 #KB11,    Review
    {KEY_TYPE_BIT_MAPPED,       BITMAP_AC_SEARCH},      //23 #KB18,    Voice
    {KEY_TYPE_BIT_MAPPED,       BITMAP_CH_DOWN},        //24 #KB25,    Channel Down
    {KEY_TYPE_BIT_MAPPED,       BITMAP_PREVIOUS},       //25 #KB32,    Previous
    {KEY_TYPE_STD,              USB_USAGE_3},           //26 #KB39,    3
    {KEY_TYPE_STD,              USB_USAGE_5},           //27 #KB46,    *

    // Column 4: order is row0 ->row6
    {KEY_TYPE_BIT_MAPPED,       BITMAP_POWER},          //28 #KB5,     STB Power
    {KEY_TYPE_BIT_MAPPED,       BITMAP_SHOPPING},       //29 #KB12,    Shopping
    {KEY_TYPE_BIT_MAPPED,       BITMAP_NEXT_TRACK},     //30 #KB19,    >>|
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MENU_UP},        //31 #KB26,    Up
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MENU},           //32 #KB33,    Option
    {KEY_TYPE_STD,              USB_USAGE_4},           //33 #KB40,    4
    {KEY_TYPE_STD,              USB_USAGE_0},           //34 #KB47,    0

    // Column 5: order is row0 ->row6
    {KEY_TYPE_STD,              USB_USAGE_MUTE},        //35 #KB6,     Mute
    {KEY_TYPE_BIT_MAPPED,       BITMAP_REWIND},         //36 #KB13,    <<
    {KEY_TYPE_BIT_MAPPED,       BITMAP_MENU_LEFT},      //37 #KB20,    Volume Up
    {KEY_TYPE_STD,              USB_USAGE_LEFT_ARROW},  //38 #KB27,    Left
    {KEY_TYPE_STD,              USB_USAGE_ENTER},       //39 #KB34,    Search
    {KEY_TYPE_STD,              USB_USAGE_5},           //40 #KB41,    5
    {KEY_TYPE_BIT_MAPPED,       BITMAP_NUMBER},         //41 #KB48,    #

    // Column 6: order is row0 ->row6
    {KEY_TYPE_STD,              USB_USAGE_NO_EVENT},    //42 #KB7,     Input
    {KEY_TYPE_STD,              USB_USAGE_STOP},        //43 #KB14,    Stop
    {KEY_TYPE_STD,              USB_USAGE_VOL_DOWN},    //44 #KB21,    Volume Down
    {KEY_TYPE_STD,              USB_USAGE_ENTER},       //45 #KB28,    OK
    {KEY_TYPE_BIT_MAPPED,       BITMAP_CH},             //46 #KB35,    Fav Channel
    {KEY_TYPE_STD,              USB_USAGE_6},           //47 #KB42,    6
    {KEY_TYPE_NONE,             USB_USAGE_NO_EVENT},    //48 #KB49,    N/A
#endif

#if USE_TOUCHPAD_VIRTUAL_KEY
    // This column defines the virtual keys for touchpad
    {KEY_TYPE_STD,              USB_USAGE_ENTER},       // VIRTUAL CENTER
    {KEY_TYPE_STD,              USB_USAGE_RIGHT_ARROW}, // VIRTUAL RIGHT
    {KEY_TYPE_STD,              USB_USAGE_LEFT_ARROW},  // VIRTUAL LEFT
    {KEY_TYPE_STD,              USB_USAGE_DOWN_ARROW},  // VIRTUAL DOWN
    {KEY_TYPE_STD,              USB_USAGE_UP_ARROW},    // VIRTUAL UP
#endif
};

#define KEY_TABLE_SIZE (sizeof(kbKeyConfig)/sizeof(KbKeyConfig))

//////////////////////////////////////////////////////////////////////////////
typedef struct {

    uint8_t                 stdRpt_changed:1;
    uint8_t                 bitMapped_changed:1;

} kbrpt_t;
static kbrpt_t keyRpt;

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////


uint8_t kbrpt_ledStates;

key_input_rpt_t key_rpts = {
    .stdRpt          = {RPT_ID_IN_STD_KEY},
    .bitMappedReport = {RPT_ID_IN_BIT_MAPPED},
};

/*******************************************************************************
 * Function Name: void KeyRpt_stdRptProcEvtKeyDown(uint8_t translationCode)
 ********************************************************************************
 * Summary: add the key to standard key report
 *
 * Parameters:
 *  key -- defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_stdRptProcEvtKeyDown(uint8_t key)
{
    uint8_t i;
    uint8_t * keyCodes = key_rpts.stdRpt.keyCodes;

    // Check if the key is already in the report
    for (i=0; keyCodes[i] && (i < KEY_MAX_KEYS_IN_STD_REPORT); i++)
    {
        if (keyCodes[i] == key)
        {
            // Already in the report. Ignore the event
            return;
        }
    }

    // Check if the std report has room
    if (i < KEY_MAX_KEYS_IN_STD_REPORT)
    {
        // Add the new key to the report
        keyCodes[i] = key;

        // Flag that the standard key report has changed
        keyRpt.stdRpt_changed = TRUE;
    }
}

/*******************************************************************************
 * Function Name: void KeyRpt_stdRptProcEvtKeyUp(uint8_t translationCode)
 ********************************************************************************
 * Summary: remove the code from standard key report
 *
 * Parameters:
 *  key -- defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_stdRptProcEvtKeyUp(uint8_t key)
{
    uint8_t i;
    uint8_t * keyCodes = key_rpts.stdRpt.keyCodes;

    // Find the key in the current standard report
    for (i=0; keyCodes[i] && i < KEY_MAX_KEYS_IN_STD_REPORT; i++)
    {
        if (keyCodes[i] == key)
        {
            // Found it. Remove it by shifting it!
            do
            {
                keyCodes[i] = keyCodes[i+1];
                // over sized? if so, we copied junk
                if (++i == KEY_MAX_KEYS_IN_STD_REPORT)
                {
                    keyCodes[--i] = 0; // replace junk data with 0
                }
            }
            while (keyCodes[i]);
            keyRpt.stdRpt_changed = TRUE;
        }
    }
}

/*******************************************************************************
 * Function Name: wiced_bool_t KeyRpt_updateBit(uint8_t *buf, uint8_t set, uint8_t bitMask)
 ********************************************************************************
 * Summary: Set or Clear the bit identified in bitMask within the byte
 *
 * Parameters:
 *  down -- TRUE when key is down
 *  translationCode -- bit position defined in USB
 *
 * Return:
 *  TRUE -- bit is changed
 *  FALSE -- no change
 *
 *******************************************************************************/
static wiced_bool_t KeyRpt_updateBit(uint8_t *buf, uint8_t set, uint8_t bitMask)
{
    uint8_t bits = *buf;

    if (set)
    {
        *buf |= bitMask;
    }
    else
    {
        *buf &= ~bitMask;
    }
    return bits != *buf;
}

/*******************************************************************************
 * Function Name: void KeyRpt_stdRptProcEvtModKey(uint8_t down, uint8_t translationCode)
 ********************************************************************************
 * Summary: handle modifier keys events
 *
 * Parameters:
 *  down -- TRUE when key is down
 *  translationCode -- bit position defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_stdRptProcEvtModKey(uint8_t set, uint8_t translationCode)
{
    // set or reset the bit
    if (KeyRpt_updateBit(&key_rpts.stdRpt.modifierKeys, set, translationCode))
    {
        // Flag that the standard key report has changed
        keyRpt.stdRpt_changed = TRUE;
    }
}

/*******************************************************************************
 * Function Name: void KeyRpt_bitRptProcEvtKey(uint8_t down, uint8_t bitPos)
 ********************************************************************************
 * Summary: handle modifier keys events
 *
 * Parameters:
 *  set -- TRUE then set the bit, otherwise, clear the bit
 *  translationCode -- bit position defined in USB
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_bitRptProcEvtKey(uint8_t set, uint8_t bitPos)
{
    uint8_t idx = bitPos / 8;
    uint8_t bitMask = (1<< (bitPos % 8));

    // set or reset the bit
    if (KeyRpt_updateBit(&key_rpts.bitMappedReport.bitMappedKeys[idx], set, bitMask))
    {
        // Flag that the standard key report has changed
        keyRpt.bitMapped_changed = TRUE;
    }
}

/*******************************************************************************
 * Function Name: void KeyRpt_procEvtUserDefinedKey(void)
 ********************************************************************************
 * Summary: User defined key event handling
 *          Not used.
 *
 * Parameters:
 *  keyCode -- key index
 *  keyDown -- key up or down
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void KeyRpt_procEvtUserDefinedKey(uint8_t down, uint8_t translationCode)
{
}

/*******************************************************************************
 * Function Name: void key_keyEvent(void)
 ********************************************************************************
 * Summary: key event handling
 *
 * Parameters:
 *  keyCode -- key index
 *  keyDown -- key up or down
 *
 * Return:
 *  TRUE -- handled correctly
 *  FALSE -- error detected
 *
 *******************************************************************************/
wiced_bool_t key_procEvtKey(uint8_t keyCode, uint8_t keyDown)
{
    // Check if we have a valid key
    if (keyCode < KEY_TABLE_SIZE)
    {
        uint8_t keyValue = kbKeyConfig[keyCode].translationValue;

        // Depending on the key type, call the appropriate function for handling
        // Pass unknown key types to user function
        switch(kbKeyConfig[keyCode].type)
        {
            case KEY_TYPE_STD:
                // Processing depends on whether the event is an up or down event
                keyDown ? KeyRpt_stdRptProcEvtKeyDown(keyValue) : KeyRpt_stdRptProcEvtKeyUp(keyValue);
                break;
            case KEY_TYPE_MODIFIER:
                KeyRpt_stdRptProcEvtModKey(keyDown, keyValue);
                break;
            case KEY_TYPE_BIT_MAPPED:
                KeyRpt_bitRptProcEvtKey(keyDown, keyValue);
                break;
            case KEY_TYPE_NONE:
                // do nothing
                break;
            default:
                KeyRpt_procEvtUserDefinedKey(keyDown, keyValue);
                break;
        }
    }
    // Check if we have an end of scan cycle event
    else if (keyCode == END_OF_SCAN_CYCLE)
    {
        key_send();
    }
    else
    {
        // key index is out of range
        return FALSE;
    }
    return TRUE;
}

/*******************************************************************************
 * Function Name: void key_send(void)
 ********************************************************************************
 * Summary: Send any pending key reports.
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_send()
{
    if (keyRpt.stdRpt_changed)
    {
        hidd_link_send_report(&key_rpts.stdRpt, sizeof(KeyboardStandardReport));
        keyRpt.stdRpt_changed = FALSE;
    }
    if (keyRpt.bitMapped_changed)
    {
        hidd_link_send_report(&key_rpts.bitMappedReport, sizeof(KeyboardBitMappedReport));
        keyRpt.bitMapped_changed = FALSE;
    }
}

/*******************************************************************************
 * Function Name: void key_clear(wiced_bool_t sendRpt)
 ********************************************************************************
 * Summary: Clear all reports to default value
 *
 * Parameters:
 *  sendRpt -- When TRUE, after clear, send standard report (to indicate all keys are up)
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_clear(wiced_bool_t sendRpt)
{
    // clear report data
    memset(&key_rpts.stdRpt.modifierKeys, 0, sizeof(KeyboardStandardReport)-1);
    memset(key_rpts.bitMappedReport.bitMappedKeys, 0, sizeof(KeyboardBitMappedReport)-1);

    keyRpt.stdRpt_changed = keyRpt.bitMapped_changed = sendRpt;

    key_send();
}

/*******************************************************************************
 * Function Name: void key_sendRollover()
 ********************************************************************************
 * Summary: Send a rollover packet
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void key_sendRollover()
{
    KeyboardStandardReport  rolloverRpt = {RPT_ID_IN_STD_KEY, 0, 0, {CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER, CODE_ROLLOVER}};
    // Tx rollover report
    WICED_BT_TRACE("\nRollOverRpt");
    hidd_link_send_report(&rolloverRpt, sizeof(KeyboardStandardReport));
}

/*******************************************************************************
 * Function Name: void key_setReport()
 ********************************************************************************
 * Summary: handle HID setReport for keyboard
 *
 * Parameters:
 *  reportType -- report type (defined in wiced_hidd_report_type_e)
 *  reportId   -- Report ID
 *  payload    -- data pointer
 *  payloadSize -- data length
 *
 * Return:
 *  TRUE -- if handled
 *
 *******************************************************************************/
wiced_bool_t key_setReport(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize)
{
    if (reportType == WICED_HID_REPORT_TYPE_OUTPUT)
    {
        // Pass to handler based on report ID. Ensure that report ID is in the payload
        if (payloadSize >= 1)
        {
            // Demux on report ID
            if(reportId == RPT_ID_OUT_KB_LED)
            {
                WICED_BT_TRACE("\nKB LED report");
                kbrpt_ledStates = *((uint8_t*)payload);
                return TRUE;
            }
        }
    }
    return FALSE;
}
#endif // SUPPORT_KEY_REPORT
