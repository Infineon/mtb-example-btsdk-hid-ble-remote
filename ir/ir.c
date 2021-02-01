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
 * This file implements Infrared (IR) Transmit feature
 *
 */
#ifdef SUPPORT_IR

#include "wiced_timer.h"
#include "wiced_hal_mia.h"
#include "wiced_bt_trace.h"
#include "aclk.h"
#include "app.h"

#define IR_CODE 0x3d
#define IR_ADDR 0x55

#define IR_MARK             0x8000    // bit15=1, bit 15 set to one to indicate logic high in data duation
#define IR_SPACE            0x0000    // bit15=0, bit 15 set to zero to indicate logic low in data duation
#define APPIR_MAX_DURATION  0x7FFF    // bit[0:14] = duration

#define APPIRTX_DATA_SIZE   128
#define APP_IR_CYCLE        107900    // 107.900 ms for the entire transaction
#define APP_CARRIER_FREQ    38000     // 38k Hz
#define APP_IR_ONE          1680      // 1.68 ms for one to stay low
#define APP_IR_ZERO         560       // 560 us for zero to stay low
#define APP_IR_SEPARATOR    560       // 560 us for the separator
#define APP_LEAD_HIGH       9000      // Leading high for 9 ms
#define APP_LEAD_LOW        4500      // Leading low for 4.5 ms
#define APP_REPEAT_LOW      2250      // Repeat low for 2.25 ms

#define APP_REPEAT_FOREVER  0xff
#define APP_IR_POR_DELAY    50        // For POR (power on reset), wait 50 ms before hardware is ready to send IR

#define IR_DUTY_CYCLE_33_PERCENT
#ifdef IR_DUTY_CYCLE_33_PERCENT
 #define ACLK ACLK1
 #define EXTENED_SETTING      HW_MIA_IR_CTL_EXTEND_MODULATE_SRC_PWM3    // use PWM3
#else
 #define ACLK ACLK0
 #define EXTENED_SETTING      0
#endif

enum
{
    APPIRTX_IDLE = 0,
    APPIRTX_STARTED,
    APPIRTX_REPEAT,
};

typedef struct
{
    uint32_t total_time_in_us;
    uint16_t data[APPIRTX_DATA_SIZE];
    uint8_t index;
    uint8_t state;
    uint8_t code;
    uint8_t repeat;
    uint8_t pending_code;
    uint8_t pending_repeat;
    uint8_t pending:1;

    wiced_timer_t allow_irtx_timer;
    IR_TX_CLOCK_SETTING txClkSetting;
} ir_data_t;

ir_data_t ir = {0,};

////////////////////////////////////////////////////////////////
///  This function initializes the appIRtx ClkSetting
////////////////////////////////////////////////////////////////
static void irtx_ClkSetting_init(void)
{
    ir.txClkSetting.clockSrc     = ACLK;
    // Need to use 24MHz couck source, 1MHz cannot make 37.9kHz
    ir.txClkSetting.clockSrcFreq = ACLK_FREQ_24_MHZ;

    ir.txClkSetting.extendedSettings = EXTENED_SETTING;
    ir.txClkSetting.pwmDutyCycleHighCount = 1;
    ir.txClkSetting.pwmDutyCycleLowCount = 2;

    ir.txClkSetting.invertOutput = FALSE;
    ir.txClkSetting.modulateFreq = APP_CARRIER_FREQ;
}

/////////////////////////////////////////////////////////////////////////////////
// Assign logic MARK or SPACE to IR database one element
//
// parameter mark: true for MARK. false for SPACE
//
//           duration : in micro-seconds
//
// This function changes ir.index and ir.total_time_in_us
/////////////////////////////////////////////////////////////////////////////////
static void appirtx_data(wiced_bool_t mark, uint32_t duration)
{
    while (duration)
    {
        uint16_t data = duration > APPIR_MAX_DURATION ? APPIR_MAX_DURATION : duration;
        ir.data[ir.index++] = (mark ? IR_MARK : IR_SPACE) | data;
        ir.total_time_in_us += data;
        duration -= data;
    }
}

/////////////////////////////////////////////////////////////////////////////////
//
/////////////////////////////////////////////////////////////////////////////////
static void appirtx_writebyte(uint8_t code)
{
    uint8_t bit=1;

    while (bit)
    {
        appirtx_data(TRUE, APP_IR_SEPARATOR); // separator .56ms
        appirtx_data(FALSE, bit & code ? APP_IR_ONE : APP_IR_ZERO); // Write one bit of data
        bit <<=1;
    }
}

/////////////////////////////////////////////////////////////////////////////////
// construct initial IR frame database
/////////////////////////////////////////////////////////////////////////////////
static void appirtx_initial_frame()
{
    // clear database
    ir.index = 0;
    ir.total_time_in_us = 0;

    appirtx_data(TRUE, APP_LEAD_HIGH);     // Leading high for 9 ms
    appirtx_data(FALSE, APP_LEAD_LOW);     // Leading low for 4.5 ms
    appirtx_writebyte(IR_ADDR);
    appirtx_writebyte(~IR_ADDR);
    appirtx_writebyte(ir.code);
    appirtx_writebyte(~ir.code);
    appirtx_data(TRUE, APP_IR_SEPARATOR);  // separator .56ms
    appirtx_data(FALSE, APP_IR_CYCLE - ir.total_time_in_us);       // stays low until frame (107.9ms) ends
}

/////////////////////////////////////////////////////////////////////////////////
// construct repeat IR frame database
/////////////////////////////////////////////////////////////////////////////////
static void appirtx_repeat_frame()
{
    // clear database
    ir.index = 0;
    ir.total_time_in_us = 0;

    appirtx_data(TRUE, APP_LEAD_HIGH);       // Leading high for 9 ms
    appirtx_data(FALSE, APP_REPEAT_LOW);     // Leading low for 2.25 ms
    appirtx_data(TRUE, APP_IR_SEPARATOR);    // separator .56ms
    appirtx_data(FALSE, APP_IR_CYCLE - ir.total_time_in_us);       // stays low until frame (107.9ms) ends
}

////////////////////////////////////////////////////////////////////////////////
/// This function is the timeout handler for allow_irtx_timer
////////////////////////////////////////////////////////////////////////////////
static void appirtx_check_pending( uint32_t arg)
{
    if (ir.pending)
    {
        WICED_BT_TRACE("\nstart pending IR");
        ir.pending = FALSE;
        //send IR
        ir_start(ir.pending_code, ir.pending_repeat);
    }
}

/////////////////////////////////////////////////////////////////////////////////
// IR state handler
/////////////////////////////////////////////////////////////////////////////////
static void appirtx_irtx_handler()
{
    switch (ir.state)
    {
    case APPIRTX_STARTED:
        appirtx_initial_frame();
        wiced_irtx_send_data(ir.data, ir.index, ir.txClkSetting);
        ir.state = APPIRTX_REPEAT;
        break;

    case APPIRTX_REPEAT:
        if (ir.repeat)
        {
            appirtx_repeat_frame();
            wiced_irtx_send_data(ir.data, ir.index, ir.txClkSetting);
            if (ir.repeat != APP_REPEAT_FOREVER)
            {
                ir.repeat--;
            }
        }
        else
        {
            ir.state = APPIRTX_IDLE;
        }
        break;

    default:
        break;
    }

    if(ir.state == APPIRTX_IDLE)
    {
        // check if any pending IR code to be sent
        appirtx_check_pending(0);
    }
}

////////////////////////////////////////////////////////////////
///  call back function when IR frame transmit is completed
////////////////////////////////////////////////////////////////
static BOOL8 appirtx_irtx_complete(IR_TX_STATE state)
{
    WICED_BT_TRACE("\nIR Done");
    appirtx_irtx_handler();
    return FALSE; // let handler to continue (not used for irtx_complete)
}

///////////////////////////////////////////////////////////////////////////
///
///////////////////////////////////////////////////////////////////////////
void ir_start(uint8_t code, uint8_t repeat_count)
{
    // if busy, we save it to pending
    if( wiced_is_timer_in_use(&ir.allow_irtx_timer) || ir.state)
    {
        ir.pending = TRUE;
        ir.pending_code = code;
        ir.pending_repeat = repeat_count;
        return;
    }

    ir.repeat = repeat_count;
    ir.code = code;
    ir.state = APPIRTX_STARTED;
    appirtx_irtx_handler();
}

////////////////////////////////////////////////////////////////
///  This function stops repeating frames.
////////////////////////////////////////////////////////////////
void ir_stop(void)
{
    ir.repeat = 0;
}

////////////////////////////////////////////////////////////////
/// ir_init
///
/// needs to be called once at start up to setup IR gpio and IR initialization.
////////////////////////////////////////////////////////////////
void ir_init(BYTE gpio)
{
    WICED_BT_TRACE("\nIR init %d", gpio);

    wiced_irtx_hook_callback(NULL, NULL, appirtx_irtx_complete);
    wiced_irtx_init(gpio);

    // If this is a power on reset, we need to wait for hardware to get ready before we can send IR
    wiced_init_timer( &ir.allow_irtx_timer, appirtx_check_pending, 0, WICED_MILLI_SECONDS_TIMER );
    if (!wiced_hal_mia_is_reset_reason_por())
    {
        wiced_start_timer(&ir.allow_irtx_timer, APP_IR_POR_DELAY);
    }

    irtx_ClkSetting_init();
    ir.state = APPIRTX_IDLE;
}

////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////
wiced_bool_t ir_button(uint8_t key, wiced_bool_t down)
{
    if (key == IR_KEY_INDEX)
    {
        WICED_BT_TRACE("\nIR button=%s", down?"DN":"UP");
        if (down)
        {
            ir_start(IR_CODE, 2);
        }
        else
        {
            // ir_stop();
        }
        return TRUE;
    }
    return FALSE;
}


#endif // SUPPORT_IR
