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
 * Keyscan Interface
 *
 */

#ifdef SUPPORT_KEYSCAN

#include "app.h"
#include "wiced_hal_mia.h"

typedef struct {
    keyPressDetected_callback_t * appCb;
} kscan_data_t;

static kscan_data_t ks = {};

/////////////////////////////////////////////////////////////////////////////////
/// This function polls for key activity and queues any key events in the
/// FW event queue. Events from the keyscan driver are processed until the driver
/// runs out of events.
/////////////////////////////////////////////////////////////////////////////////
static void kscan_pollEvent(void * userData)
{
    HidEventKey event = {{HID_EVENT_KEY_STATE_CHANGE}};

    while (wiced_hal_keyscan_get_next_event(&event.keyEvent))
    {
        if (ks.appCb)
        {
#if !defined(REMOTE_PLATFORM) && !is_20735Family
            // EVB-02 cannot produce full set of key matrix, for
            // connect & audio buttons, we need to be translated to actual key index
            switch (event.keyEvent.keyCode) {
            case FAKE_CONNECT_KEY_INDEX:
                event.keyEvent.keyCode = CONNECT_KEY_INDEX;
                break;
            case FAKE_AUDIO_KEY_INDEX:
                event.keyEvent.keyCode = AUDIO_KEY_INDEX;
                break;
            }
#endif
            (ks.appCb)(&event);
        }
    }
}

/*******************************************************************************
 * Function Name: void kscan_pollActivity(void)
 ********************************************************************************
 * Summary: Initialize keyscan configuration,
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void kscan_pollActivity(void)
{
    kscan_pollEvent(NULL);
}

/*******************************************************************************
 * Function Name: void kscan_reset(void)
 ********************************************************************************
 * Summary: Resets keyscan and getting ready key events
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void kscan_reset(void)
{
    // Reset the keyscan HW
    wiced_hal_keyscan_reset();

    // Configure GPIOs for keyscan operation
    wiced_hal_keyscan_config_gpios();
}

/*******************************************************************************
 * Function Name: void kscan_init
 ********************************************************************************
 * Summary: Initialize keyscan configuration,
 *
 * Parameters:
 *  row, col -- key matrix row & col dimention
 *  poll_callback_t * pcb -- application poll function pointer to poll user activities
 *  keyPressDetected_callback_t * cb -- application callback function pointer to handle key event
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void kscan_init(uint8_t row, uint8_t col, app_poll_callback_t * pcb, keyPressDetected_callback_t * cb)
{
    //save applicatino callback function pointer
    ks.appCb = cb;

    WICED_BT_TRACE("\nkeyscan %dx%d", row, col);
    //keyscan initialize
    wiced_hal_keyscan_configure(row, col);
    wiced_hal_keyscan_init();
    wiced_hal_keyscan_register_for_event_notification((kscan_poll_callback_t *)pcb, NULL);
}

#endif // SUPPORT_KEYSCAN
