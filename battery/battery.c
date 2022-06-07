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
 * This file defines the interface of battery report service
 *
 */

#ifdef BATTERY_REPORT_SUPPORT
#include "app.h"

BatteryReport batRpt={RPT_ID_IN_BATTERY,{100}};

/********************************************************************************
 * Function Name: Bat_batLevelChangeNotification
 ********************************************************************************
 * Summary: send out battery report when battery level changed
 *
 * Parameters:
 *  newLevel -- new battery level
 *
 * Return:
 *  None
 *
 *******************************************************************************/
static void Bat_batLevelChangeNotification(uint32_t newLevel)
{
    if (batRpt.level[0] != newLevel)
    {
        WICED_BT_TRACE("\nbat level changed to %d", newLevel);
        batRpt.level[0] = newLevel;
        hidd_link_send_report(&batRpt, sizeof(BatteryReport));
        wiced_hal_batmon_set_battery_report_sent_flag(WICED_TRUE);
    }
}

/*******************************************************************************
 * Function Name: void bat_init
 ********************************************************************************
 * Summary: initialize battery report
 *
 * Parameters:
 *  void (shutdown_cb)()  -- shutdown callback function
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void bat_init(void (shutdown_cb)())
{
    //battery monitoring configuraion
    wiced_hal_batmon_config(ADC_INPUT_VDDIO,      // ADC input pin
                            3000,               // Period in millisecs between battery measurements
                            8,                  // Number of measurements averaged for a report, max 16
                            3200,               // The full battery voltage in mili-volts
                            1800,               // The voltage at which the batteries are considered drained (in milli-volts)
                            1700,               // System should shutdown if it detects battery voltage at or below this value (in milli-volts)
                            100,                // battery report max level
                            RPT_ID_IN_BATTERY,  // battery report ID
                            1,                  // battery report length
                            1);                 // Flag indicating that a battery report should be sent when a connection is established

     //register App low battery shut down handler
    wiced_hal_batmon_register_low_battery_shutdown_cb(shutdown_cb);
    wiced_hal_batmon_add_battery_observer(Bat_batLevelChangeNotification);

    //Setup Battery Service
    wiced_hal_batmon_init();
}

#endif // BATTERY_REPORT_SUPPORT
