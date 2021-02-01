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
 * This file defines the interface of battery report service
 *
 */

#ifndef __APP_BATTERY_H__
#define __APP_BATTERY_H__

#ifdef BATTERY_REPORT_SUPPORT
#include "wiced.h"
#include "wiced_hal_batmon.h"

#define BATTERY_RPT_SIZE 1

/// Battery key report structure
typedef PACKED struct
{
    /// Set to the value specified in the config record.
    uint8_t    reportID;

    uint8_t    level[BATTERY_RPT_SIZE];
}BatteryReport;

extern BatteryReport batRpt;

/*******************************************************************************
 * Function Name: void bat_poll
 ********************************************************************************
 * Summary: poll for battery level
 *
 * Parameters:
 *  none
 *
 * Return:
 *  none
 *
 *******************************************************************************/
#define bat_poll() wiced_hal_batmon_poll_monitor();

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
void bat_init(void (shutdown_cb)());

#else
# define bat_init(c)
# define bat_poll()
#endif
#endif // __APP_BATTERY_H__
