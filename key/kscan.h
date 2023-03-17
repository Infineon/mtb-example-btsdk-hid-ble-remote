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
 * Keyscan Interface definitions
 *
 */
#ifndef __KEYSCAN_H__
#define __KEYSCAN_H__

#ifdef SUPPORT_KEYSCAN
#include "wiced.h"
#include "wiced_hal_keyscan.h"
#include "hidevent.h"

typedef void (keyPressDetected_callback_t) (HidEventKey* event);
typedef void (kscan_poll_callback_t) (void *);

/*******************************************************************************
 * Function Name: void kscan_init(uint8_t row, uint8_t col, keyPressDetected_callback_t * cb)
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
void kscan_init(uint8_t row, uint8_t col, hidd_link_app_poll_callback_t * pcb, keyPressDetected_callback_t * cb);

/*******************************************************************************
 * Function Name: void kscan_pollActivity(void)
 ********************************************************************************
 * Summary: poll all key events and callback user function for key handling
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void kscan_pollActivity(void);

/*******************************************************************************
 * Function Name: void kscan_shutdown(void)
 ********************************************************************************
 * Summary: disable keyscan and getting ready for shutdown
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
#define kscan_shutdown() wiced_hal_keyscan_turnOff()

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
void kscan_reset(void);

/*******************************************************************************
 * Function Name: void kscan_enable_ghost_detection(void)
 ********************************************************************************
 * Summary: Resets keyscan and getting ready key events
 *
 * Parameters:
 *  wiced_bool_t enable : TRUE enable ghost key detection
 *                      : FALSE disable ghost key dection
 *
 * Return:
 *  None
 *
 *******************************************************************************/
#define kscan_enable_ghost_detection(e) wiced_hal_keyscan_enable_ghost_detection(e)

/*******************************************************************************
 * Function Name: void kscan_is_any_key_pressed(void)
 ********************************************************************************
 * Summary: Return TRUE if any key is pressed down
 *
 * Parameters:
 *  None
 *
 * Return:
 *  TRUE - any key is pressed down
 *  FALSE - keyscan is idle
 *
 *******************************************************************************/
#define kscan_is_any_key_pressed() wiced_hal_keyscan_is_any_key_pressed()

#else
 #define kscan_init(r,c,pcb,cb)
 #define kscan_pollActivity()
 #define kscan_shutdown()
 #define kscan_reset()
 #define kscan_enable_ghost_detection(e)
 #define kscan_is_any_key_pressed() FALSE
#endif // SUPPORT_KEYSCAN
#endif // __KEYSCAN_H__
