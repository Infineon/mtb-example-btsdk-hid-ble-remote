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
*/

#ifdef SUPPORTING_FINDME

#include "gki_target.h"
#if 0
#include "wiced_bt_cfg.h"
#include "ble_remote_gatts.h"
#include "app.h"
#include "wiced_bt_gatt.h"
#include "wiced_hal_mia.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_keyscan.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_sdp.h"
#include "wiced_hal_batmon.h"
#include "wiced_memory.h"
#endif

tAppAlertConfig appAlert_cfg =
{
  // mild alert Buz config
  {
    {WICED_PWMBUZ_FREQ_MANUAL, // WICED_PWMBUZ_4000,
#if 0 //4K
    65473,              // init_value
    65503,             // toggle_val
#else // 4.2K
    65476,              // init_value
    65505,             // toggle_val
#endif
    1000,          // buz_on_ms
    2000,          // buz_off_ms
    10},          //repeat_nun

    // high alert Buz config
    {WICED_PWMBUZ_FREQ_MANUAL, // WICED_PWMBUZ_4000,  // freq
#if 0 //4K
    65473,              // init_value
    65503,             // toggle_val
#else // 4.2K
    65476,              // init_value
    65505,             // toggle_val
#endif
    300,           //buz_on_ms
    300,           // buz_off_ms
    20}         //repeat_nun
  },
    // mild alert Led config
    {{1000,        // led_on_ms
    2000,          // led_off_ms
    10},          // repeat_nun

    // high alert Led config
    {300,          // led_on_ms
    300,           // led_off_ms
    20}},         // repeat_nun
};

tAppFindmeState *appFindmeState;
wiced_timer_t findme_led_timer;
wiced_timer_t findme_buz_timer;

////////////////////////////////////////////////////////////////////////////////
///  This function is the alert BUZ timer timeout handler
////////////////////////////////////////////////////////////////////////////////
void app_alertBuz_timeout(uint32_t unused)
{
    uint8_t patten_id = appFindmeState->buz_pattern_id;

//WICED_BT_TRACE("\nbuz alert");
    // reached to timeout value.expired.
    if (appFindmeState->buz_on) // buz is on state
    {
//WICED_BT_TRACE("\n off");
        app_alertBuzOff(appFindmeState->buz_id);

        appFindmeState->buz_on = 0;
        appFindmeState->buz_repeat--;

        if (appFindmeState->buz_repeat == 0)
        {
            appFindmeState->buz_alert_active = 0; // buz alert is done.
            return;
        }

        if (appAlert_cfg.alertBuzCfg[patten_id].buz_off_ms > 0)
        {
            wiced_start_timer(&findme_buz_timer, appAlert_cfg.alertBuzCfg[patten_id].buz_off_ms); //timeout in ms
        }
    }
    else
    {
//WICED_BT_TRACE("\n on");
        app_alertBuzOn(appFindmeState->buz_id);

        if (appAlert_cfg.alertBuzCfg[patten_id].buz_on_ms > 0)
        {
            wiced_start_timer(&findme_buz_timer, appAlert_cfg.alertBuzCfg[patten_id].buz_on_ms); //timeout in ms
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to set BUZ frequency
////////////////////////////////////////////////////////////////////////////////
void app_alertBuzFreq(uint8_t freq, uint16_t init_value, uint16_t toggle_val)
{
//    wiced_blehidd_pwm_buz_freq(freq, init_value, toggle_val);
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to turn on BUZ
////////////////////////////////////////////////////////////////////////////////
void app_alertBuzOn(uint8_t pwm_id)
{
//    wiced_blehidd_pwm_buz_on(pwm_id);
    appFindmeState->buz_on = 1;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to turn off BUZ
////////////////////////////////////////////////////////////////////////////////
void app_alertBuzOff(uint8_t pwm_id)
{
//    wiced_blehidd_pwm_buz_off(pwm_id);
    appFindmeState->buz_on = 0;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the handling of alert BUZ based on alert level
////////////////////////////////////////////////////////////////////////////////
void app_alertBuzPlay(uint8_t pattern_id)
{
    if ((appAlert_cfg.alertBuzCfg[pattern_id].repeat_num == 0) ||
        (appAlert_cfg.alertBuzCfg[pattern_id].buz_on_ms == 0) ||
        (pattern_id >= APP_ALERT_PATTERN_MAX_ID)) // invalid parameter.
    {
        return;
    }

    app_alertBuzStop();

    appFindmeState->buz_pattern_id = pattern_id;
    appFindmeState->buz_repeat = appAlert_cfg.alertBuzCfg[pattern_id].repeat_num;

    app_alertBuzFreq(appAlert_cfg.alertBuzCfg[pattern_id].freq,
            appAlert_cfg.alertBuzCfg[pattern_id].init_value,
            appAlert_cfg.alertBuzCfg[pattern_id].toggle_val);
    app_alertBuzOn(appFindmeState->buz_id);
    //app_StartAlertBuzTimer(appAlert_cfg.alertBuzCfg[pattern_id].buz_on_ms);
    if (appAlert_cfg.alertBuzCfg[pattern_id].buz_on_ms > 0)
        wiced_start_timer(&findme_buz_timer, appAlert_cfg.alertBuzCfg[pattern_id].buz_on_ms); //timeout in ms
    appFindmeState->buz_alert_active = 1; // buz alert is activated.
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the handling of NO ALERT (BUZ)
////////////////////////////////////////////////////////////////////////////////
void app_alertBuzStop(void)
{
    if (appFindmeState->buz_alert_active)
    {
        wiced_stop_timer(&findme_buz_timer);
        app_alertBuzOff(appFindmeState->buz_id);
        appFindmeState->buz_alert_active = 0; // buz alert is deactivated.
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the alert LED timer timeout handler
////////////////////////////////////////////////////////////////////////////////
void app_alertLed_timeout(uint32_t unused)
{
    uint8_t pattern_id = appFindmeState->led_pattern_id;

//WICED_BT_TRACE("\nled alert");
    if (appFindmeState->led_on) // led is on state
    {
        app_alertLedOff();
        appFindmeState->led_repeat--;

        if (appFindmeState->led_repeat == 0)
        {
            appFindmeState->led_alert_active = 0; // led alert has beed done.
            return;
        }

        wiced_start_timer(&findme_led_timer, appAlert_cfg.alertLedCfg[pattern_id].led_off_ms);
    }
    else
    {
        app_alertLedOn();
        wiced_start_timer(&findme_led_timer, appAlert_cfg.alertLedCfg[pattern_id].led_on_ms);
    }
}


////////////////////////////////////////////////////////////////////////////////
///  This function is to turn on LED
////////////////////////////////////////////////////////////////////////////////
void app_alertLedOn(void)
{
    //configure LED on
//    wiced_hal_gpio_set_pin_output(GPIO_PORT_LED, LED_ON);
    appFindmeState->led_on = 1;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to turn off LED
////////////////////////////////////////////////////////////////////////////////
void app_alertLedOff(void)
{
    //configure LED off
//    wiced_hal_gpio_set_pin_output(GPIO_PORT_LED, LED_OFF);
    appFindmeState->led_on = 0;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to start the LED play based on alert level
////////////////////////////////////////////////////////////////////////////////
void app_alertLedPlay(uint8_t pattern_id)
{
    if ((appAlert_cfg.alertLedCfg[pattern_id].repeat_num == 0) ||
        (appAlert_cfg.alertLedCfg[pattern_id].led_on_ms == 0) ||
        (pattern_id >= APP_ALERT_PATTERN_MAX_ID)) // invalid parameter.
        return;

    app_alertLedStop();

    appFindmeState->led_pattern_id = pattern_id;
    appFindmeState->led_repeat = appAlert_cfg.alertLedCfg[pattern_id].repeat_num;

    app_alertLedOn();

    //app_StartAlertLedTimer(appAlert_cfg.alertLedCfg[pattern_id].led_on_ms);
    wiced_start_timer(&findme_led_timer, appAlert_cfg.alertLedCfg[pattern_id].led_on_ms);

    appFindmeState->led_alert_active = 1; // led alert is activated.

}

////////////////////////////////////////////////////////////////////////////////
///  This function is the handling of NO ALERT (LED)
////////////////////////////////////////////////////////////////////////////////
void app_alertLedStop(void)
{
    if (appFindmeState->led_alert_active)
    {
        wiced_stop_timer(&findme_led_timer);

        //configure LED off
//        wiced_hal_gpio_set_pin_output(GPIO_PORT_LED, LED_OFF);
        appFindmeState->led_alert_active = 0; // led alert is deactivated.
    }
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the callback function for the find me attribute handle.
/// It controls the LED behavior depending on the value of the write command
////////////////////////////////////////////////////////////////////////////////
int findme_writeCb(void *p)
{
    wiced_bt_gatt_write_t *p_data = (wiced_bt_gatt_write_t *)p;

    if ((HANDLE_BLEREMOTE_IMMEDIATE_ALERT_SERVICE_CHAR_LEVEL_VAL == p_data->handle) && (p_data->val_len == 1)) // alert level len is 1byte
    {

        appFindmeState->activeAlterLevel = *(p_data->p_val);

        switch(appFindmeState->activeAlterLevel)
        {
            case NO_ALERT:
                /* Action to Stop Alert */
                /*
                app_alertBuzStop();
                app_alertLedStop();
                */
            break;

            case MILD_ALERT:
                /* Action for Mild Alert */
                /*
                if (appFindmeState->alertType & ALERT_BUZ)
                {
                    app_alertBuzPlay(APP_ALERT_PATTERN_MILD_ID);
                }
                if (appFindmeState->alertType & ALERT_LED)
                {
                    app_alertLedPlay(APP_ALERT_PATTERN_MILD_ID);
                }
                */
            break;

            case HIGH_ALERT:
                /* Action for HIGH Alert */
                /*
                if (appFindmeState->alertType & ALERT_BUZ)
                {
                    app_alertBuzPlay(APP_ALERT_PATTERN_HIGH_ID);
                }
                if (appFindmeState->alertType & ALERT_LED)
                {
                    app_alertLedPlay(APP_ALERT_PATTERN_HIGH_ID);
                }
                */
            break;
        }

    }

    return 0;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is to check if "find me alert" is active.
///   i.e.buz/led active or not.
////////////////////////////////////////////////////////////////////////////////
uint8_t findme_is_active(void)
{
    return (appFindmeState->buz_alert_active | appFindmeState->led_alert_active) ? TRUE : FALSE;
}

////////////////////////////////////////////////////////////////////////////////
///  This function is the FIND ME profile initialization
/// - configure LED for find me alert
/// - register write handle cb for find me attribute handle.
////////////////////////////////////////////////////////////////////////////////
void findme_init(void)
{
    appFindmeState = (tAppFindmeState*)wiced_memory_permanent_allocate(sizeof(tAppFindmeState));
    memset(appFindmeState, 0x00, sizeof(tAppFindmeState));

    appFindmeState->alertType = FINDME_ALERT_TYPE; // ALERT_BUZ; // ALERT_BUZ_LED; // alert both Buz and Led
    if (appFindmeState->alertType & ALERT_BUZ)
    {
        appFindmeState->buz_id = FINDME_BUZ_PWM_ID;
//        wiced_blehidd_pwm_buz_init(GPIO_BUZ_PWM, 0);
    }
    //configure LED
//    wiced_hal_gpio_configure_pin(GPIO_PORT_LED, GPIO_PULL_UP | GPIO_OUTPUT_ENABLE, 1);

    wiced_init_timer( &findme_led_timer, app_alertLed_timeout, 0, WICED_MILLI_SECONDS_TIMER );
    wiced_init_timer( &findme_buz_timer, app_alertBuz_timeout, 0, WICED_MILLI_SECONDS_TIMER );

    wiced_bt_gatt_legattdb_regWriteHandleCb((wiced_bt_gatt_LEGATTDB_WRITE_CB)findme_writeCb);
}


#endif /* SUPPORTING_FINDME */
