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
 * BLE Remote find me
 *
 * This file provides definitions and function prototypes for BLE remote find me functions
 *
 */

#ifdef SUPPORTING_FINDME
#include "pwm.h"
#define FINDME_ALERT_TYPE        ALERT_BUZ_LED  // ALERT_LED, ALERT_BUZ

//#define LED_ON  0
//#define LED_OFF 1
//#define FINDME_LED WICED_PLATFORM_LED_1     // use first LED for findme LED
//#define GPIO_PORT_LED (*platform_led[FINDME_LED].gpio)

// findMe BUZ alert config
typedef struct
{
    uint8_t freq;         // pwm freq
    uint16_t init_value;  // pwm init value
    uint16_t toggle_val;  // pwm toggle value
    uint16_t buz_on_ms;   // buz on duration
    uint16_t buz_off_ms;  // buz on duration
    uint16_t repeat_num;  // repeat num
}AppBuzAlertConfig;

// findMe LED alert config
typedef struct
{
    uint16_t led_on_ms;    // led on duration
    uint16_t led_off_ms;   // led on duration
    uint16_t repeat_num;   // repeat num
}AppLedAlertConfig;


//Find me Alert mode
enum ble_findme_alert_level
{
    NO_ALERT                        = 0,
    MILD_ALERT                      = 1,
    HIGH_ALERT                      = 2,
    UNDIRECTED_DISCOVERABLE_ALERT   = 3,
};

// Find me Alert type
#define ALERT_NONE    0x00
#define ALERT_BUZ     0x01
#define ALERT_LED     0x02

#define ALERT_BUZ_LED (ALERT_BUZ | ALERT_LED)
// valid id's are 0 thru 5, corresponding to P26 thru p31
typedef enum {
    BUZ_ID0 = PWM0,
    BUZ_ID1 = PWM1,
    BUZ_ID2 = PWM2,
    BUZ_ID3 = PWM3,
    BUZ_ID4 = PWM4,
    BUZ_ID5 = PWM5,
} tBuzId;
#define FINDME_BUZ_PWM_ID        BUZ_ID2
// app Alert ID
enum
{
    APP_ALERT_PATTERN_MILD_ID     = 0x00,
    APP_ALERT_PATTERN_HIGH_ID     = 0x01,
    APP_ALERT_PATTERN_MAX_ID      = 0x02,
};

typedef struct
{
    uint8_t activeAlterLevel;   // active immediate alert level.
    uint8_t alertType;          // alert type(LED or BUZ or both)

    // buz alert state
    tBuzId buz_id;             // id for buz
    uint8_t buz_alert_active;   // buz alert is on playing
    uint8_t buz_on;             // app buz on state
    uint8_t buz_pattern_id;     // active buz pattern ID (mild or High)
    uint16_t buz_repeat;        // app buz repeat num
    uint16_t buz_timeout_sec;   // buz timeout in sec part
    uint16_t buz_timeout_ms;    // buz timeout in ms part
    uint16_t buz_timer_call_per_sec; // buz tick divider for ms timer

    // led alert state
    uint8_t led_alert_active;   // app led alert  is on playing
    uint8_t led_on;             // app led on state
    uint8_t led_pattern_id;     // active pattern ID (mild or High)
    uint16_t led_repeat;        // app led repeat num
    uint16_t led_timeout_sec;   // led timeout in sec part
    uint16_t led_timeout_ms;    // led timeout in ms part
    uint16_t led_timer_call_per_sec; // led tick divider for ms timer
}tAppFindmeState;

typedef struct
{
    // Alert Buz config
    AppBuzAlertConfig alertBuzCfg[APP_ALERT_PATTERN_MAX_ID];

    // Alert Led config
    AppLedAlertConfig alertLedCfg[APP_ALERT_PATTERN_MAX_ID];
}tAppAlertConfig;

void findme_init(void);
uint8_t findme_is_active(void);

#else
 #define findme_init()
 #define findme_is_active() FALSE
#endif // SUPPORTING_FINDME
