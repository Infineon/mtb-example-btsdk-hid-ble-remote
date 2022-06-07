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
 * TouchPad implementation
 *
 */
#include "app.h"
#ifdef SUPPORT_TOUCHPAD

#include "wiced_hal_mia.h"

#define MAX_TP_EVENT_DELAY_IN_MS    100
TouchPadIf_t  * touchpad   = NULL;
uint8_t  app_touchpad_rpt[TOUCHPAD_RPT_PAYLOAD_SIZE] = {0, };
#define touchpad_is_active()(touchpad && touchpad->isActive())


static Area_t zone[ZONE_MAX_COUNT] = {
     {{0x0060, 0x00d0}, {0x0120, 0x01a0}},    // Zone CENTER
     {{0x0040, 0x0000}, {0x0120, 0x00b0}},    // Zone RIGHT
     {{0x0040, 0x01a0}, {0x0120, 0x01f0}},    // Zone LEFT
     {{0x0000, 0x0080}, {0x0050, 0x0180}},    // Zone DOWN
     {{0x0130, 0x0080}, {0x01f0, 0x0180}},    // zone UP
};

void touchpad_initialize();
void touchpad_reInitialize();
void touchpad_clearEvent();
void touchpad_setEnable(uint8_t en);
uint8_t touchpad_getInfo();
uint8_t touchpad_isActive();
uint8_t touchpad_isFunctional();
uint8_t touchpad_getPinActive();
uint8_t touchpad_fingerCount();
uint8_t touchpad_waitForEvent(uint32_t waitInMs);
uint8_t touchpad_getLeftRight();
uint8_t touchpad_wakeUp();
uint8_t  touchpad_readFirmwareVersion(uint8_t * buff);
uint8_t  touchpad_proximityRpt();
AbsXYRptPtr touchpad_getAbsFingerUpRpt();
uint8_t touchpad_pollActivity(HidEventTouchpad* dataPtr, uint8_t ignore);
static TouchPadIf_t   theTouchPadIface = {
    touchpad_initialize,
    touchpad_reInitialize,
    touchpad_clearEvent,
    touchpad_setEnable,
    touchpad_getInfo,
    touchpad_isActive,
    touchpad_isFunctional,
    touchpad_getPinActive,
    touchpad_fingerCount,
    touchpad_waitForEvent,
    touchpad_getLeftRight,
    touchpad_wakeUp,
    touchpad_readFirmwareVersion,
    touchpad_proximityRpt,
    touchpad_getAbsFingerUpRpt,

    touchpad_pollActivity,
};
static TouchPadIf_t * tpIf = &theTouchPadIface;

static TPDrv_t *    tpDrv;
static Intr_State   tpIntr;
static touchpad_t   theTouchPad;
static touchpad_t * pDev = &theTouchPad;

#ifdef HANDLE_STUCK_FINGER
static void Touchpad_stuckFingerTimerCb(uint32_t unused);
#endif

////////////////////////////////////////////////////////////////////////////////
/// Create touchpad object
/// \param callback: interrupt handler callback
/// \param cookie: interrupt handler callback cookie.
/// \return pointer to TP interface object .
////////////////////////////////////////////////////////////////////////////////
TouchPadIf_t * TouchPad(void (*callBack)(void*, uint8_t), void * cookie)
{
    Intr_CIntr(&tpIntr, callBack, cookie, TP_INTERRUPT_GPIO, INTR_LVL_HIGH, GPIO_EN_INT_RISING_EDGE);
    tpDrv = IQS5xx(TP_INTERRUPT_GPIO);

    TP_RSTN_ReleaseReset();

    // clear all data buffer
    pDev->fifoIndex = 0;
    pDev->verifiedWorking = FALSE;
    pDev->tpSuspend = FALSE;
    memset(pDev->tpData, 0, sizeof(AbsXYReport) * TOUCHPAD_FIFO_CNT);
#ifdef HANDLE_STUCK_FINGER
    wiced_init_timer(&pDev->stuckFingerTimer, Touchpad_stuckFingerTimerCb, 0, WICED_MILLI_SECONDS_TIMER );
    pDev->fingerStuckTimerOn = FALSE;
    pDev->tpActive = FALSE;
#endif
    return tpIf;
}

#ifdef HANDLE_STUCK_FINGER
/////////////////////////////////////////////////////////////////////////////////
/// Stuck finger timer callback
/// \param unused:
/// \return None
////////////////////////////////////////////////////////////////////////////////
static void Touchpad_stuckFingerTimerCb(uint32_t unused)
{
    // Somehow the Touchpad does not interrupt us when finger is lifted,
    // causing isActive() to be TRUE and hence preventing sleep and SDS.
    // Maybe TP FW is doing stuck-finger incorrectly.
    // For now we do our own stuck finger handling
    if(pDev->tpActive)
    {
        // was still active, wait one more round.
        wiced_start_timer(&pDev->stuckFingerTimer, STUCK_FINGER_TIMEOUT_mS);
    }
    else
    {
        // stuck finger
        tpDrv->clearData();
        pDev->fingerStuckTimerOn = FALSE;
    }
    pDev->tpActive = FALSE;
}
#endif

///////////////////////////////////////////////////////////////////////////////
// returns first finger at left or right
///////////////////////////////////////////////////////////////////////////////
static uint8_t touchpad_waitForFingerDown()
{
    HidEventTouchpad event;
    uint8_t ver[2];

     // The x/y report is in the 2nd report followed by state change
    uint8_t retry = 4;
    // if we don't have touchpad finger status yet, we wait for touchpad event
    while (retry-- && !tpDrv->fingerCount())
    {
        if (touchpad_waitForEvent(MAX_TP_EVENT_DELAY_IN_MS))
        {
            touchpad_pollActivity(&event,FALSE);
        }
    }

    // finger is down, it is good
    if (tpDrv->fingerCount())
    {
        return TRUE;
    }

    // no finger is down, something is wrong, reset the TP
    if (!pDev->verifiedWorking && !tpDrv->readFirmwareVersion(ver))   // touchpad is not functional
    {
//WICED_BT_TRACE("\ntp re-init");
        touchpad_reInitialize();
    }
    return FALSE;
}

///////////////////////////////////////////////////////////////////////////////
// returns first finger at left or right
//
///////////////////////////////////////////////////////////////////////////////
uint8_t touchpad_getLeftRight()
{
    if (touchpad_waitForFingerDown()) // Has TP finger data
    {
        uint16_t tpPos;

        // distinguich touch position from Y pos[0]
        tpPos = (tpDrv->getRpt())->xyData.fingers[0].Ypos_H << 8;
        tpPos |= (tpDrv->getRpt())->xyData.fingers[0].Ypos_L;

        return tpPos < CLICK_THRESHOLD ? CLICK_RIGHT : CLICK_LEFT;
    }
    return CLICK_NONE;
}

#if USE_TOUCHPAD_VIRTUAL_KEY
/////////////////////////////////////////////////////////////////////////////////
/// Check whether a finger is in a specified zone
/// \param zone: the zone to check
/// \param pos: the finger position
/// \return TRUE if in zone
////////////////////////////////////////////////////////////////////////////////
static uint8_t inZone(AreaPtr zone, PosPtr pos)
{
    return (pos->x >= zone->p1.x) && (pos->x <= zone->p2.x) &&
           (pos->y >= zone->p1.y) && (pos->y <= zone->p2.y);
}

///////////////////////////////////////////////////////////////////////////////
// returns first finger zone
///////////////////////////////////////////////////////////////////////////////
static uint8_t getTouchpadZone()
{
    Pos_t pos;
    int zoneIndex;
    const XY_Data_finger_t * fingers;
    if (tpDrv->fingerCount() != 1)
    {
        return ZONE_UNDEFINED;
    }

    // We know one finger is down. Determine which finger is down
    fingers = (tpDrv->getRpt())->xyData.fingers;
    pos.x = fingers[0].Xpos_H << 8 | fingers[0].Xpos_L;
    pos.y = fingers[0].Ypos_H << 8 | fingers[0].Ypos_L;

    for (zoneIndex = 0; zoneIndex < ZONE_MAX_COUNT; zoneIndex++)
    {
        if (inZone(&zone[zoneIndex], &pos))
        {
            break;
        }
    }
    return zoneIndex;
}

///////////////////////////////////////////////////////////////////////////////
// returns first finger zone
///////////////////////////////////////////////////////////////////////////////
static uint8_t getZone()
{
    return touchpad_waitForFingerDown() ? getTouchpadZone() : ZONE_UNDEFINED;
}

#endif

///////////////////////////////////////////////////////////////////////////////
// wake up TP
///////////////////////////////////////////////////////////////////////////////
uint8_t touchpad_wakeUp()
{
    return tpDrv->wakeUp();
}

///////////////////////////////////////////////////////////////////////////////
// After initialization, the touchpad interrupt is disabled
///////////////////////////////////////////////////////////////////////////////
void touchpad_initialize()
{
    intrVtblPtr->setInterruptEnable(&tpIntr, FALSE);
    touchpad_clearEvent();
    if (tpDrv->init())
    {
        intrVtblPtr->setInterruptEnable(&tpIntr, TRUE);
    }
    else
    {
        TP_RSTN_HoldReset();   // TP is not in working condition, holding it reset.
    }

    intrVtblPtr->clearInterrupt(&tpIntr);
}

///////////////////////////////////////////////////////////////////////////////
// Re-init TP
///////////////////////////////////////////////////////////////////////////////
void touchpad_reInitialize()
{
    tpDrv->hwReset();
    touchpad_initialize();
}

///////////////////////////////////////////////////////////////////////////////
// shutdown TP
///////////////////////////////////////////////////////////////////////////////
void touchpad_shutdown()
{
    // turn off interrupt
    intrVtblPtr->setInterruptEnable(&tpIntr, FALSE);
    touchpad_clearEvent();
#if SHUTDOWN_TP_BY_RSTN
    TP_RSTN_HoldReset();    // simply hold TP in reset condition
#else
    tpDrv->shutdown();
#endif
}

///////////////////////////////////////////////////////////////////////////////
// clear interrupt and the TP data
///////////////////////////////////////////////////////////////////////////////
void touchpad_clearEvent()
{
    intrVtblPtr->clearInterrupt(&tpIntr);   // clear interrupt controller int pending
    tpDrv->clearData();
}

///////////////////////////////////////////////////////////////////////////////
// Check if TP is suspended, re-init if yes.
///////////////////////////////////////////////////////////////////////////////
void touchpad_checkForSuspend()
{
    // Touchpad could be temporary suspended if touchpad is requesting for initialization
    // This function gets call when there is no high current option. (Both Audio and IR is not active)
    // if it is suspended, we re-initialize the touchpad
    if (pDev->tpSuspend)
    {
        pDev->tpSuspend = FALSE;
        touchpad_reInitialize();
    }
}

///////////////////////////////////////////////////////////////////////////////
//  Poll TP activity
/// \param dataPtr: The TP report data ptr to be filled if applicable
/// \param  ignore: Whether to ignore the retrieved data
/// \return event code indicating data availability
///////////////////////////////////////////////////////////////////////////////
uint8_t touchpad_pollActivity(HidEventTouchpad* dataPtr, uint8_t ignore)
{
    while (intrVtblPtr->isInterruptPinActive(&tpIntr))
    {
        AbsXYReport * rpt;
        uint8_t newFingerCnt;
        uint8_t oldFingerCnt;
        uint8_t goodRead;

        oldFingerCnt = tpDrv->fingerCount();
        goodRead = tpDrv->readXYData();
        intrVtblPtr->clearInterrupt(&tpIntr);
        rpt = tpDrv->getRpt();

        if (!goodRead || (rpt->xyData.info & SHOW_RESET) || pDev->tpSuspend)
        {
            // something is wrong, reset the touchpad
            if (!goodRead)
            {
                touchpad_reInitialize();
            }
            else if (!pDev->tpSuspend) // reset request from TP
            {
                // During high current usage while IR or audio is active, the current could be unstable.
                // We temporary skip initiazation and suspend the touchpad until the power is stable.
                pDev->tpSuspend = TRUE;
                TP_RSTN_HoldReset();
            }
            return HID_EVENT_NONE;
        }
    pDev->verifiedWorking = TRUE;

        if (ignore)
        {
            rpt->xyData.info = 0;
            tpDrv->clearFingerData(&rpt->xyData.fingers[0]);
            tpDrv->clearFingerData(&rpt->xyData.fingers[1]);
        }
        newFingerCnt = tpDrv->fingerCount();
#if 0 // This is used to wake up our chip by TP chip to do battery monitor every, say, 24 hrs.
      // To use BATTERY_MONITOR_INTERRUPT as returend fingerCount, special TP fw is needed.
      // when we wake up from HIDOFF, we will check battery (get 8 sample reads) at startup, if the voltage is too low, it gets shutdown right away and will never reach here.
        if (newFingerCnt == BATTERY_MONITOR_INTERRUPT)
        {
            rpt->xyData.info = oldFingerCnt; // restore original finger count.
            return HID_EVENT_NONE; // not a valid X/Y data
        }
#endif

        // ignore repeated finger up
        if (!(oldFingerCnt || newFingerCnt))
        {
            dataPtr->eventInfo.eventType = HID_EVENT_TP_INFO;
            return HID_EVENT_TP_INFO;
        }

        // need to send report
        rpt->count++;
        memcpy(&pDev->tpData[pDev->fifoIndex], rpt, sizeof(AbsXYReport));
        //dataPtr->size = sizeof(AbsXYReport);
        dataPtr->eventInfo.eventType = (!oldFingerCnt || !newFingerCnt) ? HID_EVENT_TP_FINGER_STATUS_CHANGE : HID_EVENT_TP_DATA;
        //dataPtr->ptr = &pDev->tpData[pDev->fifoIndex];
        dataPtr->userDataPtr = &pDev->tpData[pDev->fifoIndex];

        //WICED_BT_TRACE("\nfc=%d", newFingerCnt);
        if (++pDev->fifoIndex >= TOUCHPAD_FIFO_CNT)
        {
            pDev->fifoIndex = 0;
        }
#ifdef HANDLE_STUCK_FINGER
        if (pDev->fingerStuckTimerOn)
        {
            pDev->tpActive = TRUE;
        }

        if (newFingerCnt && !pDev->fingerStuckTimerOn)
        {
            wiced_start_timer(&pDev->stuckFingerTimer, STUCK_FINGER_TIMEOUT_mS);
            pDev->fingerStuckTimerOn = TRUE;
        }
#endif
        return HID_EVENT_AVAILABLE;
    }

    // This is a work around to prevent TP is in not working state after waking up from HIDOFF, in this case, we need to re-initialze the device
    if (!pDev->verifiedWorking)
    {
        // check if we can read version. If not, then re-initialize the touchpad
        uint8_t ver[2];
        if (tpDrv->readFirmwareVersion(ver))
        {
            pDev->verifiedWorking = TRUE;
        }
        else
        {
            touchpad_reInitialize();
        }
    }
    return HID_EVENT_NONE;
}

///////////////////////////////////////////////////////////////////////////////
//  Read TP FW version
/// \param buff: The buff to retain fw version data, must be at least 2 bytes.
/// \return Whether read is successful
///////////////////////////////////////////////////////////////////////////////
uint8_t  touchpad_readFirmwareVersion(uint8_t * buff)
{
    return tpDrv->readFirmwareVersion(buff);
}

///////////////////////////////////////////////////////////////////////////////
//  Check whether data available is due to proximity
///////////////////////////////////////////////////////////////////////////////
uint8_t  touchpad_proximityRpt()
{
    return tpDrv->proximityRpt();
}

///////////////////////////////////////////////////////////////////////////////
//  Retrieve absolute XY report data
///////////////////////////////////////////////////////////////////////////////
AbsXYRptPtr touchpad_getAbsFingerUpRpt()
{
    AbsXYRptPtr rpt;
    if (tpDrv->fingerCount())         // if the finger status was down
    {
        rpt = tpDrv->getRpt();
        rpt->xyData.info = 0;   // finger up
        return rpt;
    }
    return NULL;
}

///////////////////////////////////////////////////////////////////////////////
//  Set TP enable status
///////////////////////////////////////////////////////////////////////////////
void touchpad_setEnable(uint8_t en)
{
    intrVtblPtr->setInterruptEnable(&tpIntr, en);
}

///////////////////////////////////////////////////////////////////////////////
//  Get TP data info field
///////////////////////////////////////////////////////////////////////////////
uint8_t touchpad_getInfo()
{
    return tpDrv->getInfo();
}

///////////////////////////////////////////////////////////////////////////////
//  Check if TP is active
///////////////////////////////////////////////////////////////////////////////
uint8_t touchpad_isActive()
{
    return tpDrv->fingerCount();       // considered active when any finger is down
}

///////////////////////////////////////////////////////////////////////////////
//  Check if TP is functional
///////////////////////////////////////////////////////////////////////////////
uint8_t touchpad_isFunctional()
{
    return pDev->verifiedWorking;
}

///////////////////////////////////////////////////////////////////////////////
//  Check finger count
///////////////////////////////////////////////////////////////////////////////
uint8_t touchpad_fingerCount()
{
    return tpDrv->fingerCount();       // considered active when any finger is down
}


///////////////////////////////////////////////////////////////////////////////
//  Check if TP intr pin is active
///////////////////////////////////////////////////////////////////////////////
uint8_t touchpad_getPinActive()
{
    return intrVtblPtr->isInterruptPinActive(&tpIntr);
}

///////////////////////////////////////////////////////////////////////////////
//  Wait for TP event
/// \param waintInMs: time to wait for in msec
/// \return Whether there is event before wait timeout
///////////////////////////////////////////////////////////////////////////////
uint8_t touchpad_waitForEvent(uint32_t waitInMs)
{
    return tpDrv->waitForRDY(INTR_LVL_HIGH,waitInMs);
}

////////////////////////////////////////////////////////////////////////////////
/// Callback to handle GPIO activity interrupt due to peripherals, e.g. touchpad
////////////////////////////////////////////////////////////////////////////////
void gpioActivityDetected(void *appData, uint8_t portPin)
{
    app_appActivityDetected(appData);
}

////////////////////////////////////////////////////////////////////////////////
/// Poll touchpad activity
////////////////////////////////////////////////////////////////////////////////
uint8_t pollTouchpadActivity(void)
{
    // check for touuchpad
    if (touchpad->getPinActive())
    {
        uint8_t dataAvailable;
        dataAvailable = touchpad->pollActivity(&bleRemoteAppState->touchpadEvent, FALSE);

        // ignore touchpad events?
        if (!hidd_link_is_discoverable())
        {
            if (!hidd_is_paired())
            {
                if (!touchpad->proximityRpt())      // Don't use proximity finger event to initiate discovery
                {
                    hidd_link_connect();
                }
            }
            else if ( dataAvailable )
            {
                if (dataAvailable != HID_EVENT_TP_INFO)
                {
                    if (!audio_is_active())
                    {
                        //WICED_BT_TRACE("\npollTouchpadActivity: dataAvailable=%d, prt=0x%x", dataAvailable, bleRemoteAppState->touchpadEvent.userDataPtr);
                        wiced_hidd_event_queue_add_event_with_overflow(&app.eventQueue,
                              &bleRemoteAppState->touchpadEvent.eventInfo,  sizeof(bleRemoteAppState->touchpadEvent),
                              app.pollSeqn);

                        return HIDLINK_ACTIVITY_REPORTABLE;
                    }
                }
                return (hidd_link_is_connected() && !touchpad->proximityRpt())?
                    HIDLINK_ACTIVITY_NONE : HIDLINK_ACTIVITY_REPORTABLE;
            }
        }
    }
    return HIDLINK_ACTIVITY_NONE;
}

////////////////////////////////////////////////////////////////////////////////
/// Process TP event
////////////////////////////////////////////////////////////////////////////////
void app_procEvtTouchpad(void)
{
    HidEventUserDefine *tp_event = (HidEventUserDefine *)wiced_hidd_event_queue_get_current_element(&app.eventQueue);
    AbsXYReport *tp = (AbsXYReport* )tp_event->userDataPtr;

    //set gatt attribute value here before sending the report
    memset(app_touchpad_rpt, 0, TOUCHPAD_REPORT_DATA_SIZE(tp));
    memcpy(app_touchpad_rpt, TOUCHPAD_REPORT_DATA(tp), TOUCHPAD_REPORT_DATA_SIZE(tp));

    hidd_link_send_report(tp->reportID, WICED_HID_REPORT_TYPE_INPUT, app_touchpad_rpt, TOUCHPAD_REPORT_DATA_SIZE(tp));
}

#if USE_TOUCHPAD_VIRTUAL_KEY
////////////////////////////////////////////////////////////////////////////////
/// Handle virtual key generation upon pressing key embedded under tocuhpad
/// \param ke: On input the key event corresponding to touchpad key
///            On output updated with the virtual key index based on zone touched.
/// \return True if a virtual key pressed, i.e. a zone touched that maps to a key.
////////////////////////////////////////////////////////////////////////////////
static void virtual_key(uint8_t * keyCode, wiced_bool_t isDown)
{
    static uint8_t v_key;

    if (isDown)
    {
        uint8_t zone = getZone();

        // we translate the key if valid zone found
        v_key = (zone < ZONE_UNDEFINED) ? zone + KB_MAX_KEYS : *keyCode;
    }
    *keyCode = v_key;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void touchpad_validate_key(uint8_t * keyCode, wiced_bool_t down)
{
    if (touchpad && (TOUCHPAD_BUTTON_KEYINDEX == *keyCode))
    {
        // this function will update key event to the real event we want to send
        sendKey = virtual_key(keyCode, down);
    }
}
#endif
///////////////////////////////////////////////////////////////////////////////
// clear touchpad data buffer. If the last report was finger down, send finger up
///////////////////////////////////////////////////////////////////////////////
void touchpad_flush()
{

    if (touchpad)
    {
        touchpad->clearEvent();
        AbsXYRptPtr tp = touchpad->getAbsFingerUpRpt();
        if (tp)
        {
            //set gatt attribute value here before sending the report
            memset(app_touchpad_rpt, 0, TOUCHPAD_RPT_PAYLOAD_SIZE);
            memcpy(app_touchpad_rpt, TOUCHPAD_REPORT_DATA(tp), TOUCHPAD_REPORT_DATA_SIZE(tp));

            hidd_link_send_report(tp->reportID, WICED_HID_REPORT_TYPE_INPUT, app_touchpad_rpt, TOUCHPAD_REPORT_DATA_SIZE(tp));
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
wiced_bool_t touchpad_event(wiced_hidd_app_event_queue_t * eventQ)
{
    HidEventUserDefine *pEvt;
    wiced_bool_t handled = FALSE;

    while (((pEvt = (HidEventUserDefine *)wiced_hidd_event_queue_get_current_element(eventQ)) != NULL) &&
           ((pEvt->eventInfo.eventType == HID_EVENT_TP_FINGER_STATUS_CHANGE) || (pEvt->eventInfo.eventType == HID_EVENT_TP_DATA)))
    {
        app_procEvtTouchpad();
        wiced_hidd_event_queue_remove_current_element(eventQ);
        handled = TRUE;
    }

    return handled;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void touchpad_init(void (*callBack)(void*, uint8_t))
{
    touchpad = TouchPad(callBack, NULL);

    //touchpad event
    bleRemoteAppState->touchpadEvent.eventInfo.eventType = HID_EVENT_TP_DATA;

    if (wiced_hal_mia_is_reset_reason_por())
    {
        touchpad->reInitialize();
    }
    else
    {
        touchpad->setEnable(TRUE);
    }

    // reenforce GPIO configuration so we don't lose it after entering uBCS
    wiced_hal_gpio_slimboot_reenforce_cfg(GPIO_RSTN_TP, GPIO_OUTPUT_ENABLE);
}
#else // !SUPPORT_TOUCHPAD

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
void touchpad_init(void (*callBack)(void*, uint8_t))
{
 #ifdef REMOTE_PLATFROM
    wiced_hal_gpio_configure_pin(GPIO_RSTN_TP, GPIO_OUTPUT_ENABLE, GPIO_TOUCHPAD_OFF);
 #endif
}

#endif // SUPPORT_TOUCHPAD
