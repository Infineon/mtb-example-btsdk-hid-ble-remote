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
* Remote Control Application
*
* The BLE Remote Control application is a single chip SoC compliant with HID over GATT Profile (HOGP).
* Supported features include key, microphone (voice over HOGP), Infrared Transmit (IR TX), touchpad.
*
* During initialization the app registers with LE stack, WICED HID Device Library and
* keyscan and external HW peripherals to receive various notifications including
* bonding complete, connection status change, peer GATT request/commands and
* interrupts for key pressed/released, ADC audio, and Touchpad.
* Press any key will start LE advertising. When device is successfully bonded, the app
* saves bonded host's information in the NVRAM.
* When user presses/releases any key, a key report will be sent to the host.
* On connection up or battery level changed, a battery report will be sent to the host.
* When battery level is bellowed shutdown voltage, device will critical shutdown.
* When user presses and holds microphone key, voice streaming starts until user releases
* microphone key.
* When user presses and holds power key, IR TX starts until power key is released.
*
* Features demonstrated
*  - GATT database and Device configuration initialization
*  - Registration with LE stack for various events
*  - Sending HID reports to the host
*  - Processing write requests from the host
*  - Low power management
*  - Over the air firmware update (OTAFWU)
*
* See readme for instructions.
*/
#include "app.h"
#include "wiced_hal_mia.h"
#include "wiced_memory.h"

#define RECOVERY_COUNT 3
#define keyscanActive() (wiced_hal_keyscan_is_any_key_pressed() || wiced_hal_keyscan_events_pending())

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
typedef struct {
    wiced_hidd_app_event_queue_t eventQueue;
    app_queue_t events[APP_QUEUE_MAX];
    uint8_t pollSeqn;
    uint8_t recoveryInProgress;
    uint8_t setReport_status;

    uint8_t firstTransportStateChangeNotification:1;
} app_t;

static app_t app = {
.firstTransportStateChangeNotification = 1,
};

/////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_connect_button
/////////////////////////////////////////////////////////////////////////////////
/// Checks for if the current key is connet button.
/// If it is, handle the action accordingly and return TRUE to indicate it is taken care of.
/// Otherwise, it returns FALSE.
///
/// Parameter:
///   keyCode -- the key to check.
///   Down -- TRUE to indicate the key is pressed down.
///
/// Return:
///   TRUE -- keyCode is connect button and it is handled.
///   FALSE -- keyCode is not connect button.
///
////////////////////////////////////////////////////////////////////////////////
static wiced_bool_t APP_connect_button(uint8_t keyCode, wiced_bool_t down)
{
    if (keyCode == CONNECT_KEY_INDEX)
    {
        WICED_BT_TRACE("\nConnect button=%s", down?"DN":"UP");
        if (down)
        {
#ifdef CONNECTED_ADVERTISING_SUPPORTED
            hidd_blelink_allowDiscoverable();
#else
            hidd_link_virtual_cable_unplug();
            hidd_enter_pairing();
#endif
        }
        return TRUE;
    }
    return FALSE;
}

/////////////////////////////////////////////////////////////////////////////////
/// This is a callback function from keyscan when key action is detected
/////////////////////////////////////////////////////////////////////////////////
static void APP_keyDetected(HidEventKey* kbKeyEvent)
{
    static uint8_t suppressEndScanCycleAfterConnectButton = TRUE;
#if ANDROID_AUDIO
    static uint8_t audio_key_down = FALSE;
#endif
    uint8_t keyDown = kbKeyEvent->keyEvent.upDownFlag == KEY_DOWN;
    uint8_t keyCode = kbKeyEvent->keyEvent.keyCode;

#ifdef CONNECTED_ADVERTISING_SUPPORTED
    // while in CONNECTED - advertising state, any key press shall terminate terminate advertising and go back to CONNECTED state
    if (blelink.second_conn_state && hidd_link_is_connected() &&
        (keyCode != END_OF_SCAN_CYCLE) && keyDown)
    {
        wiced_bt_start_advertisements(BTM_BLE_ADVERT_OFF, 0, NULL);
    }
#endif

    // Check for buttons
    if (!APP_connect_button(keyCode, keyDown)
#ifdef SUPPORT_IR
        && !ir_button(keyCode, keyDown)
#endif
#if HID_AUDIO
        && !audio_button(keyCode, keyDown)
#endif
       )
    {
        // Check if this is an end-of-scan cycle event
        if (keyCode == END_OF_SCAN_CYCLE)
        {
            // Yes. Queue it if it need not be suppressed
            if (!suppressEndScanCycleAfterConnectButton)
            {
                wiced_hidd_event_queue_add_event_with_overflow(&app.eventQueue, &kbKeyEvent->eventInfo, sizeof(HidEventKey), app.pollSeqn);
            }

#if ANDROID_AUDIO
            if (audio_key_down)
            {
                audio_key_down = FALSE;
                audio_START_REQ();
            }
#endif
            // Enable end-of-scan cycle suppression since this is the start of a new cycle
            suppressEndScanCycleAfterConnectButton = TRUE;
        }
        else if (keyCode != ROLLOVER)
        {
            // If this is touchpad button, translate to virtual key
            touchpad_validate_key(&kbKeyEvent->keyEvent.keyCode, keyDown);

            WICED_BT_TRACE("\nkc:%d %c", kbKeyEvent->keyEvent.keyCode, keyDown ? 'D':'U');

            // No. Queue the key event
            wiced_hidd_event_queue_add_event_with_overflow(&app.eventQueue, &kbKeyEvent->eventInfo, sizeof(HidEventKey), app.pollSeqn);

#if ANDROID_AUDIO
            if (keyCode == AUDIO_KEY_INDEX && keyDown)
            {
                audio_key_down = TRUE;
            }
#endif
            // Disable end-of-scan cycle suppression
            suppressEndScanCycleAfterConnectButton = FALSE;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_stdErrResp
////////////////////////////////////////////////////////////////////////////////
/// This function provides a standard response for errors. The response is:
///     - A rollover report is sent to the host if we are not already recovering from an error
///     - All reports are cleared and marked as modified. They will be sent once
///       we have recovered from the error.
///     - Marks the func-lock key as up but dow not toggle its state even if
///       the associated toggle flag is set. This allows for proper reconstruction
///       of the keyboard state including func-lock dependent keys after the recovery
///     - The recovery poll count is also set to the configured value.
///     - Connect button state is cleared since we don't know if the connect button press
///       is valid
///
/// Parameter:
///   none
///
/// Return:
///   none
///
////////////////////////////////////////////////////////////////////////////////
static void APP_stdErrResp(void)
{
    // Send a rollover report
    if (!app.recoveryInProgress)
    {
        // Send rollover report
        key_sendRollover();
    }

    // Reset recovery timeout
    app.recoveryInProgress = RECOVERY_COUNT;

    key_clear(FALSE);
}

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_stdErrRespWithFwHwReset
////////////////////////////////////////////////////////////////////////////////
/// This function provides a standard response identical to kbAppStdErrorResponse.
/// In addition, it also performs the following actions:
///     - All pending events are flushed
///     - The keyscan HW is reset
/// This function is typically used when the FW itself is (or involved in) in error.
/// In such cases the FW no longer has the correct state of anything and we
/// must resort to a total reset
/// This function is called when battery voltage drops below the configured threshold.
///
/// Parameter:
///   none
///
/// Return:
///   none
///
////////////////////////////////////////////////////////////////////////////////
static void APP_stdErrRespWithFwHwReset(void)
{
    // Provide standard error response
    APP_stdErrResp();

    // Flush the event fifo
    wiced_hidd_event_queue_flush(&app.eventQueue);

    // reset keyscan
    kscan_reset();
}

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_procEvtUserDefined
////////////////////////////////////////////////////////////////////////////////
/// Process a user defined event. By default the keyboard application
/// define key and scroll events. If an application needs additional types of
/// events it should define them and override this function to process them.
/// This function should remove the user defined event from the event queue
/// after processing it. This function can consume additional events after
/// the user defined event.
///
/// Parameter:
///   none
///
/// Return:
///   none
///
////////////////////////////////////////////////////////////////////////////////
static void APP_procEvtUserDefined(void)
{
    if (!audio_event(&app.eventQueue) &&
        !touchpad_event(&app.eventQueue))
    {
        // no one claims for it, remove it
        if (wiced_hidd_event_queue_get_current_element(&app.eventQueue)!= NULL)
        {
            wiced_hidd_event_queue_remove_current_element(&app.eventQueue);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_shutdown
////////////////////////////////////////////////////////////////////////////////
/// This function is called when battery voltage drops below the configured threshold.
///
/// Parameter:
///   none
///
/// Return:
///   none
///
////////////////////////////////////////////////////////////////////////////////
static void APP_shutdown(void)
{
    WICED_BT_TRACE("\napp_shutdown");

    // Flush the event queue
    wiced_hidd_event_queue_flush(&app.eventQueue);

    kscan_shutdown();
    audio_shutdown();
    touchpad_shutdown();

    if(hidd_link_is_connected())
    {
        hidd_link_disconnect();
    }
    // Disable Interrupts
    wiced_hal_mia_enable_mia_interrupt(FALSE);
    wiced_hal_mia_enable_lhl_interrupt(FALSE);
}

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_pollActivityUser
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///  This function provides an implementation for the HID application abstract
///  function pollActivityUser(). It polls the following sources for user activity:
///        - Keys
///  Any detected activity is queued as events in the event fifo.
///  When pin code entry is in progress, this function will also call
///  handlePinCodeEntry to do pin code processing.
///
/// \return
///   Bit mapped value indicating
///       - HIDLINK_ACTIVITY_NON_REPORTABLE - if any key (excluding connect button) is down. Always
///         set in pin code entry state
///       - HIDLINK_ACTIVITY_REPORTABLE - if any event is queued. Always
///         set in pin code entry state
///       - HIDLINK_ACTIVITY_NONE otherwise
///  As long as it is not HIDLINK_ACTIVITY_NONE, the btlpm will be notified for low power management.
////////////////////////////////////////////////////////////////////////////////
uint8_t APP_pollActivityUser(void)
{
    uint8_t status;

    // Poll the hardware for events
    wiced_hal_mia_pollHardware();

    // Poll and queue key activity
    kscan_pollActivity();

    // For all other cases, return value indicating whether any event is pending or
    status = wiced_hidd_event_queue_get_num_elements(&app.eventQueue) || kscan_is_any_key_pressed() ? HIDLINK_ACTIVITY_REPORTABLE : HIDLINK_ACTIVITY_NONE;

    audio_pollActivityVoice();
    touchpad_pollActivity(status);

    return status;
}

////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_sleep_handler
////////////////////////////////////////////////////////////////////////////////
/// Summary:
///    Sleep permit query to check if sleep is allowed and sleep time
///
/// Parameters:
///  type -- quary type. It can be WICED_SLEEP_POLL_TIME_TO_SLEEP or WICED_SLEEP_POLL_SLEEP_PERMISSION
///
/// Return:
///  WICED_SLEEP_NOT_ALLOWED -- not allow to sleep
///  When WICED_SLEEP_POLL_TIME_TO_SLEEP:
///     WICED_SLEEP_MAX_TIME_TO_SLEEP or the time to sleep
///  When WICED_SLEEP_POLL_SLEEP_PERMISSION:
///     WICED_SLEEP_ALLOWED_WITH_SHUTDOWN -- allowed to sleep, but no SDS nor ePDS
///     WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN -- allowed to enter SDS/ePDS
///
////////////////////////////////////////////////////////////////////////////////
static uint32_t APP_sleep_handler(wiced_sleep_poll_type_t type )
{
    uint32_t ret = WICED_SLEEP_NOT_ALLOWED;

#if SLEEP_ALLOWED
    switch(type)
    {
        case WICED_SLEEP_POLL_TIME_TO_SLEEP:
            if (!(app.recoveryInProgress
                  || keyscanActive()
                  || ir_is_active()
                  || audio_is_active()
                  || touchpad_is_active()
                  || findme_is_active()
                 )
               )
            {
                ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
            }
            break;

        case WICED_SLEEP_POLL_SLEEP_PERMISSION:
 #if SLEEP_ALLOWED > 1
            ret = WICED_SLEEP_ALLOWED_WITH_SHUTDOWN;
            // a key is down, no deep sleep
            if ( keyscanActive()
  #ifdef SUPPORT_TOUCHPAD
                || touchpad_is_active()
  #endif
  #ifdef SUPPORT_AUDIO
                || audio_is_active()
  #endif
               )
 #endif
            ret = WICED_SLEEP_ALLOWED_WITHOUT_SHUTDOWN;
            break;
    }
#endif

    return ret;
}

/////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_procErrEvtQueue
////////////////////////////////////////////////////////////////////////////////
/// This function handles event queue errors. This includes event queue overflow
/// unexpected events, missing expected events, and events in unexpected order.
/// This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
/// to address the problem. A user defined implementation should at least
/// remove the first element in the queue if this event is an overflow event
///
/// Parameter:
///   none
///
/// Return:
///   none
///
////////////////////////////////////////////////////////////////////////////////
static void APP_procErrEvtQueue(void)
{
    WICED_BT_TRACE("\nKSQerr");
    APP_stdErrRespWithFwHwReset();
}

////////////////////////////////////////////////////////////////////////////////
/// This function handles error events reported by the keyscan HW. Typically
/// these would be ghost events.
/// This function does a FW/HW reset via stdErrRespWithFwHwReset in an attempt
/// to address the problem.
/// NOTE: if audio stop event was flushed out, put it back into event queue too.
////////////////////////////////////////////////////////////////////////////////
static void APP_procErrKeyscan(void)
{
    WICED_BT_TRACE("\nAPP_procErrKeyscan");

    //call base class handling
    APP_stdErrRespWithFwHwReset();

    key_clear(TRUE);
    touchpad_flush();
}

/////////////////////////////////////////////////////////////////////////////////
/// Function Name: APP_generateAndTxReports
/////////////////////////////////////////////////////////////////////////////////
/// This function provides an implementation for the generateAndTxReports() function
/// defined by the HID application. This function is only called when the active transport
/// is connected. This function performs the following actions:
///  - When pin code entry is in progress, the behavior of this function is changed.
///    It only checks and transmits the pin code report; normal event processing is
///    suspended.
///  - If the number of packets in the hardware fifo is less than the report generation
///    threshold and the event queue is not empty, this function will process events
///    by calling the event processing functions, e.g. procEvtKey() etc
///  - This function also tracks the recovery period after an error. If
///    the recovery count is non-zero, it is decremented as long as there is room
///    for one report in the transport
///
/// Parameter:
///   none
///
/// Return:
///   none
///
/////////////////////////////////////////////////////////////////////////////////
static void APP_generateAndTxReports(void)
{
    app_queue_t *curEvent;

    {
        // Normal state

        // If we are recovering from an error, decrement the recovery count as long as the transport
        // has room. Avoid the case where no event processing is done during recovery because
        // transport is full, as the failure might be a non-responding transport.
        if (app.recoveryInProgress)
        {
            // If recovery is complete, transmit any modified reports that we have been hoarding
            if (!--app.recoveryInProgress)
            {
                key_send();
            }
        }

        // Continue report generation as long as the transport has room and we have events to process
        while ((wiced_bt_buffer_poolutilization (HCI_ACL_POOL_ID) < 80) &&
               ((curEvent = (app_queue_t *)wiced_hidd_event_queue_get_current_element(&app.eventQueue)) != NULL))
        {
            // Further processing depends on the event type
            switch (curEvent->type)
            {
                case HID_EVENT_KEY_STATE_CHANGE:
                    // process the event key. If fails, resets keys
                    if (!key_procEvtKey(curEvent->key.keyEvent.keyCode, curEvent->key.keyEvent.upDownFlag == KEY_DOWN))
                    {
                        APP_procErrKeyscan();
                    }
                    wiced_hidd_event_queue_remove_current_element(&app.eventQueue);
                    break;

                case HID_EVENT_EVENT_FIFO_OVERFLOW:
                    WICED_BT_TRACE("\nHID_EVENT_EVENT_FIFO_OVERFLOW");
                    // Call event queue error handler
                    APP_procErrEvtQueue();
                    wiced_hidd_event_queue_remove_current_element(&app.eventQueue);
                    break;

                default:
                    APP_procEvtUserDefined();
                    break;
            }

            // The current event should be deleted by the event processing function.
            // Additional events may also be consumed but we don't care about that
        }

    }
}

/*******************************************************************************
 * Function Name: APP_pollReportUserActivity
 ********************************************************************************
 * Summary:
 *  This function should be called by the transport when it wants the application
 *  to poll for user activity. This function performs the following actions:
 *   - Polls for activity. If user activity is detected, events should be
 *     queued up for processing
 *   - If an unmasked user activity is detected, it passes the activity type to the
 *     transports
 *   - If the active transport is connected, requests generation of reports via
 *     generateAndTransmitReports()
 *   - Does connect button polling and informs the BT transport once the connect
 *     button has been held for the configured amount of time.
 *  Note: transport may be NULL if no transport context is required - like when
 *  none
 *
 * Parameters:
 *  newState new state of the link
 *
 * Return:
 *  none
 *
 *******************************************************************************/
static void APP_pollReportUserActivity(void)
{
    uint8_t activitiesDetectedInLastPoll;

    // Increment polling sequence number.
    app.pollSeqn++;

    // Check for activity. This should queue events if any user activity is detected
    activitiesDetectedInLastPoll = APP_pollActivityUser();

    // Check if the active transport is connected
    if(hidd_link_is_connected())
    {
        if((app.pollSeqn % 128) == 0)
        {
            WICED_BT_TRACE(".");
        }

        // Generate a report
        if(!bt_cfg.security_requirement_mask || hidd_link_is_encrypted())
        {
            APP_generateAndTxReports();
        }

        if (audio_is_active())
        {
            audio_pollActivityVoice();
        }
        else
        {
            if (!ota_is_active())
            {
                // poll for battery monitor
                bat_poll();
            }
        }
    }
    else
    {
        // Check if we have any user activity. If it is paired and not connected, we reconnect.
        if (activitiesDetectedInLastPoll != HIDLINK_ACTIVITY_NONE && hidd_is_paired())
        {
            // ask the transport to connect.
            hidd_link_connect();
        }
    }
}

#ifdef TESTING_USING_HCI
/////////////////////////////////////////////////////////////////////////////////
/// This is a callback function from keyscan when key action is detected
/////////////////////////////////////////////////////////////////////////////////
static void APP_hci_key_event(uint8_t keyCode, wiced_bool_t keyDown)
{
    // Translate to app defined key
    switch (keyCode)
    {
        case KEY_IR:
            keyCode = IR_KEY_INDEX;
            break;

        case KEY_AUDIO:
            keyCode = AUDIO_KEY_INDEX;
            break;

        case KEY_CONNECT:
            keyCode = CONNECT_KEY_INDEX;
            break;

//        case KEY_MOTION:          // ignored
        // unknown key, ignored
        default:
            WICED_BT_TRACE("\nFunction not supported");
            keyCode = ROLLOVER;     // mark as unhandled key
            break;
    }

    if (keyCode!=ROLLOVER) // known key
    {
        HidEventKey kbKeyEvent = {{HID_EVENT_KEY_STATE_CHANGE}};

        kbKeyEvent.keyEvent.upDownFlag = keyDown ? KEY_DOWN : KEY_UP;
        kbKeyEvent.keyEvent.keyCode = keyCode;
        APP_keyDetected(&kbKeyEvent);

        kbKeyEvent.keyEvent.keyCode = END_OF_SCAN_CYCLE;
        APP_keyDetected(&kbKeyEvent);

        APP_pollReportUserActivity();
    }
}
#endif

/*******************************************************************************
 * Function Name: APP_setReport
 ********************************************************************************
 * Summary:
 *  This function implements the rxSetReport function defined by
 *  the HID application to handle "Set Report" messages.
 *  This function looks at the report ID and passes the message to the
 *  appropriate handler.
 *
 * Parameters:
 *  reportType -- type of incoming report,  WICED_HID_REPORT_TYPE_INPUT, WICED_HID_REPORT_TYPE_OUTPUT, or WICED_HID_REPORT_TYPE_FEATURE
 *  payload -- pointer to data that came along with the set report request after the report ID
 *  payloadSize -- size of the payload excluding the report ID
 *
 * Return:
 *  none
 *
 *******************************************************************************/
uint8_t APP_setReport(wiced_hidd_report_type_t reportType, uint8_t *payload, uint16_t payloadSize)
{
    uint8_t reportId = *payload++;
    app_setReport(reportType, reportId, payload, --payloadSize);
    return app.setReport_status;
}

/*******************************************************************************
 * Function Name: app_setreport
 ********************************************************************************
 * Summary:
 *  This function implements the rxSetReport function defined by
 *  the HID application to handle "Set Report" messages.
 *  This function looks at the report ID and passes the message to the
 *  appropriate handler.
 *
 * Parameters:
 *  reportType -- type of incoming report,  WICED_HID_REPORT_TYPE_INPUT, WICED_HID_REPORT_TYPE_OUTPUT, or WICED_HID_REPORT_TYPE_FEATURE
 *  reportId -- report id
 *  payload -- pointer to data that came along with the set report request after the report ID
 *  payloadSize -- size of the payload excluding the report ID
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_setReport(wiced_hidd_report_type_t reportType,
                     uint8_t reportId,
                     void *payload,
                     uint16_t payloadSize)
{
    WICED_BT_TRACE("\napp_setReport: %d", payloadSize);

#ifdef HID_AUDIO
    if (audio_setReport(reportType, reportId, payload, payloadSize))
    {
        return;
    }
#endif
    if (key_setReport(reportType, reportId, payload, payloadSize))
    {
        return;
    }
#ifdef PTS_HIDS_CONFORMANCE_TC_CW_BV_03_C
    if ((reportType == WICED_HID_REPORT_TYPE_FEATURE) && (reportId == RPT_ID_IN_CNT_CTL))
    {
        app_connection_ctrl_rpt = *((uint8_t*)payload);
        WICED_BT_TRACE("\nPTS_HIDS_CONFORMANCE_TC_CW_BV_03_C write val: %d ", app_connection_ctrl_rpt);
    }
#endif
}

/*******************************************************************************
 * Function Name: app_queueEvent
 ********************************************************************************
 * Summary:
 *  Queue an event to event queue
 *
 * Parameters:
 *  event -- event to queue
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_queueEvent(app_queue_t * event)
{
    wiced_hidd_event_queue_add_event_with_overflow(&app.eventQueue, &event->info, APP_QUEUE_SIZE, app.pollSeqn);
}

/*******************************************************************************
 * Function Name: app_transportStateChangeNotification
 ********************************************************************************
 * Summary:
 *    This function informs the application that the state of a link changed.
 *
 * Parameters:
 *  newState new state of the link
 *
 * Return:
 *  none
 *
 *******************************************************************************/
void app_transportStateChangeNotification(uint32_t newState)
{
    int16_t flags;
    WICED_BT_TRACE("\nTransport state changed to ", newState);
    hidd_led_blink_stop(LED_LE_LINK);

    hidd_set_deep_sleep_allowed(WICED_FALSE);

    switch (newState) {
    case HIDLINK_LE_CONNECTED:
        WICED_BT_TRACE("connected");
        hidd_led_on(LED_LE_LINK);

        // enable ghost detection
        kscan_enable_ghost_detection(TRUE);

#ifdef SUPPORT_TOUCHPAD
        if (touchpad)
        {
            touchpad->setEnable(TRUE);
        }
#endif
        hidd_link_enable_poll_callback(BT_TRANSPORT_LE, WICED_TRUE);

        if(app.firstTransportStateChangeNotification)
        {
            //Wake up from HID Off and already have a connection then allow HID Off in 1 second
            //This will allow time to send a key press.
            //To do need to check if key event is in the queue at lpm query
            hidd_deep_sleep_not_allowed(1000); // 1 second. timeout in ms
        }
        else
        {
            //We connected after power on reset or HID off recovery.
            //Start 20 second timer to allow time to setup connection encryption
            //before allowing HID Off/Micro-BCS.
            hidd_deep_sleep_not_allowed(20000); //20 seconds. timeout in ms

        }
        break;

    case HIDLINK_LE_DISCONNECTED:
        WICED_BT_TRACE("disconnected");
        hidd_led_off(LED_LE_LINK);

        // disable Ghost detection
        kscan_enable_ghost_detection(FALSE);

#ifdef SUPPORT_AUDIO
        //stop audio
        hidd_mic_audio_stop();
#endif
        // Tell the transport to stop polling
        hidd_link_enable_poll_callback(BT_TRANSPORT_LE, WICED_FALSE);

        //allow Shut Down Sleep (SDS) only if we are not attempting reconnect
        if (!hidd_link_is_reconnect_timer_running())
            hidd_deep_sleep_not_allowed(2000); // 2 seconds. timeout in ms
        break;

    case HIDLINK_LE_DISCOVERABLE:
        WICED_BT_TRACE("discoverable");
        hidd_led_blink(LED_LE_LINK, 0, 500);
        touchpad_flush();
        break;

    case HIDLINK_LE_RECONNECTING:
        WICED_BT_TRACE("reconnecting");
        hidd_led_blink(LED_LE_LINK, 0, 200);     // faster blink LINK line to indicate reconnecting
        break;

    case HIDLINK_LE_ADVERTISING_IN_uBCS_DIRECTED:
    case HIDLINK_LE_ADVERTISING_IN_uBCS_UNDIRECTED:
        hidd_set_deep_sleep_allowed(WICED_TRUE);
        break;
    }

    app.firstTransportStateChangeNotification = 0;
}

/*******************************************************************************
 * sleep configuration
 *******************************************************************************/
wiced_sleep_config_t    hidd_link_sleep_config = {
    . sleep_mode            = WICED_SLEEP_MODE_NO_TRANSPORT,  //sleep_mode
    .host_wake_mode         = 0,                              //host_wake_mode
    .device_wake_mode       = 0,                              //device_wake_mode
    .device_wake_source     = WICED_SLEEP_WAKE_SOURCE_GPIO | WICED_SLEEP_WAKE_SOURCE_KEYSCAN | WICED_SLEEP_WAKE_SOURCE_QUAD,  //device_wake_source
    .device_wake_gpio_num   = 255,                            //must set device_wake_gpio_num to 255 for WICED_SLEEP_MODE_NO_TRANSPORT
    .sleep_permit_handler   = APP_sleep_handler,              //sleep_permit_handler
#if defined(CYW20819A1) || defined(CYW20820A1)
    .post_sleep_cback_handler=NULL,                           //post_sleep_handler
#endif
};

/*******************************************************************************
 * Callback functions
 *******************************************************************************/
static hidd_link_callback_t appCallbacks =
{
    .p_app_poll_user_activities                 = APP_pollReportUserActivity,
    .p_app_connection_failed_notification       = NULL,

#ifdef SUPPORT_CODE_ENTRY
    .p_app_enter_pincode_entry_mode             = NULL,
    .p_app_enter_passcode_entry_mode            = NULL,
    .p_app_exit_pin_and_passcode_entry_mode     = NULL,
#endif
    .p_app_get_idle                             = NULL,
    .p_app_set_idle                             = NULL,
    .p_app_get_protocol                         = NULL,
    .p_app_set_protocol                         = NULL,
    .p_app_get_report                           = NULL,
    .p_app_set_report                           = APP_setReport,
    .p_app_rx_data                              = NULL,
};

/*******************************************************************************
 * Function Name: app_start()
 ********************************************************************************
 * Summary: This is application start function. After system is up, when the
 *          bt management calls with BTM_ENABLED_EVT, this function is called to
 *          start application
 *
 * Parameters:
 *  none
 *
 * Return:
 *  WICED_BT_SUCCESS -- if initialization is okay and ready to start;
 *                      otherwise, should return the error code defined in wiced_result_t
 *
 *******************************************************************************/
wiced_result_t app_start(void)
{
    WICED_BT_TRACE("\napp_start");

    // allocate necessary memory and initialize event queue
    wiced_hidd_event_queue_init(&app.eventQueue, (uint8_t *)&app.events, APP_QUEUE_SIZE, APP_QUEUE_MAX);

    // register applicaton callbacks
    hidd_link_register_callbacks(&appCallbacks);

    hidd_hci_control_register_key_handler(APP_hci_key_event);

    /* transport init */
    bt_init();
    hidd_sleep_configure(&hidd_link_sleep_config);

    /* component/peripheral init */
    bat_init(APP_shutdown);
    ir_init(IR_TX_GPIO);
    audio_init(APP_pollReportUserActivity);
    touchpad_init(gpioActivityDetected);
    findme_init();
    key_init(NUM_KEYSCAN_ROWS, NUM_KEYSCAN_COLS, APP_pollReportUserActivity, APP_keyDetected);

    hidd_link_init(); // linitialize link last

    WICED_BT_TRACE("\nFree RAM bytes=%d bytes", wiced_memory_get_free_bytes());

    return WICED_BT_SUCCESS;
}

#ifdef ANDROID_AUDIO
/*
 * bleremote ble link management callbacks
 */
wiced_result_t app_management_cback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_RESUME_HIDD_LIB_HANDLER;

    switch( event )
    {
        case BTM_ENCRYPTION_STATUS_EVT:
            if (p_event_data->encryption_status.result == WICED_SUCCESS)
            {
                //configure ATT MTU size with peer device
                wiced_bt_gatt_configure_mtu(blelink.gatts_conn_id, bt_cfg.gatt_cfg.max_mtu_size);
            }
            // let default library to handle the reset
            break;
    }
    return result;
}
#else
 #define app_management_cback NULL
#endif

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
void application_start( void )
{
    extern const wiced_platform_led_config_t platform_led[];
    extern const size_t led_count;

    // Initialize LED/UART for debug
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
    hidd_led_init(led_count, platform_led);

    app_init(app_start, app_management_cback);

#if (SLEEP_ALLOWED == 3)
    hidd_allowed_hidoff(TRUE);
#endif

    WICED_BT_TRACE("\nDEV=%d Version:%d.%d Rev=%d Build=%d",hidd_chip_id(), WICED_SDK_MAJOR_VER, WICED_SDK_MINOR_VER, WICED_SDK_REV_NUMBER, WICED_SDK_BUILD_NUMBER);
    WICED_BT_TRACE("\nSLEEP_ALLOWED=%d",SLEEP_ALLOWED);
    WICED_BT_TRACE("\nLED=%d",LED_SUPPORT);

#ifdef OTA_FIRMWARE_UPGRADE
    WICED_BT_TRACE("\nOTA_FW_UPGRADE");
 #ifdef OTA_SECURE_FIRMWARE_UPGRADE
    WICED_BT_TRACE("\nOTA_SEC_FW_UPGRADE");
 #endif
#endif

#ifdef SUPPORT_AUDIO
 #ifdef CELT_ENCODER
    WICED_BT_TRACE("\nENABLE_AUDIO(CELT)");
 #elif defined(ADPCM_ENCODER)
  #ifdef ANDROID_AUDIO
   #ifdef ANDROID_AUDIO_1_0
    WICED_BT_TRACE("\nANDROID_AUDIO(ADPCM)");
   #else
    WICED_BT_TRACE("\nANDROID_AUDIO 0.4(ADPCM)");
   #endif
  #else
    WICED_BT_TRACE("\nENABLE_AUDIO(ADPCM)");
  #endif
 #else
    WICED_BT_TRACE("\nENABLE_AUDIO(mSBC)");
 #endif
 #ifdef SUPPORT_DIGITAL_MIC
    WICED_BT_TRACE("\nPDM");
 #endif
#endif

#ifdef AUTO_RECONNECT
    WICED_BT_TRACE("\nAUTO_RECONNECT");
#endif

#ifdef SKIP_CONNECT_PARAM_UPDATE_EVEN_IF_NO_PREFERED
    WICED_BT_TRACE("\nSKIP_PARAM_UPDATE");
#endif

#ifdef START_ADV_WHEN_POWERUP_NO_CONNECTED
    WICED_BT_TRACE("\nSTART_ADV_ON_POWERUP");
#endif

#ifdef CONNECTED_ADVERTISING_SUPPORTED
    WICED_BT_TRACE("\nENABLE_CONNECTED_ADV");
#endif

#ifdef ENDLESS_LE_ADVERTISING
    WICED_BT_TRACE("\nENDLESS_ADV");
#endif

#ifdef ASSYM_PERIPHERAL_LATENCY
    WICED_BT_TRACE("\nASSYMETRIC_PERIPHERAL_LATENCY");
#endif

#ifdef LE_LOCAL_PRIVACY_SUPPORT
    WICED_BT_TRACE("\nLE_LOCAL_PRIVACY_SUPPORT");
#endif

#ifdef SUPPORT_IR
    WICED_BT_TRACE("\nENABLE_IR");
#endif

#ifdef EASY_PAIR
    WICED_BT_TRACE("\nENABLE_EASY_PAIR");
#endif

#ifdef SUPPORTING_FINDME
    WICED_BT_TRACE("\nENABLE_FINDME");
#endif

#ifdef FASTPAIR_ENABLE
    WICED_BT_TRACE("\nFASTPAIR_ENABLE");
#endif

}
